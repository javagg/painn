use glam::{Mat4, Vec3, Vec4};
use crate::cad;
use truck_polymesh::PolygonMesh;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

impl Vertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct GridVertex {
    pub(crate) position: [f32; 3],
}

impl GridVertex {
    const ATTRS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![0 => Float32x3];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GridVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct AxesVertex {
    pub(crate) position: [f32; 3],
    pub(crate) color: [f32; 3],
}

impl AxesVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<AxesVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}


#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScenePoint {
    pub x: f32,
    pub y: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SceneRect {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

impl SceneRect {
    pub fn new(x: f32, y: f32, width: f32, height: f32) -> Self {
        Self { x, y, width, height }
    }
}

pub(crate) struct Camera {
    pub(crate) eye: Vec3,
    pub(crate) forward: Vec3,
    pub(crate) right: Vec3,
    pub(crate) up: Vec3,
    pub(crate) aspect: f32,
    pub(crate) fovy: f32,
    pub(crate) mode: CameraMode,
    pub(crate) ortho_half_h: f32,
    pub(crate) near: f32,
    pub(crate) far: f32,
}

pub(crate) fn camera_from_params(
    target: Vec3,
    distance: f32,
    yaw: f32,
    pitch: f32,
    bounds: SceneRect,
    mode: CameraMode,
) -> Camera {
    let aspect = if bounds.height > 1.0 {
        bounds.width / bounds.height
    } else {
        1.0
    };

    let fovy = 45.0_f32.to_radians();
    let near = 0.02_f32;
    let far = 500.0_f32;
    let distance = distance.clamp(0.1, 200.0);
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let offset = Vec3::new(distance * cp * sy, distance * sp, distance * cp * cy);
    let eye = target + offset;
    let forward = (target - eye).normalize_or_zero();
    let world_up = Vec3::Y;
    let right = forward.cross(world_up).normalize_or_zero();
    let up = right.cross(forward);
    let ortho_half_h = match mode {
        CameraMode::Perspective => (0.5 * fovy).tan() * distance,
        CameraMode::Orthographic => distance.max(0.1),
    };

    Camera {
        eye,
        forward,
        right,
        up,
        aspect,
        fovy,
        mode,
        ortho_half_h,
        near,
        far,
    }
}

pub(crate) fn ray_from_cursor(
    cursor: ScenePoint,
    bounds: SceneRect,
    camera: &Camera,
) -> Option<(Vec3, Vec3)> {
    if bounds.width <= 1.0 || bounds.height <= 1.0 {
        return None;
    }

    let x_ndc = (cursor.x / bounds.width) * 2.0 - 1.0;
    let y_ndc = 1.0 - (cursor.y / bounds.height) * 2.0;

    match camera.mode {
        CameraMode::Perspective => {
            let half_h = (0.5 * camera.fovy).tan();
            let half_w = half_h * camera.aspect;

            let dir = (camera.forward
                + camera.right * (x_ndc * half_w)
                + camera.up * (y_ndc * half_h))
                .normalize_or_zero();

            Some((camera.eye, dir))
        }
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            let origin = camera.eye
                + camera.right * (x_ndc * half_w)
                + camera.up * (y_ndc * half_h);
            Some((origin, camera.forward))
        }
    }
}

pub(crate) fn intersect_plane(plane: GridPlane, origin: Vec3, dir: Vec3) -> Option<Vec3> {
    let (num, denom) = match plane {
        GridPlane::XY => (-origin.z, dir.z),
        GridPlane::YZ => (-origin.x, dir.x),
        GridPlane::XZ => (-origin.y, dir.y),
    };

    if denom.abs() <= 1.0e-6 {
        return None;
    }

    let t = num / denom;
    if t <= 0.0 {
        return None;
    }

    Some(origin + dir * t)
}

pub(crate) fn entity_radius(entity: &SceneEntity) -> f32 {
    let max_size = entity.size.x.max(entity.size.y).max(entity.size.z);
    match entity.kind {
        SolidKind::Box => max_size * 0.7,
        SolidKind::Sphere => max_size * 0.5,
        SolidKind::Cylinder => max_size * 0.6,
        SolidKind::Cone => max_size * 0.6,
        SolidKind::Torus => max_size * 0.8,
    }
}

pub(crate) fn project_point(mvp: Mat4, bounds: SceneRect, p: Vec3) -> Option<ScenePoint> {
    let clip = mvp * Vec4::new(p.x, p.y, p.z, 1.0);
    if clip.w.abs() <= 1.0e-6 {
        return None;
    }

    let inv_w = 1.0 / clip.w;
    let ndc = Vec3::new(clip.x, clip.y, clip.z) * inv_w;
    if ndc.z < -1.0 || ndc.z > 1.0 {
        return None;
    }

    let x = (ndc.x * 0.5 + 0.5) * bounds.width;
    let y = (1.0 - (ndc.y * 0.5 + 0.5)) * bounds.height;
    Some(ScenePoint { x, y })
}

pub(crate) fn pick_entity(
    cursor: ScenePoint,
    bounds: SceneRect,
    camera: &Camera,
    entities: &[SceneEntity],
) -> Option<u64> {
    let view = Mat4::look_at_rh(camera.eye, camera.eye + camera.forward, Vec3::Y);
    let proj = match camera.mode {
        CameraMode::Perspective => {
            Mat4::perspective_rh(camera.fovy, camera.aspect, camera.near, camera.far)
        }
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, camera.near, camera.far)
        }
    };
    let mvp = proj * view;

    let mut best: Option<(u64, f32)> = None;
    for entity in entities {
        let Some(screen) = project_point(mvp, bounds, entity.position) else {
            continue;
        };

        let dx = screen.x - cursor.x;
        let dy = screen.y - cursor.y;
        let dist2 = dx * dx + dy * dy;

        let (dist, half_h) = match camera.mode {
            CameraMode::Perspective => {
                let to_eye = entity.position - camera.eye;
                let dist = to_eye.length().max(1.0e-6);
                let half_h = (0.5 * camera.fovy).tan() * dist;
                (dist, half_h)
            }
            CameraMode::Orthographic => (camera.ortho_half_h, camera.ortho_half_h),
        };

        let pixels_per_world = bounds.height / (2.0 * half_h);
        let radius_px = (entity_radius(entity) * pixels_per_world).max(10.0);

        if dist2 <= radius_px * radius_px {
            let score = dist2 + dist * 0.01;
            if best.map(|(_, b)| score < b).unwrap_or(true) {
                best = Some((entity.id, score));
            }
        }
    }

    best.map(|(id, _)| id)
}

fn translate_mesh(mesh: &mut PolygonMesh, offset: Vec3) {
    for p in mesh.positions_mut() {
        p.x += offset.x as f64;
        p.y += offset.y as f64;
        p.z += offset.z as f64;
    }
}

fn entity_to_mesh(entity: &SceneEntity) -> PolygonMesh {
    let size = entity.size;
    let solid = match entity.kind {
        SolidKind::Box => cad::box_solid(size.x as f64, size.y as f64, size.z as f64),
        SolidKind::Sphere => cad::sphere((size.x * 0.5) as f64),
        SolidKind::Cylinder => cad::cylinder_solid(size.y as f64, (size.x * 0.35) as f64),
        SolidKind::Cone => cad::cone_solid(size.y as f64, (size.x * 0.4) as f64),
        SolidKind::Torus => cad::torus_solid((size.x * 0.7) as f64, (size.x * 0.25) as f64),
    };

    let mut mesh = cad::to_mesh(&solid);
    translate_mesh(&mut mesh, entity.position);
    mesh
}

pub(crate) fn build_scene_mesh(entities: &[SceneEntity]) -> Option<PolygonMesh> {
    let mut iter = entities.iter();
    let Some(first) = iter.next() else {
        return None;
    };

    let mut mesh = entity_to_mesh(first);
    for entity in iter {
        let other = entity_to_mesh(entity);
        mesh.merge(other);
    }
    Some(mesh)
}

pub(crate) fn mesh_to_vertex_index(mesh: &PolygonMesh) -> (Vec<Vertex>, Vec<u32>) {
    let positions = mesh.positions();
    let has_normals = !mesh.normals().is_empty();

    let mut vertices: Vec<Vertex> = positions
        .iter()
        .map(|p| Vertex {
            position: [p.x as f32, p.y as f32, p.z as f32],
            normal: [0.0, 0.0, 1.0],
        })
        .collect();

    let mut tri_indices: Vec<u32> = Vec::new();
    tri_indices.reserve(mesh.tri_faces().len() * 3);

    for tri in mesh.tri_faces() {
        tri_indices.extend_from_slice(&[
            tri[0].pos as u32,
            tri[1].pos as u32,
            tri[2].pos as u32,
        ]);
    }
    for quad in mesh.quad_faces() {
        let a = quad[0].pos as u32;
        let b = quad[1].pos as u32;
        let c = quad[2].pos as u32;
        let d = quad[3].pos as u32;
        tri_indices.extend_from_slice(&[a, b, c, a, c, d]);
    }
    for face in mesh.other_faces() {
        if face.len() < 3 {
            continue;
        }
        let a = face[0].pos as u32;
        for i in 1..(face.len() - 1) {
            let b = face[i].pos as u32;
            let c = face[i + 1].pos as u32;
            tri_indices.extend_from_slice(&[a, b, c]);
        }
    }

    let mut normal_sums = vec![Vec3::ZERO; vertices.len()];
    let mesh_normals = mesh.normals();

    if has_normals {
        for tri in mesh.tri_faces() {
            for v in tri.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos] += Vec3::new(n.x as f32, n.y as f32, n.z as f32);
                }
            }
        }
        for quad in mesh.quad_faces() {
            for v in quad.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos] += Vec3::new(n.x as f32, n.y as f32, n.z as f32);
                }
            }
        }
        for face in mesh.other_faces() {
            for v in face.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos] += Vec3::new(n.x as f32, n.y as f32, n.z as f32);
                }
            }
        }
    } else {
        for tri in tri_indices.chunks_exact(3) {
            let ia = tri[0] as usize;
            let ib = tri[1] as usize;
            let ic = tri[2] as usize;

            let a = Vec3::from_array(vertices[ia].position);
            let b = Vec3::from_array(vertices[ib].position);
            let c = Vec3::from_array(vertices[ic].position);

            let n = (b - a).cross(c - a);

            normal_sums[ia] += n;
            normal_sums[ib] += n;
            normal_sums[ic] += n;
        }
    }

    for (v, ns) in vertices.iter_mut().zip(normal_sums.into_iter()) {
        v.normal = ns.normalize_or_zero().to_array();
        if v.normal == [0.0, 0.0, 0.0] {
            v.normal = [0.0, 0.0, 1.0];
        }
    }

    (vertices, tri_indices)
}

pub(crate) fn build_grid_vertices(plane: GridPlane, extent: f32, step: f32) -> Vec<GridVertex> {
    let steps = (extent / step).round() as i32;
    let mut out = Vec::new();

    for i in -steps..=steps {
        let v = i as f32 * step;
        match plane {
            GridPlane::XY => {
                out.push(GridVertex {
                    position: [-extent, v, 0.0],
                });
                out.push(GridVertex {
                    position: [extent, v, 0.0],
                });
                out.push(GridVertex {
                    position: [v, -extent, 0.0],
                });
                out.push(GridVertex {
                    position: [v, extent, 0.0],
                });
            }
            GridPlane::YZ => {
                out.push(GridVertex {
                    position: [0.0, -extent, v],
                });
                out.push(GridVertex {
                    position: [0.0, extent, v],
                });
                out.push(GridVertex {
                    position: [0.0, v, -extent],
                });
                out.push(GridVertex {
                    position: [0.0, v, extent],
                });
            }
            GridPlane::XZ => {
                out.push(GridVertex {
                    position: [-extent, 0.0, v],
                });
                out.push(GridVertex {
                    position: [extent, 0.0, v],
                });
                out.push(GridVertex {
                    position: [v, 0.0, -extent],
                });
                out.push(GridVertex {
                    position: [v, 0.0, extent],
                });
            }
        }
    }

    out
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct Uniforms {
    pub(crate) model_view: [[f32; 4]; 4],
    pub(crate) mvp: [[f32; 4]; 4],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridPlane {
    XY,
    YZ,
    XZ,
}

impl GridPlane {
    pub const ALL: [GridPlane; 3] = [GridPlane::XY, GridPlane::YZ, GridPlane::XZ];

    pub fn label(self) -> &'static str {
        match self {
            GridPlane::XY => "XY",
            GridPlane::YZ => "YZ",
            GridPlane::XZ => "XZ",
        }
    }
}

impl std::fmt::Display for GridPlane {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

impl Default for GridPlane {
    fn default() -> Self {
        GridPlane::XZ
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraPreset {
    Top,
    Bottom,
    Front,
    Back,
    Left,
    Right,
    Iso,
    IsoLeft,
    IsoRight,
}

impl CameraPreset {
    pub const ALL: [CameraPreset; 9] = [
        CameraPreset::Top,
        CameraPreset::Bottom,
        CameraPreset::Front,
        CameraPreset::Back,
        CameraPreset::Left,
        CameraPreset::Right,
        CameraPreset::Iso,
        CameraPreset::IsoLeft,
        CameraPreset::IsoRight,
    ];

    pub fn label(self) -> &'static str {
        match self {
            CameraPreset::Top => "Top",
            CameraPreset::Bottom => "Bottom",
            CameraPreset::Front => "Front",
            CameraPreset::Back => "Back",
            CameraPreset::Left => "Left",
            CameraPreset::Right => "Right",
            CameraPreset::Iso => "Isometric",
            CameraPreset::IsoLeft => "Isometric Left",
            CameraPreset::IsoRight => "Isometric Right",
        }
    }
}

impl std::fmt::Display for CameraPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

pub(crate) fn preset_angles(preset: CameraPreset) -> (f32, f32) {
    match preset {
        CameraPreset::Top => (0.0, 1.55),
        CameraPreset::Bottom => (0.0, -1.55),
        CameraPreset::Front => (0.0, 0.0),
        CameraPreset::Back => (std::f32::consts::PI, 0.0),
        CameraPreset::Left => (-std::f32::consts::FRAC_PI_2, 0.0),
        CameraPreset::Right => (std::f32::consts::FRAC_PI_2, 0.0),
        CameraPreset::Iso => (
            std::f32::consts::FRAC_PI_4,
            35.264_f32.to_radians(),
        ),
        CameraPreset::IsoLeft => (
            -std::f32::consts::FRAC_PI_4,
            35.264_f32.to_radians(),
        ),
        CameraPreset::IsoRight => (
            std::f32::consts::FRAC_PI_4,
            35.264_f32.to_radians(),
        ),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    Perspective,
    Orthographic,
}

impl CameraMode {
    pub const ALL: [CameraMode; 2] = [CameraMode::Orthographic, CameraMode::Perspective,];

    pub fn label(self) -> &'static str {
        match self {
            CameraMode::Orthographic => "Orthographic",
            CameraMode::Perspective => "Perspective",
        }
    }
}

impl std::fmt::Display for CameraMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolidKind {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Torus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneTool {
    Select,
    Box,
    Sphere,
    Cylinder,
    Cone,
    Torus,
}

impl SceneTool {
    pub const ALL: [SceneTool; 6] = [
        SceneTool::Select,
        SceneTool::Box,
        SceneTool::Sphere,
        SceneTool::Cylinder,
        SceneTool::Cone,
        SceneTool::Torus,
    ];

    pub fn label(self) -> &'static str {
        match self {
            SceneTool::Select => "Select",
            SceneTool::Box => "Box",
            SceneTool::Sphere => "Sphere",
            SceneTool::Cylinder => "Cylinder",
            SceneTool::Cone => "Cone",
            SceneTool::Torus => "Torus",
        }
    }

    pub fn create_kind(self) -> Option<SolidKind> {
        match self {
            SceneTool::Select => None,
            SceneTool::Box => Some(SolidKind::Box),
            SceneTool::Sphere => Some(SolidKind::Sphere),
            SceneTool::Cylinder => Some(SolidKind::Cylinder),
            SceneTool::Cone => Some(SolidKind::Cone),
            SceneTool::Torus => Some(SolidKind::Torus),
        }
    }
}

impl std::fmt::Display for SceneTool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone)]
pub struct SceneEntity {
    pub(crate) id: u64,
    pub(crate) kind: SolidKind,
    pub(crate) position: Vec3,
    pub(crate) size: Vec3,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct SceneEntityInfo {
    pub id: u64,
    pub kind: SolidKind,
    pub position: [f32; 3],
    pub size: [f32; 3],
}

impl From<&SceneEntity> for SceneEntityInfo {
    fn from(e: &SceneEntity) -> Self {
        Self {
            id: e.id,
            kind: e.kind,
            position: e.position.to_array(),
            size: e.size.to_array(),
        }
    }
}
