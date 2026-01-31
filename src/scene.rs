use iced::widget::shader::{self, Viewport};
use iced::{keyboard, mouse, Element, Event, Length, Point, Rectangle};
use iced::wgpu;
use wgpu::util::DeviceExt;

use crate::cad;
use truck_polymesh::PolygonMesh;
use std::sync::Arc;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

impl Vertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GridVertex {
    position: [f32; 3],
}

impl GridVertex {
    const ATTRS: [wgpu::VertexAttribute; 1] = wgpu::vertex_attr_array![0 => Float32x3];

    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GridVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AxesVertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl AxesVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<AxesVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

#[inline]
fn mat4_mul(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0_f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = (0..4).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

#[inline]
fn mat4_identity() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[inline]
fn mat4_mul_vec4(m: [[f32; 4]; 4], v: [f32; 4]) -> [f32; 4] {
    let mut out = [0.0_f32; 4];
    for r in 0..4 {
        out[r] = m[0][r] * v[0] + m[1][r] * v[1] + m[2][r] * v[2] + m[3][r] * v[3];
    }
    out
}

type Vec3 = [f32; 3];

#[inline]
fn vec3_add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn vec3_sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn vec3_mul(a: Vec3, s: f32) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
fn vec3_dot(a: Vec3, b: Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn vec3_cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
fn vec3_normalize(v: Vec3) -> Vec3 {
    let len2 = vec3_dot(v, v);
    if len2 > 1.0e-12 {
        let inv = 1.0 / len2.sqrt();
        vec3_mul(v, inv)
    } else {
        [0.0, 0.0, 0.0]
    }
}

struct Camera {
    eye: Vec3,
    forward: Vec3,
    right: Vec3,
    up: Vec3,
    aspect: f32,
    fovy: f32,
    mode: CameraMode,
    ortho_half_h: f32,
}

fn camera_from_state(state: &SceneState, bounds: Rectangle, mode: CameraMode) -> Camera {
    camera_from_params(state.target, state.distance, state.yaw, state.pitch, bounds, mode)
}

fn camera_from_params(
    target: Vec3,
    distance: f32,
    yaw: f32,
    pitch: f32,
    bounds: Rectangle,
    mode: CameraMode,
) -> Camera {
    let aspect = if bounds.height > 1.0 {
        bounds.width / bounds.height
    } else {
        1.0
    };

    let fovy = 45.0_f32.to_radians();
    let distance = distance.clamp(0.1, 200.0);
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let offset = [distance * cp * sy, distance * sp, distance * cp * cy];
    let eye = vec3_add(target, offset);
    let forward = vec3_normalize(vec3_sub(target, eye));
    let world_up = [0.0, 1.0, 0.0];
    let right = vec3_normalize(vec3_cross(forward, world_up));
    let up = vec3_cross(right, forward);
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
    }
}

fn ray_from_cursor(cursor: Point, bounds: Rectangle, camera: &Camera) -> Option<(Vec3, Vec3)> {
    if bounds.width <= 1.0 || bounds.height <= 1.0 {
        return None;
    }

    let x_ndc = (cursor.x / bounds.width) * 2.0 - 1.0;
    let y_ndc = 1.0 - (cursor.y / bounds.height) * 2.0;

    match camera.mode {
        CameraMode::Perspective => {
            let half_h = (0.5 * camera.fovy).tan();
            let half_w = half_h * camera.aspect;

            let dir = vec3_normalize(vec3_add(
                camera.forward,
                vec3_add(
                    vec3_mul(camera.right, x_ndc * half_w),
                    vec3_mul(camera.up, y_ndc * half_h),
                ),
            ));

            Some((camera.eye, dir))
        }
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            let origin = vec3_add(
                camera.eye,
                vec3_add(
                    vec3_mul(camera.right, x_ndc * half_w),
                    vec3_mul(camera.up, y_ndc * half_h),
                ),
            );
            Some((origin, camera.forward))
        }
    }
}

fn intersect_plane(plane: GridPlane, origin: Vec3, dir: Vec3) -> Option<Vec3> {
    let (num, denom) = match plane {
        GridPlane::XY => (-origin[2], dir[2]),
        GridPlane::YZ => (-origin[0], dir[0]),
        GridPlane::XZ => (-origin[1], dir[1]),
    };

    if denom.abs() <= 1.0e-6 {
        return None;
    }

    let t = num / denom;
    if t <= 0.0 {
        return None;
    }

    Some(vec3_add(origin, vec3_mul(dir, t)))
}

fn entity_radius(entity: &SceneEntity) -> f32 {
    match entity.kind {
        SolidKind::Box => entity.size * 0.7,
        SolidKind::Sphere => entity.size * 0.5,
        SolidKind::Cylinder => entity.size * 0.6,
        SolidKind::Cone => entity.size * 0.6,
        SolidKind::Torus => entity.size * 0.8,
    }
}

fn project_point(mvp: [[f32; 4]; 4], bounds: Rectangle, p: Vec3) -> Option<Point> {
    let clip = mat4_mul_vec4(mvp, [p[0], p[1], p[2], 1.0]);
    if clip[3].abs() <= 1.0e-6 {
        return None;
    }

    let inv_w = 1.0 / clip[3];
    let ndc = [clip[0] * inv_w, clip[1] * inv_w, clip[2] * inv_w];
    if ndc[2] < -1.0 || ndc[2] > 1.0 {
        return None;
    }

    let x = (ndc[0] * 0.5 + 0.5) * bounds.width;
    let y = (1.0 - (ndc[1] * 0.5 + 0.5)) * bounds.height;
    Some(Point { x, y })
}

fn pick_entity(
    cursor: Point,
    bounds: Rectangle,
    camera: &Camera,
    entities: &[SceneEntity],
) -> Option<u64> {
    let view = mat4_look_at_rh(camera.eye, vec3_add(camera.eye, camera.forward), [0.0, 1.0, 0.0]);
    let proj = match camera.mode {
        CameraMode::Perspective => mat4_perspective(camera.aspect, camera.fovy, 0.02, 500.0),
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            mat4_orthographic(-half_w, half_w, -half_h, half_h, 0.02, 500.0)
        }
    };
    let mvp = mat4_mul(proj, view);

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
                let to_eye = vec3_sub(entity.position, camera.eye);
                let dist = vec3_dot(to_eye, to_eye).sqrt().max(1.0e-6);
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
        p.x += offset[0] as f64;
        p.y += offset[1] as f64;
        p.z += offset[2] as f64;
    }
}

fn entity_to_mesh(entity: &SceneEntity) -> PolygonMesh {
    let size = entity.size as f64;
    let solid = match entity.kind {
        SolidKind::Box => cad::box_solid(size, size, size),
        SolidKind::Sphere => cad::sphere(size * 0.5),
        SolidKind::Cylinder => cad::cylinder_solid(size, size * 0.35),
        SolidKind::Cone => cad::cone_solid(size, size * 0.4),
        SolidKind::Torus => cad::torus_solid(size * 0.7, size * 0.25),
    };

    let mut mesh = cad::to_mesh(&solid);
    translate_mesh(&mut mesh, entity.position);
    mesh
}

fn build_scene_mesh(entities: &[SceneEntity]) -> Option<PolygonMesh> {
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

fn mesh_to_vertex_index(mesh: &PolygonMesh) -> (Vec<Vertex>, Vec<u32>) {
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

    let mut normal_sums = vec![[0.0_f32; 3]; vertices.len()];
    let mesh_normals = mesh.normals();

    if has_normals {
        for tri in mesh.tri_faces() {
            for v in tri.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos][0] += n.x as f32;
                    normal_sums[v.pos][1] += n.y as f32;
                    normal_sums[v.pos][2] += n.z as f32;
                }
            }
        }
        for quad in mesh.quad_faces() {
            for v in quad.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos][0] += n.x as f32;
                    normal_sums[v.pos][1] += n.y as f32;
                    normal_sums[v.pos][2] += n.z as f32;
                }
            }
        }
        for face in mesh.other_faces() {
            for v in face.iter() {
                if let Some(nor) = v.nor {
                    let n = mesh_normals[nor];
                    normal_sums[v.pos][0] += n.x as f32;
                    normal_sums[v.pos][1] += n.y as f32;
                    normal_sums[v.pos][2] += n.z as f32;
                }
            }
        }
    } else {
        for tri in tri_indices.chunks_exact(3) {
            let ia = tri[0] as usize;
            let ib = tri[1] as usize;
            let ic = tri[2] as usize;

            let a = vertices[ia].position;
            let b = vertices[ib].position;
            let c = vertices[ic].position;

            let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];

            let n = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];

            normal_sums[ia][0] += n[0];
            normal_sums[ia][1] += n[1];
            normal_sums[ia][2] += n[2];

            normal_sums[ib][0] += n[0];
            normal_sums[ib][1] += n[1];
            normal_sums[ib][2] += n[2];

            normal_sums[ic][0] += n[0];
            normal_sums[ic][1] += n[1];
            normal_sums[ic][2] += n[2];
        }
    }

    for (v, ns) in vertices.iter_mut().zip(normal_sums.into_iter()) {
        let len = (ns[0] * ns[0] + ns[1] * ns[1] + ns[2] * ns[2]).sqrt();
        if len > 1.0e-9 {
            v.normal = [ns[0] / len, ns[1] / len, ns[2] / len];
        } else {
            v.normal = [0.0, 0.0, 1.0];
        }
    }

    (vertices, tri_indices)
}

#[inline]
fn mat4_perspective(aspect: f32, fovy_radians: f32, z_near: f32, z_far: f32) -> [[f32; 4]; 4] {
    let aspect = aspect.max(1.0e-6);
    let f = 1.0 / (0.5 * fovy_radians).tan();
    let nf = 1.0 / (z_near - z_far);

    // Right-handed, depth range 0..1 (wgpu).
    [
        [f / aspect, 0.0, 0.0, 0.0],
        [0.0, f, 0.0, 0.0],
        [0.0, 0.0, z_far * nf, -1.0],
        [0.0, 0.0, (z_near * z_far) * nf, 0.0],
    ]
}

fn mat4_orthographic(
    left: f32,
    right: f32,
    bottom: f32,
    top: f32,
    z_near: f32,
    z_far: f32,
) -> [[f32; 4]; 4] {
    let rml = right - left;
    let tmb = top - bottom;
    let fmn = z_far - z_near;

    if rml.abs() <= 1.0e-6 || tmb.abs() <= 1.0e-6 || fmn.abs() <= 1.0e-6 {
        return mat4_identity();
    }

    let tx = -(right + left) / rml;
    let ty = -(top + bottom) / tmb;
    let tz = -z_near / fmn;

    [
        [2.0 / rml, 0.0, 0.0, 0.0],
        [0.0, 2.0 / tmb, 0.0, 0.0],
        [0.0, 0.0, -1.0 / fmn, 0.0],
        [tx, ty, tz, 1.0],
    ]
}

#[inline]
fn mat4_look_at_rh(eye: Vec3, target: Vec3, up: Vec3) -> [[f32; 4]; 4] {
    let f = vec3_normalize(vec3_sub(target, eye));
    let s = vec3_normalize(vec3_cross(f, up));
    let u = vec3_cross(s, f);

    // Column-major.
    let col0 = [s[0], s[1], s[2], 0.0];
    let col1 = [u[0], u[1], u[2], 0.0];
    let col2 = [-f[0], -f[1], -f[2], 0.0];
    let col3 = [
        -vec3_dot(s, eye),
        -vec3_dot(u, eye),
        vec3_dot(f, eye),
        1.0,
    ];

    [col0, col1, col2, col3]
}

fn build_grid_vertices(plane: GridPlane, extent: f32, step: f32) -> Vec<GridVertex> {
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
struct Uniforms {
    model_view: [[f32; 4]; 4],
    mvp: [[f32; 4]; 4],
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
struct SceneEntity {
    id: u64,
    kind: SolidKind,
    position: Vec3,
    size: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct GridKey {
    enabled: bool,
    plane: GridPlane,
    extent: f32,
    step: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PreviewKey {
    start: Vec3,
    end: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct HighlightKey {
    id: u64,
    position: Vec3,
    size: f32,
}

#[derive(Debug)]
pub struct Pipeline {
    pipeline: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
    grid_pipeline: wgpu::RenderPipeline,
    grid_vertices: wgpu::Buffer,
    grid_vertex_count: u32,
    grid_key: Option<GridKey>,
    preview_pipeline: wgpu::RenderPipeline,
    preview_vertices: wgpu::Buffer,
    preview_vertex_count: u32,
    preview_key: Option<PreviewKey>,
    preview_version: u64,
    highlight_pipeline: wgpu::RenderPipeline,
    highlight_vertices: wgpu::Buffer,
    highlight_vertex_count: u32,
    highlight_key: Option<HighlightKey>,
    entities_version: u64,
    uniforms: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    depth: wgpu::TextureView,
    depth_size: (u32, u32),
    last_bounds: (f32, f32, f32, f32),
    // Axes overlay
    axes_pipeline: wgpu::RenderPipeline,
    axes_vertices: wgpu::Buffer,
    axes_vertex_count: u32,
}

impl shader::Pipeline for Pipeline {
    fn new(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
    ) -> Self {
        use wgpu::util::DeviceExt;

        const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_mesh_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    model_view: mat4x4<f32>,
    mvp: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = uniforms.mvp * vec4<f32>(in.position, 1.0);
    let nmat = mat3x3<f32>(
        uniforms.model_view[0].xyz,
        uniforms.model_view[1].xyz,
        uniforms.model_view[2].xyz,
    );
    out.normal = normalize(nmat * in.normal);
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let ambient = 0.18;
    let light_dir = normalize(vec3<f32>(0.3, 0.6, 0.8));
    let ndotl = max(dot(normalize(in.normal), light_dir), 0.0);

    let base = vec3<f32>(0.88, 0.90, 0.96);
    let color = base * (ambient + (1.0 - ambient) * ndotl);

    return vec4<f32>(color, 1.0);
}
"#
                    .into(),
            ),
        });

        let grid_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_grid_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    model_view: mat4x4<f32>,
    mvp: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = uniforms.mvp * vec4<f32>(in.position, 1.0);
    return out;
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return vec4<f32>(0.55, 0.58, 0.66, 0.55);
}
"#
                .into(),
            ),
        });

        let preview_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_preview_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    model_view: mat4x4<f32>,
    mvp: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = uniforms.mvp * vec4<f32>(in.position, 1.0);
    return out;
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return vec4<f32>(0.95, 0.82, 0.2, 0.9);
}
"#
                .into(),
            ),
        });

        let highlight_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_highlight_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    model_view: mat4x4<f32>,
    mvp: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = uniforms.mvp * vec4<f32>(in.position, 1.0);
    return out;
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return vec4<f32>(0.2, 0.9, 0.95, 0.9);
}
"#
                .into(),
            ),
        });

        let bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("scene_triangle_bind_group_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let initial_uniforms = Uniforms {
            model_view: mat4_identity(),
            mvp: mat4_identity(),
        };
        let uniforms = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_mesh_uniforms"),
            contents: bytemuck::bytes_of(&initial_uniforms),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scene_mesh_bind_group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms.as_entire_binding(),
            }],
        });

        let solid = cad::bottle(2.0, 1.2, 0.8);
        let mesh = cad::to_mesh(&solid);
        let positions = mesh.positions();
        let has_normals = !mesh.normals().is_empty();

        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for p in positions {
            min[0] = min[0].min(p.x);
            min[1] = min[1].min(p.y);
            min[2] = min[2].min(p.z);
            max[0] = max[0].max(p.x);
            max[1] = max[1].max(p.y);
            max[2] = max[2].max(p.z);
        }

        let center = [
            (min[0] + max[0]) * 0.5,
            (min[1] + max[1]) * 0.5,
            (min[2] + max[2]) * 0.5,
        ];
        let extent = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
        let max_extent = extent
            .into_iter()
            .fold(0.0_f64, |acc, v| acc.max(v))
            .max(1.0e-9);
        let scale = 1.8 / max_extent;

        let vertices: Vec<Vertex> = positions
            .iter()
            .map(|p| Vertex {
                position: [
                    ((p.x - center[0]) * scale) as f32,
                    ((p.y - center[1]) * scale) as f32,
                    ((p.z - center[2]) * scale) as f32,
                ],
                normal: [0.0, 0.0, 1.0],
            })
            .collect();

        // Build triangle indices (tri/quad/ngon fan) and compute smooth per-position normals.
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

        let mut normal_sums = vec![[0.0_f32; 3]; vertices.len()];
        let mesh_normals = mesh.normals();

        if has_normals {
            // Prefer OBJ normals if present.
            for tri in mesh.tri_faces() {
                for v in tri.iter() {
                    if let Some(nor) = v.nor {
                        let n = mesh_normals[nor];
                        normal_sums[v.pos][0] += n.x as f32;
                        normal_sums[v.pos][1] += n.y as f32;
                        normal_sums[v.pos][2] += n.z as f32;
                    }
                }
            }
            for quad in mesh.quad_faces() {
                for v in quad.iter() {
                    if let Some(nor) = v.nor {
                        let n = mesh_normals[nor];
                        normal_sums[v.pos][0] += n.x as f32;
                        normal_sums[v.pos][1] += n.y as f32;
                        normal_sums[v.pos][2] += n.z as f32;
                    }
                }
            }
            for face in mesh.other_faces() {
                for v in face.iter() {
                    if let Some(nor) = v.nor {
                        let n = mesh_normals[nor];
                        normal_sums[v.pos][0] += n.x as f32;
                        normal_sums[v.pos][1] += n.y as f32;
                        normal_sums[v.pos][2] += n.z as f32;
                    }
                }
            }
        } else {
            // Fallback: compute smooth normals from triangle geometry.
            for tri in tri_indices.chunks_exact(3) {
                let ia = tri[0] as usize;
                let ib = tri[1] as usize;
                let ic = tri[2] as usize;

                let a = vertices[ia].position;
                let b = vertices[ib].position;
                let c = vertices[ic].position;

                let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];

                let n = [
                    ab[1] * ac[2] - ab[2] * ac[1],
                    ab[2] * ac[0] - ab[0] * ac[2],
                    ab[0] * ac[1] - ab[1] * ac[0],
                ];

                normal_sums[ia][0] += n[0];
                normal_sums[ia][1] += n[1];
                normal_sums[ia][2] += n[2];

                normal_sums[ib][0] += n[0];
                normal_sums[ib][1] += n[1];
                normal_sums[ib][2] += n[2];

                normal_sums[ic][0] += n[0];
                normal_sums[ic][1] += n[1];
                normal_sums[ic][2] += n[2];
            }
        }

        let mut vertices = vertices;
        for (v, ns) in vertices.iter_mut().zip(normal_sums.into_iter()) {
            let len = (ns[0] * ns[0] + ns[1] * ns[1] + ns[2] * ns[2]).sqrt();
            if len > 1.0e-9 {
                v.normal = [ns[0] / len, ns[1] / len, ns[2] / len];
            } else {
                v.normal = [0.0, 0.0, 1.0];
            }
        }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_mesh_vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_mesh_indices"),
            contents: bytemuck::cast_slice(&tri_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene_mesh_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_mesh_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[Vertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview: None,
            cache: None,
        });

        let grid_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_grid_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &grid_shader,
                entry_point: Some("vs_main"),
                buffers: &[GridVertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &grid_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview: None,
            cache: None,
        });

        let preview_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_preview_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &preview_shader,
                entry_point: Some("vs_main"),
                buffers: &[GridVertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &preview_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview: None,
            cache: None,
        });

        let highlight_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_highlight_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &highlight_shader,
                entry_point: Some("vs_main"),
                buffers: &[GridVertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &highlight_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview: None,
            cache: None,
        });

        // Axes overlay pipeline (no uniforms, colored lines in clip space)
        let axes_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_axes_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = vec4<f32>(in.position, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
"#
                    .into(),
            ),
        });

        let axes_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene_axes_pipeline_layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });

        let axes_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_axes_pipeline"),
            layout: Some(&axes_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &axes_shader,
                entry_point: Some("vs_main"),
                buffers: &[AxesVertex::layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &axes_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            multiview: None,
            cache: None,
        });

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("scene_depth"),
            size: wgpu::Extent3d {
                width: 1,
                height: 1,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        Self {
            pipeline,
            vertices: vertex_buffer,
            indices: index_buffer,
            index_count: 0,
            grid_pipeline,
            grid_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_grid_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            grid_vertex_count: 0,
            grid_key: None,
            preview_pipeline,
            preview_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_preview_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            preview_vertex_count: 0,
            preview_key: None,
            preview_version: 0,
            highlight_pipeline,
            highlight_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_highlight_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            highlight_vertex_count: 0,
            highlight_key: None,
            entities_version: 0,
            uniforms,
            bind_group,
            depth,
            depth_size: (1, 1),
            last_bounds: (0.0, 0.0, 1.0, 1.0),
            axes_pipeline,
            axes_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_axes_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            axes_vertex_count: 0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Primitive {
    target: [f32; 3],
    distance: f32,
    yaw: f32,
    pitch: f32,
    camera_mode: CameraMode,
    show_grid: bool,
    grid_plane: GridPlane,
    grid_extent: f32,
    grid_step: f32,
    entities: Arc<Vec<SceneEntity>>,
    entities_version: u64,
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
    selected: Option<u64>,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        // Ensure the depth buffer matches the swapchain (frame) size.
        let physical = viewport.physical_size();
        let target_w = physical.width.max(1);
        let target_h = physical.height.max(1);

        if pipeline.depth_size != (target_w, target_h) {
            let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("scene_depth"),
                size: wgpu::Extent3d {
                    width: target_w,
                    height: target_h,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth24Plus,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            pipeline.depth = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());
            pipeline.depth_size = (target_w, target_h);
        }

        // Store the widget bounds in physical pixels so we can set the viewport in `render`.
        let scale = viewport.scale_factor();
        pipeline.last_bounds = (
            bounds.x * scale,
            bounds.y * scale,
            (bounds.width * scale).max(1.0),
            (bounds.height * scale).max(1.0),
        );

        // CAD-like orbit camera around a target point.
        let camera = camera_from_params(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            *bounds,
            self.camera_mode,
        );

        let view = mat4_look_at_rh(
            camera.eye,
            vec3_add(camera.eye, camera.forward),
            [0.0, 1.0, 0.0],
        );

        let proj = match camera.mode {
            CameraMode::Perspective => mat4_perspective(camera.aspect, camera.fovy, 0.02, 500.0),
            CameraMode::Orthographic => {
                let half_h = camera.ortho_half_h;
                let half_w = half_h * camera.aspect;
                mat4_orthographic(-half_w, half_w, -half_h, half_h, 0.02, 500.0)
            }
        };

        let model = mat4_identity();
        let model_view = mat4_mul(view, model);
        let mvp = mat4_mul(proj, model_view);

        let uniforms = Uniforms { model_view, mvp };
        queue.write_buffer(&pipeline.uniforms, 0, bytemuck::bytes_of(&uniforms));

        // Build axes overlay vertices in clip space (-1..1) for a mini viewport.
        // Project world axes onto camera's screen axes using camera.right/up.
        let len = 0.9_f32;
        let mut axes_vertices: Vec<AxesVertex> = Vec::with_capacity(32);

        let mut add_axis_with_label = |v: Vec3, color: [f32; 3], ch: char| {
            let x = vec3_dot(v, camera.right);
            let y = vec3_dot(v, camera.up);
            let ex = x * len;
            let ey = y * len;

            // Axis line from origin to endpoint in clip space
            axes_vertices.push(AxesVertex { position: [0.0, 0.0, 0.0], color });
            axes_vertices.push(AxesVertex { position: [ex, ey, 0.0], color });

            // Label near endpoint, screen-aligned, small size in clip units
            let mut nx = x;
            let mut ny = y;
            let mag = (nx * nx + ny * ny).sqrt();
            if mag > 1.0e-4 { nx /= mag; ny /= mag; } else { nx = 1.0; ny = 0.0; }
            let pad = 0.08_f32; // clip units (~8px at 100px viewport)
            let px = ex + nx * pad;
            let py = ey + ny * pad;
            let s = 0.12_f32; // glyph half-size in clip units

            match ch {
                'X' => {
                    axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px + s, py - s, 0.0], color });
                }
                'Y' => {
                    // upper-left to center, upper-right to center, center to bottom
                    axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                    axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                    axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                    axes_vertices.push(AxesVertex { position: [px,     py - s, 0.0], color });
                }
                'Z' => {
                    // top horizontal
                    axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                    // diagonal
                    axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                    // bottom horizontal
                    axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                    axes_vertices.push(AxesVertex { position: [px + s, py - s, 0.0], color });
                }
                _ => {}
            }
        };

        add_axis_with_label([1.0, 0.0, 0.0], [0.95, 0.3, 0.3], 'X'); // X - red
        add_axis_with_label([0.0, 1.0, 0.0], [0.4, 0.9, 0.5], 'Y');   // Y - green
        add_axis_with_label([0.0, 0.0, 1.0], [0.35, 0.6, 0.95], 'Z'); // Z - blue

        pipeline.axes_vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_axes_vertices"),
            contents: bytemuck::cast_slice(&axes_vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        pipeline.axes_vertex_count = axes_vertices.len() as u32;

        let grid_key = GridKey {
            enabled: self.show_grid,
            plane: self.grid_plane,
            extent: self.grid_extent,
            step: self.grid_step,
        };
        if pipeline.grid_key != Some(grid_key) {
            pipeline.grid_key = Some(grid_key);
            if !self.show_grid {
                pipeline.grid_vertex_count = 0;
            } else {
                let extent = self.grid_extent.max(0.5);
                let step = self.grid_step.max(0.05);
                let vertices = build_grid_vertices(self.grid_plane, extent, step);

                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("scene_grid_vertices"),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                pipeline.grid_vertices = buffer;
                pipeline.grid_vertex_count = vertices.len() as u32;
            }
        }

        if pipeline.entities_version != self.entities_version {
            pipeline.entities_version = self.entities_version;

            if let Some(mesh) = build_scene_mesh(self.entities.as_slice()) {
                let (vertices, indices) = mesh_to_vertex_index(&mesh);

                pipeline.vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("scene_mesh_vertices"),
                    contents: bytemuck::cast_slice(&vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });

                pipeline.indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("scene_mesh_indices"),
                    contents: bytemuck::cast_slice(&indices),
                    usage: wgpu::BufferUsages::INDEX,
                });

                pipeline.index_count = indices.len() as u32;
            } else {
                pipeline.index_count = 0;
            }
        }

        if pipeline.preview_version != self.preview_version {
            pipeline.preview_version = self.preview_version;
            if let Some((start, end)) = self.preview_line {
                let key = PreviewKey { start, end };
                if pipeline.preview_key != Some(key) {
                    pipeline.preview_key = Some(key);
                    let vertices = [
                        GridVertex { position: start },
                        GridVertex { position: end },
                    ];
                    pipeline.preview_vertices =
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some("scene_preview_vertices"),
                            contents: bytemuck::cast_slice(&vertices),
                            usage: wgpu::BufferUsages::VERTEX,
                        });
                    pipeline.preview_vertex_count = 2;
                }
            } else {
                pipeline.preview_key = None;
                pipeline.preview_vertex_count = 0;
            }
        }

        let highlight_key = self.selected.and_then(|id| {
            self.entities
                .iter()
                .find(|e| e.id == id)
                .map(|e| HighlightKey {
                    id: e.id,
                    position: e.position,
                    size: e.size,
                })
        });

        if pipeline.highlight_key != highlight_key {
            pipeline.highlight_key = highlight_key;
            if let Some(key) = highlight_key {
                let s = (key.size * 0.6).max(0.05);
                let p = key.position;
                let vertices = [
                    GridVertex {
                        position: [p[0] - s, p[1], p[2]],
                    },
                    GridVertex {
                        position: [p[0] + s, p[1], p[2]],
                    },
                    GridVertex {
                        position: [p[0], p[1] - s, p[2]],
                    },
                    GridVertex {
                        position: [p[0], p[1] + s, p[2]],
                    },
                    GridVertex {
                        position: [p[0], p[1], p[2] - s],
                    },
                    GridVertex {
                        position: [p[0], p[1], p[2] + s],
                    },
                ];

                pipeline.highlight_vertices =
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("scene_highlight_vertices"),
                        contents: bytemuck::cast_slice(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    });
                pipeline.highlight_vertex_count = vertices.len() as u32;
            } else {
                pipeline.highlight_vertex_count = 0;
            }
        }
    }

    fn draw(
        &self,
        _pipeline: &Self::Pipeline,
        _render_pass: &mut wgpu::RenderPass<'_>,
    ) -> bool {
        // Use `render` so we can attach a depth buffer.
        false
    }

    fn render(
        &self,
        pipeline: &Self::Pipeline,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &Rectangle<u32>,
    ) {
        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("scene_mesh_depth_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load,
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &pipeline.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });

        let (bx, by, bw, bh) = pipeline.last_bounds;
        render_pass.set_viewport(bx, by, bw, bh, 0.0, 1.0);
        render_pass.set_scissor_rect(
            clip_bounds.x,
            clip_bounds.y,
            clip_bounds.width,
            clip_bounds.height,
        );

        if pipeline.grid_vertex_count > 0 {
            render_pass.set_pipeline(&pipeline.grid_pipeline);
            render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
            render_pass.set_vertex_buffer(0, pipeline.grid_vertices.slice(..));
            render_pass.draw(0..pipeline.grid_vertex_count, 0..1);
        }

        if pipeline.preview_vertex_count > 0 {
            render_pass.set_pipeline(&pipeline.preview_pipeline);
            render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
            render_pass.set_vertex_buffer(0, pipeline.preview_vertices.slice(..));
            render_pass.draw(0..pipeline.preview_vertex_count, 0..1);
        }

        if pipeline.highlight_vertex_count > 0 {
            render_pass.set_pipeline(&pipeline.highlight_pipeline);
            render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
            render_pass.set_vertex_buffer(0, pipeline.highlight_vertices.slice(..));
            render_pass.draw(0..pipeline.highlight_vertex_count, 0..1);
        }

        render_pass.set_pipeline(&pipeline.pipeline);
        render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
        render_pass.set_vertex_buffer(0, pipeline.vertices.slice(..));
        render_pass.set_index_buffer(pipeline.indices.slice(..), wgpu::IndexFormat::Uint32);
        if pipeline.index_count > 0 {
            render_pass.draw_indexed(0..pipeline.index_count, 0, 0..1);
        }

        // Draw axes overlay at bottom-right corner
        if self.axes_enabled && pipeline.axes_vertex_count > 0 {
            let size = self.axes_size.max(24.0); // pixels
            let margin = self.axes_margin.max(0.0);
            let (bx, by, bw, bh) = pipeline.last_bounds;
            let vx = (bx + bw - size - margin).max(bx);
            let vy = (by + bh - size - margin).max(by);
            let vw = size.min(bw);
            let vh = size.min(bh);

            render_pass.set_viewport(vx, vy, vw, vh, 0.0, 1.0);
            // Keep scissor as widget clip
            render_pass.set_pipeline(&pipeline.axes_pipeline);
            render_pass.set_vertex_buffer(0, pipeline.axes_vertices.slice(..));
            render_pass.draw(0..pipeline.axes_vertex_count, 0..1);

            // Restore full viewport for any further draws (not strictly needed here)
            render_pass.set_viewport(bx, by, bw, bh, 0.0, 1.0);
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Scene {
    show_grid: bool,
    grid_plane: GridPlane,
    grid_extent: f32,
    grid_step: f32,
    tool: SceneTool,
    camera_mode: CameraMode,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
}

#[derive(Debug, Clone)]
pub struct SceneState {
    target: [f32; 3],
    distance: f32,
    yaw: f32,
    pitch: f32,
    dragging: Dragging,
    last_cursor: Option<Point>,
    entities: Vec<SceneEntity>,
    entities_version: u64,
    selected: Option<u64>,
    next_id: u64,
    drag_start: Option<Vec3>,
    drag_offset: Vec3,
    modifiers: keyboard::Modifiers,
    sphere_center: Option<Vec3>,
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dragging {
    None,
    Pan,
    Rotate,
    MoveEntity,
    CreateEntity,
}

impl Default for SceneState {
    fn default() -> Self {
        Self {
            target: [0.0, 0.0, 0.0],
            distance: 3.0,
            yaw: 0.8,
            pitch: -0.6,
            dragging: Dragging::None,
            last_cursor: None,
            entities: Vec::new(),
            entities_version: 0,
            selected: None,
            next_id: 1,
            drag_start: None,
            drag_offset: [0.0, 0.0, 0.0],
            modifiers: keyboard::Modifiers::default(),
            sphere_center: None,
            preview_line: None,
            preview_version: 0,
        }
    }
}

impl<Message> shader::Program<Message> for Scene {
    type State = SceneState;
    type Primitive = Primitive;

    fn update(
        &self,
        state: &mut Self::State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        if let Event::Keyboard(keyboard::Event::ModifiersChanged(mods)) = event {
            state.modifiers = *mods;
        }

        if let Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) = event {
            if matches!(
                key,
                keyboard::Key::Named(keyboard::key::Named::Delete)
                    | keyboard::Key::Named(keyboard::key::Named::Backspace)
            ) {
                if let Some(selected) = state.selected {
                    let before = state.entities.len();
                    state.entities.retain(|e| e.id != selected);
                    if state.entities.len() != before {
                        state.entities_version = state.entities_version.wrapping_add(1);
                    }
                    state.selected = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
        }

        let Some(cursor_pos) = cursor.position_in(bounds) else {
            if matches!(
                event,
                Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right))
                    | Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left))
            ) {
                state.dragging = Dragging::None;
                state.last_cursor = None;
                state.drag_start = None;
                return Some(shader::Action::request_redraw().and_capture());
            }

            return None;
        };

        let camera = camera_from_state(state, bounds, self.camera_mode);

        match event {
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let scroll_y = match *delta {
                    mouse::ScrollDelta::Lines { y, .. } => y,
                    mouse::ScrollDelta::Pixels { y, .. } => y / 120.0,
                };

                if scroll_y.abs() > f32::EPSILON {
                    let factor = 1.1_f32.powf(scroll_y);
                    state.distance = (state.distance / factor).clamp(0.1, 200.0);
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                if self.tool == SceneTool::Sphere {
                    if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                            state.sphere_center = Some(hit_pos);
                            state.preview_line = Some((hit_pos, hit_pos));
                            state.preview_version = state.preview_version.wrapping_add(1);
                            state.dragging = Dragging::None;
                            state.last_cursor = Some(cursor_pos);
                            return Some(shader::Action::request_redraw().and_capture());
                        }
                    }
                }

                let hit = pick_entity(cursor_pos, bounds, &camera, &state.entities);
                if let Some(id) = hit {
                    state.selected = Some(id);
                    state.dragging = Dragging::MoveEntity;
                } else if let Some(create_kind) = self.tool.create_kind() {
                    if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                            let id = state.next_id;
                            state.next_id += 1;
                            state.entities.push(SceneEntity {
                                id,
                                kind: create_kind,
                                position: hit_pos,
                                size: 0.2,
                            });
                            state.entities_version = state.entities_version.wrapping_add(1);
                            state.selected = Some(id);
                            state.dragging = Dragging::CreateEntity;
                            state.drag_start = Some(hit_pos);
                        }
                    }
                }

                if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                    if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                        if let Some(id) = state.selected {
                            if let Some(entity) = state.entities.iter().find(|e| e.id == id) {
                                state.drag_offset = vec3_sub(entity.position, hit_pos);
                            }
                        }
                    }
                }

                state.last_cursor = Some(cursor_pos);
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if matches!(state.dragging, Dragging::MoveEntity | Dragging::CreateEntity) {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    state.drag_start = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                if self.tool == SceneTool::Sphere {
                    if let Some(center) = state.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                                let radius = vec3_dot(vec3_sub(hit_pos, center), vec3_sub(hit_pos, center))
                                    .sqrt()
                                    .max(0.05);
                                let id = state.next_id;
                                state.next_id += 1;
                                state.entities.push(SceneEntity {
                                    id,
                                    kind: SolidKind::Sphere,
                                    position: center,
                                    size: radius * 2.0,
                                });
                                state.entities_version = state.entities_version.wrapping_add(1);
                                state.selected = Some(id);
                                state.sphere_center = None;
                                state.preview_line = None;
                                state.preview_version = state.preview_version.wrapping_add(1);
                                state.last_cursor = Some(cursor_pos);
                                return Some(shader::Action::request_redraw().and_capture());
                            }
                        }
                    }
                }

                if state.modifiers.shift() {
                    state.dragging = Dragging::Pan;
                } else {
                    state.dragging = Dragging::Rotate;
                }
                state.last_cursor = Some(cursor_pos);
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                if matches!(state.dragging, Dragging::Pan | Dragging::Rotate) {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if self.tool == SceneTool::Sphere {
                    if let Some(center) = state.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                                state.preview_line = Some((center, hit_pos));
                                state.preview_version = state.preview_version.wrapping_add(1);
                                state.last_cursor = Some(cursor_pos);
                                return Some(shader::Action::request_redraw().and_capture());
                            }
                        }
                    }
                }

                match state.dragging {
                    Dragging::None => {}
                    Dragging::MoveEntity => {
                        if let Some((origin, dir)) = ray_from_cursor(cursor_pos, bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                                if let Some(id) = state.selected {
                                    if let Some(entity) = state.entities.iter_mut().find(|e| e.id == id) {
                                        entity.position = vec3_add(hit_pos, state.drag_offset);
                                        state.entities_version = state.entities_version.wrapping_add(1);
                                    }
                                }
                            }
                        }

                        state.last_cursor = Some(cursor_pos);
                        return Some(shader::Action::request_redraw().and_capture());
                    }
                    Dragging::CreateEntity => {
                        if let (Some(start), Some((origin, dir))) =
                            (state.drag_start, ray_from_cursor(cursor_pos, bounds, &camera))
                        {
                            if let Some(hit_pos) = intersect_plane(self.grid_plane, origin, dir) {
                                if let Some(id) = state.selected {
                                    if let Some(entity) = state.entities.iter_mut().find(|e| e.id == id) {
                                        let diff = vec3_sub(hit_pos, start);
                                        let dist = vec3_dot(diff, diff).sqrt().max(0.1);
                                        entity.size = dist;
                                        state.entities_version = state.entities_version.wrapping_add(1);
                                    }
                                }
                            }
                        }

                        state.last_cursor = Some(cursor_pos);
                        return Some(shader::Action::request_redraw().and_capture());
                    }
                    Dragging::Pan => {
                        if let Some(last) = state.last_cursor {
                            let dx = cursor_pos.x - last.x;
                            let dy = cursor_pos.y - last.y;

                            if bounds.width > 1.0 && bounds.height > 1.0 {
                                let dx_ndc = (dx * 2.0) / bounds.width;
                                let dy_ndc = (-dy * 2.0) / bounds.height;

                                let half_h = camera.ortho_half_h;
                                let half_w = half_h * camera.aspect;

                                let pan = vec3_add(
                                    vec3_mul(camera.right, dx_ndc * half_w),
                                    vec3_mul(camera.up, dy_ndc * half_h),
                                );
                                state.target = vec3_add(state.target, pan);
                            }

                            state.last_cursor = Some(cursor_pos);
                            return Some(shader::Action::request_redraw().and_capture());
                        } else {
                            state.last_cursor = Some(cursor_pos);
                            return Some(shader::Action::request_redraw().and_capture());
                        }
                    }
                    Dragging::Rotate => {
                        if let Some(last) = state.last_cursor {
                            let dx = cursor_pos.x - last.x;
                            let dy = cursor_pos.y - last.y;

                            let rot_speed = 2.5;
                            if bounds.width > 1.0 && bounds.height > 1.0 {
                                state.yaw += (dx / bounds.width) * rot_speed;
                                state.pitch += (dy / bounds.height) * rot_speed;
                            } else {
                                state.yaw += dx * 0.01;
                                state.pitch += dy * 0.01;
                            }

                            let max_pitch = 1.55;
                            state.pitch = state.pitch.clamp(-max_pitch, max_pitch);

                            state.last_cursor = Some(cursor_pos);
                            return Some(shader::Action::request_redraw().and_capture());
                        } else {
                            state.last_cursor = Some(cursor_pos);
                            return Some(shader::Action::request_redraw().and_capture());
                        }
                    }
                }
            }
            _ => {}
        }

        None
    }

    fn draw(&self, state: &Self::State, _cursor: mouse::Cursor, _bounds: Rectangle) -> Primitive {
        Primitive {
            target: state.target,
            distance: state.distance,
            yaw: state.yaw,
            pitch: state.pitch,
            camera_mode: self.camera_mode,
            show_grid: self.show_grid,
            grid_plane: self.grid_plane,
            grid_extent: self.grid_extent,
            grid_step: self.grid_step,
            entities: Arc::new(state.entities.clone()),
            entities_version: state.entities_version,
            preview_line: state.preview_line,
            preview_version: state.preview_version,
            selected: state.selected,
            axes_enabled: self.axes_enabled,
            axes_size: self.axes_size,
            axes_margin: self.axes_margin,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.position_in(bounds).is_some() && state.dragging != Dragging::None {
            mouse::Interaction::Grabbing
        } else if cursor.position_in(bounds).is_some() {
            mouse::Interaction::Grab
        } else {
            mouse::Interaction::default()
        }
    }
}

pub fn widget<'a, Message>(
    show_grid: bool,
    grid_plane: GridPlane,
    grid_extent: f32,
    grid_step: f32,
    tool: SceneTool,
    camera_mode: CameraMode,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
) -> Element<'a, Message>
where
    Message: 'a,
{
    iced::widget::shader::Shader::new(Scene {
        show_grid,
        grid_plane,
        grid_extent,
        grid_step,
        tool,
        camera_mode,
        axes_enabled,
        axes_size,
        axes_margin,
    })
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
}

