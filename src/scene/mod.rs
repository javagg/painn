use iced::widget::shader::{self, Viewport};
use iced::{keyboard, mouse, Element, Event, Length, Point, Rectangle};
use iced::wgpu;
use wgpu::util::DeviceExt;
use glam::{Mat4, Vec3, Vec4};

use crate::cad;
use crate::camera::{camera_from_params, ray_from_cursor, Camera};
use truck_polymesh::PolygonMesh;
use std::sync::Arc;

mod axes;
mod grid;
mod scale;
use axes::{build_axes_vertices, create_axes_pipeline, create_empty_axes_buffer};
use grid::{build_grid_vertices, create_grid_pipeline, GridVertex};
use scale::{build_scale_vertices, create_scale_pipeline};

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

pub use crate::camera::CameraMode;
pub use grid::GridPlane;

fn intersect_plane(plane: GridPlane, origin: Vec3, dir: Vec3) -> Option<Vec3> {
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

fn entity_radius(entity: &SceneEntity) -> f32 {
    match entity.kind {
        SolidKind::Box => entity.size * 0.7,
        SolidKind::Sphere => entity.size * 0.5,
        SolidKind::Cylinder => entity.size * 0.6,
        SolidKind::Cone => entity.size * 0.6,
        SolidKind::Torus => entity.size * 0.8,
    }
}

fn project_point(mvp: Mat4, bounds: Rectangle, p: Vec3) -> Option<Point> {
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
    Some(Point { x, y })
}

fn nice_scale_length(px_per_unit: f32, target_px: f32) -> (f32, f32) {
    if px_per_unit <= 1.0e-5 || target_px <= 1.0e-3 {
        return (1.0, target_px.max(1.0));
    }

    let raw_world = target_px / px_per_unit;
    let power = 10_f32.powf(raw_world.log10().floor());
    let mut lead = raw_world / power;
    lead = if lead < 1.5 {
        1.0
    } else if lead < 3.5 {
        2.0
    } else if lead < 7.5 {
        5.0
    } else {
        10.0
    };

    let world_len = lead * power;
    let bar_px = world_len * px_per_unit;
    (world_len, bar_px)
}

fn pick_entity(
    cursor: Point,
    bounds: Rectangle,
    camera: &Camera,
    entities: &[SceneEntity],
) -> Option<u64> {
    let view = Mat4::look_at_rh(camera.eye, camera.eye + camera.forward, Vec3::Y);
    let proj = match camera.mode {
        CameraMode::Perspective => Mat4::perspective_rh(camera.fovy, camera.aspect, 0.02, 500.0),
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, 0.02, 500.0)
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

fn build_scene_mesh_with_external(
    entities: &[SceneEntity],
    external: Option<&PolygonMesh>,
) -> Option<PolygonMesh> {
    let base = build_scene_mesh(entities);

    match (base, external) {
        (Some(mut mesh), Some(external_mesh)) => {
            mesh.merge(external_mesh.clone());
            Some(mesh)
        }
        (Some(mesh), None) => Some(mesh),
        (None, Some(external_mesh)) => Some(external_mesh.clone()),
        (None, None) => None,
    }
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

fn mesh_to_edge_vertices(mesh: &PolygonMesh) -> Vec<GridVertex> {
    use std::collections::HashSet;

    let positions = mesh.positions();
    let mut edges: HashSet<(usize, usize)> = HashSet::new();

    let mut add_edge = |a: usize, b: usize| {
        let (min_i, max_i) = if a < b { (a, b) } else { (b, a) };
        edges.insert((min_i, max_i));
    };

    for tri in mesh.tri_faces() {
        add_edge(tri[0].pos, tri[1].pos);
        add_edge(tri[1].pos, tri[2].pos);
        add_edge(tri[2].pos, tri[0].pos);
    }

    for quad in mesh.quad_faces() {
        add_edge(quad[0].pos, quad[1].pos);
        add_edge(quad[1].pos, quad[2].pos);
        add_edge(quad[2].pos, quad[3].pos);
        add_edge(quad[3].pos, quad[0].pos);
    }

    for face in mesh.other_faces() {
        if face.len() < 2 {
            continue;
        }
        for i in 0..face.len() {
            let a = face[i].pos;
            let b = face[(i + 1) % face.len()].pos;
            add_edge(a, b);
        }
    }

    let mut vertices = Vec::with_capacity(edges.len() * 2);
    for (a, b) in edges {
        let pa = positions[a];
        let pb = positions[b];
        vertices.push(GridVertex {
            position: [pa.x as f32, pa.y as f32, pa.z as f32],
        });
        vertices.push(GridVertex {
            position: [pb.x as f32, pb.y as f32, pb.z as f32],
        });
    }
    vertices
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    model_view: [[f32; 4]; 4],
    mvp: [[f32; 4]; 4],
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

fn preset_angles(preset: CameraPreset) -> (f32, f32) {
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
    id: u64,
    kind: SolidKind,
    position: Vec3,
    size: f32,
}

#[derive(Debug, Clone)]
pub struct SceneEntityInfo {
    pub id: u64,
    pub kind: SolidKind,
    #[allow(dead_code)]
    pub position: [f32; 3],
    pub size: f32,
}

impl From<&SceneEntity> for SceneEntityInfo {
    fn from(e: &SceneEntity) -> Self {
        Self {
            id: e.id,
            kind: e.kind,
            position: e.position.to_array(),
            size: e.size,
        }
    }
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
    edge_pipeline: wgpu::RenderPipeline,
    edge_vertices: wgpu::Buffer,
    edge_vertex_count: u32,
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
    mesh_version: (u64, u64),
    uniforms: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    depth: wgpu::TextureView,
    depth_size: (u32, u32),
    last_bounds: (f32, f32, f32, f32),
    // Axes overlay
    axes_pipeline: wgpu::RenderPipeline,
    axes_vertices: wgpu::Buffer,
    axes_vertex_count: u32,
    // Scale overlay
    scale_pipeline: wgpu::RenderPipeline,
    scale_vertices: wgpu::Buffer,
    scale_vertex_count: u32,
    scale_viewport: (f32, f32),
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

        let edge_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_edge_shader"),
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
    return vec4<f32>(0.08, 0.08, 0.08, 0.9);
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
            model_view: Mat4::IDENTITY.to_cols_array_2d(),
            mvp: Mat4::IDENTITY.to_cols_array_2d(),
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

        let grid_pipeline = create_grid_pipeline(device, format, &pipeline_layout);

        let edge_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_edge_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &edge_shader,
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
                module: &edge_shader,
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

        let axes_pipeline = create_axes_pipeline(device, format);
        let scale_pipeline = create_scale_pipeline(device, format);
        let scale_vertices = build_scale_vertices();
        let scale_vertices_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_scale_vertices"),
            contents: bytemuck::cast_slice(&scale_vertices),
            usage: wgpu::BufferUsages::VERTEX,
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
            edge_pipeline,
            edge_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_edge_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            edge_vertex_count: 0,
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
            mesh_version: (0, 0),
            uniforms,
            bind_group,
            depth,
            depth_size: (1, 1),
            last_bounds: (0.0, 0.0, 1.0, 1.0),
            axes_pipeline,
            axes_vertices: create_empty_axes_buffer(device),
            axes_vertex_count: 0,
            scale_pipeline,
            scale_vertices: scale_vertices_buffer,
            scale_vertex_count: scale_vertices.len() as u32,
            scale_viewport: (1.0, 1.0),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Primitive {
    target: Vec3,
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
    external_mesh: Option<Arc<PolygonMesh>>,
    external_version: u64,
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
    selected: Option<u64>,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
    scale_enabled: bool,
    scale_target_px: f32,
    scale_height: f32,
    scale_margin: f32,
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

        let view = Mat4::look_at_rh(camera.eye, camera.eye + camera.forward, Vec3::Y);

        let proj = match camera.mode {
            CameraMode::Perspective => Mat4::perspective_rh(camera.fovy, camera.aspect, 0.02, 500.0),
            CameraMode::Orthographic => {
                let half_h = camera.ortho_half_h;
                let half_w = half_h * camera.aspect;
                Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, 0.02, 500.0)
            }
        };

        let model = Mat4::IDENTITY;
        let model_view = view * model;
        let mvp = proj * model_view;

        let uniforms = Uniforms {
            model_view: model_view.to_cols_array_2d(),
            mvp: mvp.to_cols_array_2d(),
        };
        queue.write_buffer(&pipeline.uniforms, 0, bytemuck::bytes_of(&uniforms));

        let axes_vertices = build_axes_vertices(&camera);

        pipeline.axes_vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_axes_vertices"),
            contents: bytemuck::cast_slice(&axes_vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        pipeline.axes_vertex_count = axes_vertices.len() as u32;

        let target_px = self.scale_target_px.clamp(40.0, 240.0);
        let px_per_unit = match (
            project_point(mvp, *bounds, self.target),
            project_point(mvp, *bounds, self.target + camera.right),
        ) {
            (Some(a), Some(b)) => (b.x - a.x).hypot(b.y - a.y),
            _ => 0.0,
        };
        let (_, bar_px) = nice_scale_length(px_per_unit, target_px);
        let bar_px = bar_px.clamp(30.0, 260.0);
        let bar_rel = 0.8_f32;
        let vw = (bar_px / bar_rel).max(30.0);
        let vh = self.scale_height.clamp(18.0, 80.0);
        pipeline.scale_viewport = (vw, vh);

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

        let mesh_version = (self.entities_version, self.external_version);
        if pipeline.mesh_version != mesh_version {
            pipeline.mesh_version = mesh_version;

            if let Some(mesh) = build_scene_mesh_with_external(
                self.entities.as_slice(),
                self.external_mesh.as_deref(),
            ) {
                let (vertices, indices) = mesh_to_vertex_index(&mesh);
                let edge_vertices = mesh_to_edge_vertices(&mesh);

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

                pipeline.edge_vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("scene_edge_vertices"),
                    contents: bytemuck::cast_slice(&edge_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });

                pipeline.edge_vertex_count = edge_vertices.len() as u32;

                pipeline.index_count = indices.len() as u32;
            } else {
                pipeline.index_count = 0;
                pipeline.edge_vertex_count = 0;
            }
        }

        if pipeline.preview_version != self.preview_version {
            pipeline.preview_version = self.preview_version;
            if let Some((start, end)) = self.preview_line {
                let key = PreviewKey { start, end };
                if pipeline.preview_key != Some(key) {
                    pipeline.preview_key = Some(key);
                    let vertices = [
                        GridVertex { position: start.to_array() },
                        GridVertex { position: end.to_array() },
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

        if pipeline.edge_vertex_count > 0 {
            render_pass.set_pipeline(&pipeline.edge_pipeline);
            render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
            render_pass.set_vertex_buffer(0, pipeline.edge_vertices.slice(..));
            render_pass.draw(0..pipeline.edge_vertex_count, 0..1);
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

        // Draw scale bar overlay at bottom-left corner
        if self.scale_enabled && pipeline.scale_vertex_count > 0 {
            let margin = self.scale_margin.max(0.0);
            let (bx, by, bw, bh) = pipeline.last_bounds;
            let (vw_raw, vh_raw) = pipeline.scale_viewport;
            let vw = vw_raw.min(bw);
            let vh = vh_raw.min(bh);
            let vx = (bx + margin).min(bx + bw - vw).max(bx);
            let vy = (by + bh - vh - margin).max(by);

            render_pass.set_viewport(vx, vy, vw, vh, 0.0, 1.0);
            render_pass.set_pipeline(&pipeline.scale_pipeline);
            render_pass.set_vertex_buffer(0, pipeline.scale_vertices.slice(..));
            render_pass.draw(0..pipeline.scale_vertex_count, 0..1);

            render_pass.set_viewport(bx, by, bw, bh, 0.0, 1.0);
        }
    }
}

#[derive(Clone)]
pub struct Scene<Message> {
    show_grid: bool,
    grid_plane: GridPlane,
    grid_extent: f32,
    grid_step: f32,
    tool: SceneTool,
    camera_mode: CameraMode,
    camera_preset: Option<CameraPreset>,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
    scale_enabled: bool,
    scale_target_px: f32,
    scale_height: f32,
    scale_margin: f32,
    zoom_request_factor: f32,
    zoom_request_version: u64,
    external_mesh: Option<Arc<PolygonMesh>>,
    external_version: u64,
    on_entities_snapshot: Option<std::rc::Rc<dyn Fn(Vec<SceneEntityInfo>) -> Message + 'static>>, 
    request_select_id: Option<u64>,
    request_focus_id: Option<u64>,
}

#[derive(Debug, Clone)]
pub struct SceneState {
    target: Vec3,
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
    zoom_request_version: u64,
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
            target: Vec3::ZERO,
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
            drag_offset: Vec3::ZERO,
            modifiers: keyboard::Modifiers::default(),
            sphere_center: None,
            preview_line: None,
            preview_version: 0,
            zoom_request_version: 0,
        }
    }
}

impl<Message> shader::Program<Message> for Scene<Message> {
    type State = SceneState;
    type Primitive = Primitive;

    fn update(
        &self,
        state: &mut Self::State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        if self.zoom_request_version != state.zoom_request_version {
            state.zoom_request_version = self.zoom_request_version;
            if (self.zoom_request_factor - 1.0).abs() > f32::EPSILON {
                state.distance = (state.distance / self.zoom_request_factor).clamp(0.1, 200.0);
                return Some(shader::Action::request_redraw());
            }
        }

        // Apply sidebar-driven requests (select/focus)
        if let Some(id) = self.request_select_id {
            if state.entities.iter().any(|e| e.id == id) {
                state.selected = Some(id);
            }
        }
        if let Some(id) = self.request_focus_id {
            if let Some(ent) = state.entities.iter().find(|e| e.id == id) {
                state.target = ent.position;
            }
        }
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
                        if let Some(cb) = &self.on_entities_snapshot {
                            let list: Vec<SceneEntityInfo> = state.entities.iter().map(SceneEntityInfo::from).collect();
                            return Some(shader::Action::publish(cb(list)).and_capture());
                        }
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

        // Apply camera preset if present
        let (yaw, pitch) = if let Some(p) = self.camera_preset {
            preset_angles(p)
        } else {
            (state.yaw, state.pitch)
        };
        let camera = camera_from_params(state.target, state.distance, yaw, pitch, bounds, self.camera_mode);

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
                            if let Some(cb) = &self.on_entities_snapshot {
                                let list: Vec<SceneEntityInfo> = state.entities.iter().map(SceneEntityInfo::from).collect();
                                return Some(shader::Action::publish(cb(list)).and_capture());
                            }
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
                                state.drag_offset = entity.position - hit_pos;
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
                                let radius = (hit_pos - center).length().max(0.05);
                                let id = state.next_id;
                                state.next_id += 1;
                                state.entities.push(SceneEntity {
                                    id,
                                    kind: SolidKind::Sphere,
                                    position: center,
                                    size: radius * 2.0,
                                });
                                state.entities_version = state.entities_version.wrapping_add(1);
                                if let Some(cb) = &self.on_entities_snapshot {
                                    let list: Vec<SceneEntityInfo> = state.entities.iter().map(SceneEntityInfo::from).collect();
                                    return Some(shader::Action::publish(cb(list)).and_capture());
                                }
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
                                        entity.position = hit_pos + state.drag_offset;
                                        state.entities_version = state.entities_version.wrapping_add(1);
                                        if let Some(cb) = &self.on_entities_snapshot {
                                            let list: Vec<SceneEntityInfo> = state.entities.iter().map(SceneEntityInfo::from).collect();
                                            return Some(shader::Action::publish(cb(list)).and_capture());
                                        }
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
                                        let diff = hit_pos - start;
                                        let dist = diff.length().max(0.1);
                                        entity.size = dist;
                                        state.entities_version = state.entities_version.wrapping_add(1);
                                        if let Some(cb) = &self.on_entities_snapshot {
                                            let list: Vec<SceneEntityInfo> = state.entities.iter().map(SceneEntityInfo::from).collect();
                                            return Some(shader::Action::publish(cb(list)).and_capture());
                                        }
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

                                let pan = camera.right * (dx_ndc * half_w)
                                    + camera.up * (dy_ndc * half_h);
                                state.target += pan;
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
            yaw: if let Some(p) = self.camera_preset { preset_angles(p).0 } else { state.yaw },
            pitch: if let Some(p) = self.camera_preset { preset_angles(p).1 } else { state.pitch },
            camera_mode: self.camera_mode,
            show_grid: self.show_grid,
            grid_plane: self.grid_plane,
            grid_extent: self.grid_extent,
            grid_step: self.grid_step,
            entities: Arc::new(state.entities.clone()),
            entities_version: state.entities_version,
            external_mesh: self.external_mesh.clone(),
            external_version: self.external_version,
            preview_line: state.preview_line,
            preview_version: state.preview_version,
            selected: state.selected,
            axes_enabled: self.axes_enabled,
            axes_size: self.axes_size,
            axes_margin: self.axes_margin,
            scale_enabled: self.scale_enabled,
            scale_target_px: self.scale_target_px,
            scale_height: self.scale_height,
            scale_margin: self.scale_margin,
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
    camera_preset: Option<CameraPreset>,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
    scale_enabled: bool,
    scale_target_px: f32,
    scale_height: f32,
    scale_margin: f32,
    zoom_request_factor: f32,
    zoom_request_version: u64,
    external_mesh: Option<Arc<PolygonMesh>>,
    external_version: u64,
    request_select_id: Option<u64>,
    request_focus_id: Option<u64>,
    on_entities_snapshot: impl Fn(Vec<SceneEntityInfo>) -> Message + 'static,
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
        camera_preset,
        axes_enabled,
        axes_size,
        axes_margin,
        scale_enabled,
        scale_target_px,
        scale_height,
        scale_margin,
        zoom_request_factor,
        zoom_request_version,
        external_mesh,
        external_version,
        on_entities_snapshot: Some(std::rc::Rc::new(on_entities_snapshot)),
        request_select_id,
        request_focus_id,
    })
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
}
