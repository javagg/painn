use iced::widget::shader::{self, Viewport};
use iced::{mouse, Element, Event, Length, Point, Rectangle};
use iced::wgpu;

use crate::cad;

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

#[inline]
fn mat4_mul(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    // Column-major matrices (WGSL convention).
    // Element at row r, col c is m[c][r].
    let mut out = [[0.0_f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = (0..4).map(|k| a[k][r] * b[c][k]).sum();
        }
    }
    out
}

#[inline]
fn mat4_scale(sx: f32, sy: f32, sz: f32) -> [[f32; 4]; 4] {
    [
        [sx, 0.0, 0.0, 0.0],
        [0.0, sy, 0.0, 0.0],
        [0.0, 0.0, sz, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[inline]
fn mat4_rot_x(angle: f32) -> [[f32; 4]; 4] {
    let (s, c) = angle.sin_cos();
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, s, 0.0],
        [0.0, -s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[inline]
fn mat4_rot_y(angle: f32) -> [[f32; 4]; 4] {
    let (s, c) = angle.sin_cos();
    [
        [c, 0.0, -s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniforms {
    transform: [[f32; 4]; 4],
}

impl Uniforms {
    fn from_pan_zoom(pan: [f32; 2], zoom: f32) -> Self {
        let zoom = zoom.clamp(0.25, 8.0);

        // WGSL matrices are column-major. Each inner [f32; 4] is a column.
        let col0 = [zoom, 0.0, 0.0, 0.0];
        let col1 = [0.0, zoom, 0.0, 0.0];
        let col2 = [0.0, 0.0, 1.0, 0.0];
        let col3 = [pan[0], pan[1], 0.0, 1.0];

        Self {
            transform: [col0, col1, col2, col3],
        }
    }
}

#[derive(Debug)]
pub struct Pipeline {
    pipeline: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    index_count: u32,
    uniforms: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

impl shader::Pipeline for Pipeline {
    fn new(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
    ) -> Self {
        use wgpu::util::DeviceExt;

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_mesh_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    transform: mat4x4<f32>,
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
    out.position = uniforms.transform * vec4<f32>(in.position, 1.0);
    let nmat = mat3x3<f32>(
        uniforms.transform[0].xyz,
        uniforms.transform[1].xyz,
        uniforms.transform[2].xyz,
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

        let initial_uniforms = Uniforms::from_pan_zoom([0.0, 0.0], 1.0);
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

        let mesh = cad::load_obj();
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
            depth_stencil: None,
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

        Self {
            pipeline,
            vertices: vertex_buffer,
            indices: index_buffer,
            index_count: tri_indices.len() as u32,
            uniforms,
            bind_group,
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Primitive {
    pan: [f32; 2],
    zoom: f32,
    yaw: f32,
    pitch: f32,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        _device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &Rectangle,
        _viewport: &Viewport,
    ) {
        let pan_zoom = Uniforms::from_pan_zoom(self.pan, self.zoom).transform;

        let aspect = if bounds.height > 1.0 {
            (bounds.width / bounds.height).max(1.0e-6)
        } else {
            1.0
        };

        // Keep the model's aspect consistent with the view.
        let aspect_fix = mat4_scale(1.0 / aspect, 1.0, 1.0);
        let model = mat4_mul(mat4_rot_y(self.yaw), mat4_rot_x(self.pitch));

        let transform = mat4_mul(pan_zoom, mat4_mul(aspect_fix, model));
        let uniforms = Uniforms { transform };
        queue.write_buffer(&pipeline.uniforms, 0, bytemuck::bytes_of(&uniforms));
    }

    fn draw(
        &self,
        pipeline: &Self::Pipeline,
        render_pass: &mut wgpu::RenderPass<'_>,
    ) -> bool {
        render_pass.set_pipeline(&pipeline.pipeline);
        render_pass.set_bind_group(0, &pipeline.bind_group, &[]);
        render_pass.set_vertex_buffer(0, pipeline.vertices.slice(..));
        render_pass.set_index_buffer(pipeline.indices.slice(..), wgpu::IndexFormat::Uint32);
        render_pass.draw_indexed(0..pipeline.index_count, 0, 0..1);
        true
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Scene;

#[derive(Debug, Clone)]
pub struct SceneState {
    zoom: f32,
    pan: [f32; 2],
    yaw: f32,
    pitch: f32,
    dragging: Dragging,
    last_cursor: Option<Point>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dragging {
    None,
    Pan,
    Rotate,
}

impl Default for SceneState {
    fn default() -> Self {
        Self {
            zoom: 1.0,
            pan: [0.0, 0.0],
            yaw: 0.8,
            pitch: -0.6,
            dragging: Dragging::None,
            last_cursor: None,
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
        let Some(cursor_pos) = cursor.position_in(bounds) else {
            if matches!(
                event,
                Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right))
                    | Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left))
            ) {
                state.dragging = Dragging::None;
                state.last_cursor = None;
                return Some(shader::Action::request_redraw().and_capture());
            }

            return None;
        };

        match event {
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let scroll_y = match *delta {
                    mouse::ScrollDelta::Lines { y, .. } => y,
                    mouse::ScrollDelta::Pixels { y, .. } => y / 120.0,
                };

                if scroll_y.abs() > f32::EPSILON {
                    let factor = 1.1_f32.powf(scroll_y);
                    state.zoom = (state.zoom * factor).clamp(0.25, 8.0);
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                state.dragging = Dragging::Rotate;
                state.last_cursor = Some(cursor_pos);
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if state.dragging == Dragging::Rotate {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                state.dragging = Dragging::Pan;
                state.last_cursor = Some(cursor_pos);
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                if state.dragging == Dragging::Pan {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                match state.dragging {
                    Dragging::None => {}
                    Dragging::Pan => {
                        if let Some(last) = state.last_cursor {
                            let dx = last.x - cursor_pos.x;
                            let dy = last.y - cursor_pos.y;

                            if bounds.width > 1.0 && bounds.height > 1.0 {
                                let dx_ndc = (dx * 2.0) / bounds.width;
                                let dy_ndc = (-dy * 2.0) / bounds.height;

                                // Keep panning speed consistent while zoomed in.
                                let zoom = state.zoom.max(0.25);
                                state.pan[0] += dx_ndc / zoom;
                                state.pan[1] += dy_ndc / zoom;
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
                            let dx =  last.x - cursor_pos.x;
                            let dy = last.y - cursor_pos.y;

                            let rot_speed = 0.01;
                            state.yaw += dx * rot_speed;
                            state.pitch += dy * rot_speed;

                            let max_pitch = 1.55; // ~89deg to avoid flipping
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
            pan: state.pan,
            zoom: state.zoom,
            yaw: state.yaw,
            pitch: state.pitch,
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

pub fn widget<'a, Message>() -> Element<'a, Message>
where
    Message: 'a,
{
    iced::widget::shader::Shader::new(Scene)
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
}

