use iced::widget::shader::{self, Viewport};
use iced::{mouse, Element, Event, Length, Point, Rectangle};
use iced::wgpu;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 2],
    color: [f32; 3],
}

impl Vertex {
    const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x2, 1 => Float32x3];

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
    vertex_count: u32,
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
            label: Some("scene_triangle_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct Uniforms {
    transform: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec2<f32>,
    @location(1) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.position = uniforms.transform * vec4<f32>(in.position, 0.0, 1.0);
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
            label: Some("scene_triangle_uniforms"),
            contents: bytemuck::bytes_of(&initial_uniforms),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scene_triangle_bind_group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniforms.as_entire_binding(),
            }],
        });

        let vertices: [Vertex; 3] = [
            Vertex {
                position: [0.0, 0.6],
                color: [1.0, 0.2, 0.2],
            },
            Vertex {
                position: [-0.6, -0.6],
                color: [0.2, 1.0, 0.2],
            },
            Vertex {
                position: [0.6, -0.6],
                color: [0.2, 0.4, 1.0],
            },
        ];

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_triangle_vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene_triangle_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_triangle_pipeline"),
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
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
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
            vertex_count: vertices.len() as u32,
            uniforms,
            bind_group,
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Primitive {
    pan: [f32; 2],
    zoom: f32,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        _device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: &Rectangle,
        _viewport: &Viewport,
    ) {
        let uniforms = Uniforms::from_pan_zoom(self.pan, self.zoom);
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
        render_pass.draw(0..pipeline.vertex_count, 0..1);
        true
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Scene;

#[derive(Debug, Clone)]
pub struct SceneState {
    zoom: f32,
    pan: [f32; 2],
    panning: bool,
    last_cursor: Option<Point>,
}

impl Default for SceneState {
    fn default() -> Self {
        Self {
            zoom: 1.0,
            pan: [0.0, 0.0],
            panning: false,
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
            if matches!(event, Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)))
            {
                state.panning = false;
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
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                state.panning = true;
                state.last_cursor = Some(cursor_pos);
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                state.panning = false;
                state.last_cursor = None;
                return Some(shader::Action::request_redraw().and_capture());
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if state.panning {
                    if let Some(last) = state.last_cursor {
                        let dx = cursor_pos.x - last.x;
                        let dy = cursor_pos.y - last.y;

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
            }
            _ => {}
        }

        None
    }

    fn draw(&self, state: &Self::State, _cursor: mouse::Cursor, _bounds: Rectangle) -> Primitive {
        Primitive {
            pan: state.pan,
            zoom: state.zoom,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.position_in(bounds).is_some() && state.panning {
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

