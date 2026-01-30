use iced::widget::shader::{self, Viewport};
use iced::{mouse, Element, Event, Length, Point, Rectangle};
use iced::wgpu;

use vello::kurbo::{Affine, BezPath};
use vello::peniko::{Color, Fill};
use vello::{AaConfig, RenderParams, Renderer, RendererOptions};

use std::sync::Mutex;

pub struct Pipeline {
    vello: Mutex<Renderer>,

    blit_pipeline: wgpu::RenderPipeline,
    blit_bind_group_layout: wgpu::BindGroupLayout,
    blit_sampler: wgpu::Sampler,

    target_size: (u32, u32),
    target_texture: Option<wgpu::Texture>,
    target_view: Option<wgpu::TextureView>,
    blit_bind_group: Option<wgpu::BindGroup>,
}

impl shader::Pipeline for Pipeline {
    fn new(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
    ) -> Self {
        let vello = Mutex::new(
            Renderer::new(device, RendererOptions::default())
                .expect("Failed to create vello Renderer"),
        );

        let blit_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("vello_blit_sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let blit_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("vello_blit_bind_group_layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float {
                                filterable: true,
                            },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });

        let blit_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("vello_blit_pipeline_layout"),
                bind_group_layouts: &[&blit_bind_group_layout],
                push_constant_ranges: &[],
            });

        let blit_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("vello_blit_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) tex_coords: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vi: u32) -> VertexOutput {
    var out: VertexOutput;

    out.tex_coords = vec2<f32>(
        f32((vi << 1u) & 2u),
        f32(vi & 2u),
    );

    out.position = vec4<f32>(out.tex_coords * 2.0 - 1.0, 0.0, 1.0);

    // Invert y so the texture is not upside down
    out.tex_coords.y = 1.0 - out.tex_coords.y;
    return out;
}

@group(0) @binding(0)
var texture: texture_2d<f32>;
@group(0) @binding(1)
var texture_sampler: sampler;

@fragment
fn fs_main(vs: VertexOutput) -> @location(0) vec4<f32> {
    return textureSample(texture, texture_sampler, vs.tex_coords);
}
"#
                .into(),
            ),
        });

        let blit_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("vello_blit_pipeline"),
            layout: Some(&blit_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &blit_shader,
                entry_point: Some("vs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[],
            },
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                unclipped_depth: false,
                polygon_mode: wgpu::PolygonMode::Fill,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &blit_shader,
                entry_point: Some("fs_main"),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            multiview: None,
            cache: None,
        });

        Self {
            vello,
            blit_pipeline,
            blit_bind_group_layout,
            blit_sampler,
            target_size: (0, 0),
            target_texture: None,
            target_view: None,
            blit_bind_group: None,
        }
    }
}

impl Pipeline {
    fn ensure_target_texture(&mut self, device: &wgpu::Device, width: u32, height: u32) {
        let width = width.max(1);
        let height = height.max(1);

        if self.target_texture.is_some() && self.target_size == (width, height) {
            return;
        }

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("vello_intermediate_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::STORAGE_BINDING
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vello_blit_bind_group"),
            layout: &self.blit_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.blit_sampler),
                },
            ],
        });

        self.target_size = (width, height);
        self.target_texture = Some(texture);
        self.target_view = Some(view);
        self.blit_bind_group = Some(bind_group);
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
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: &Rectangle,
        viewport: &Viewport,
    ) {
        let scale = viewport.scale_factor();
        let width = (bounds.width * scale).round().max(1.0) as u32;
        let height = (bounds.height * scale).round().max(1.0) as u32;

        pipeline.ensure_target_texture(device, width, height);

        let Some(target) = pipeline.target_view.as_ref() else {
            return;
        };

        let mut scene = vello::Scene::new();

        // 以“逻辑像素”坐标构建一个简单的三角形；
        // 再通过 Affine::scale(scale_factor) 映射到物理像素纹理。
        let w = bounds.width.max(1.0) as f64;
        let h = bounds.height.max(1.0) as f64;
        let mut path = BezPath::new();
        path.move_to((w * 0.5, h * 0.15));
        path.line_to((w * 0.15, h * 0.85));
        path.line_to((w * 0.85, h * 0.85));
        path.close_path();

        let zoom = self.zoom.clamp(0.25, 8.0) as f64;
        let pan_x = self.pan[0] as f64;
        let pan_y = self.pan[1] as f64;

        // screen_phys = scale * (pan + zoom * world_logical)
        let transform = Affine::scale(scale as f64)
            * Affine::translate((pan_x, pan_y))
            * Affine::scale(zoom);

        scene.fill(
            Fill::NonZero,
            transform,
            Color::from_rgb8(0x52, 0xC1, 0xFF),
            None,
            &path,
        );

        if let Ok(mut renderer) = pipeline.vello.lock() {
            let _ = renderer.render_to_texture(
                device,
                queue,
                &scene,
                target,
                &RenderParams {
                    base_color: Color::from_rgb8(0x12, 0x12, 0x12),
                    width,
                    height,
                    antialiasing_method: AaConfig::Msaa16,
                },
            );
        }
   }

    fn draw(
        &self,
        pipeline: &Self::Pipeline,
        render_pass: &mut wgpu::RenderPass<'_>,
    ) -> bool {
        let Some(bind_group) = pipeline.blit_bind_group.as_ref() else {
            return true;
        };

        render_pass.set_pipeline(&pipeline.blit_pipeline);
        render_pass.set_bind_group(0, bind_group, &[]);
        render_pass.draw(0..3, 0..1);
        true
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VelloProgram;

#[derive(Debug, Clone)]
pub struct VelloState {
    zoom: f32,
    pan: [f32; 2],
    panning: bool,
    last_cursor: Option<Point>,
}

impl Default for VelloState {
    fn default() -> Self {
        Self {
            zoom: 1.0,
            pan: [0.0, 0.0],
            panning: false,
            last_cursor: None,
        }
    }
}

impl<Message> shader::Program<Message> for VelloProgram {
    type Primitive = Primitive;

    type State = VelloState;

    fn update(
        &self,
        state: &mut Self::State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        let cursor_pos = cursor.position_in(bounds);

        match event {
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let Some(cursor_pos) = cursor_pos else {
                    return None;
                };

                let scroll_y = match *delta {
                    mouse::ScrollDelta::Lines { y, .. } => y,
                    mouse::ScrollDelta::Pixels { y, .. } => y / 120.0,
                };

                if scroll_y.abs() <= f32::EPSILON {
                    return None;
                }

                let old_zoom = state.zoom.clamp(0.25, 8.0);
                let factor = 1.1_f32.powf(scroll_y);
                let new_zoom = (old_zoom * factor).clamp(0.25, 8.0);

                if (new_zoom - old_zoom).abs() <= f32::EPSILON {
                    return None;
                }

                // 以鼠标位置为缩放中心（逻辑坐标）
                let world_x = (cursor_pos.x - state.pan[0]) / old_zoom;
                let world_y = (cursor_pos.y - state.pan[1]) / old_zoom;
                state.zoom = new_zoom;
                state.pan[0] = cursor_pos.x - world_x * new_zoom;
                state.pan[1] = cursor_pos.y - world_y * new_zoom;

                Some(shader::Action::request_redraw().and_capture())
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                state.panning = true;
                state.last_cursor = cursor_pos;
                Some(shader::Action::request_redraw().and_capture())
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Right)) => {
                state.panning = false;
                state.last_cursor = None;
                Some(shader::Action::request_redraw().and_capture())
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                if !state.panning {
                    return None;
                }

                let Some(cursor_pos) = cursor_pos else {
                    return None;
                };

                if let Some(last) = state.last_cursor {
                    let dx = cursor_pos.x - last.x;
                    let dy = cursor_pos.y - last.y;
                    state.pan[0] += dx;
                    state.pan[1] += dy;
                }

                state.last_cursor = Some(cursor_pos);
                Some(shader::Action::request_redraw().and_capture())
            }
            _ => None,
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

    fn draw(
        &self,
        state: &Self::State,
        _cursor: mouse::Cursor,
        _bounds: Rectangle,
    ) -> Self::Primitive {
        Primitive {
            pan: state.pan,
            zoom: state.zoom,
        }
    }
}

pub fn widget<'a, Message>() -> Element<'a, Message>
where
    Message: 'a,
{
    iced::widget::shader::Shader::new(VelloProgram)
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
}