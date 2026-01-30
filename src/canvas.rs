use crate::drawing::{CanvasEvent, Draft, Shape, Tool};

use iced::widget::shader::{self, Viewport};
use iced::{mouse, Color as IcedColor, Element, Event, Length, Point, Rectangle};
use iced::wgpu;

use vello::kurbo::{Affine, BezPath, Circle, Rect as KRect, Stroke};
use vello::peniko::{Color as VelloColor, Fill};
use vello::{AaConfig, RenderParams, Renderer, RendererOptions};

use std::sync::{Arc, Mutex};

pub fn canvas<'a, Message>(
    tool: Tool,
    shapes: &'a [Shape],
    draft: &'a Draft,
    stroke_width: f32,
    stroke_color: IcedColor,
    fill_color: Option<IcedColor>,
    show_grid: bool,
    zoom: f32,
    view_offset: Point,
    cursor_pos: Option<Point>,
    on_event: fn(CanvasEvent) -> Message,
) -> Element<'a, Message>
where
    Message: 'a,
{
    iced::widget::shader::Shader::new(Board {
        tool,
        shapes,
        draft,
        stroke_width,
        stroke_color,
        fill_color,
        show_grid,
        zoom,
        view_offset,
        cursor_pos,
        on_event,
    })
    .width(Length::Fill)
    .height(Length::Fill)
    .into()
}

struct Board<'a, Message> {
    tool: Tool,
    shapes: &'a [Shape],
    draft: &'a Draft,
    stroke_width: f32,
    stroke_color: IcedColor,
    fill_color: Option<IcedColor>,
    show_grid: bool,
    zoom: f32,
    view_offset: Point,
    cursor_pos: Option<Point>,
    on_event: fn(CanvasEvent) -> Message,
}

impl<Message> shader::Program<Message> for Board<'_, Message> {
    type State = ();
    type Primitive = Primitive;

    fn update(
        &self,
        _state: &mut Self::State,
        event: &Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        use iced::mouse::{Button, Event as MouseEvent};

        let Some(pos) = cursor.position_in(bounds) else {
            return None;
        };

        let world_pos = Point {
            x: (pos.x - self.view_offset.x) / self.zoom,
            y: (pos.y - self.view_offset.y) / self.zoom,
        };

        let message = match event {
            Event::Mouse(MouseEvent::ButtonPressed(Button::Left)) => {
                (self.on_event)(CanvasEvent::PressedLeft(world_pos))
            }
            Event::Mouse(MouseEvent::ButtonReleased(Button::Left)) => {
                (self.on_event)(CanvasEvent::ReleasedLeft(world_pos))
            }
            Event::Mouse(MouseEvent::ButtonPressed(Button::Right)) => {
                (self.on_event)(CanvasEvent::PressedRight(world_pos))
            }
            Event::Mouse(MouseEvent::CursorMoved { .. }) => {
                (self.on_event)(CanvasEvent::Moved(world_pos))
            }
            Event::Mouse(MouseEvent::WheelScrolled { delta }) => {
                let amount = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => *y,
                };
                (self.on_event)(CanvasEvent::ZoomAt { delta: amount, cursor: pos })
            }
            _ => return None,
        };

        Some(shader::Action::publish(message).and_capture())
    }

    fn draw(&self, _state: &Self::State, _cursor: mouse::Cursor, _bounds: Rectangle) -> Primitive {
        Primitive {
            tool: self.tool,
            shapes: Arc::from(self.shapes.to_vec()),
            draft: self.draft.clone(),
            stroke_width: self.stroke_width,
            stroke_color: self.stroke_color,
            fill_color: self.fill_color,
            show_grid: self.show_grid,
            zoom: self.zoom,
            view_offset: self.view_offset,
            cursor_pos: self.cursor_pos,
        }
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            if self.tool == Tool::Select {
                mouse::Interaction::Pointer
            } else {
                mouse::Interaction::Crosshair
            }
        } else {
            mouse::Interaction::default()
        }
    }
}

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
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
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
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
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

#[derive(Debug, Clone)]
pub struct Primitive {
    tool: Tool,
    shapes: Arc<[Shape]>,
    draft: Draft,
    stroke_width: f32,
    stroke_color: IcedColor,
    fill_color: Option<IcedColor>,
    show_grid: bool,
    zoom: f32,
    view_offset: Point,
    cursor_pos: Option<Point>,
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

        let transform = Affine::scale(scale as f64)
            * Affine::translate((self.view_offset.x as f64, self.view_offset.y as f64))
            * Affine::scale(self.zoom as f64);

        if self.show_grid {
            draw_grid_vello(
                &mut scene,
                transform,
                *bounds,
                self.zoom,
                self.view_offset,
            );
        }

        for shape in self.shapes.iter() {
            draw_shape_vello(&mut scene, transform, shape);
        }

        let preview_stroke = IcedColor::from_rgb8(0xE6, 0xB8, 0x2E);
        if let Some(preview) = preview_shape(
            self.tool,
            &self.draft,
            self.stroke_width,
            self.stroke_color,
            self.fill_color,
            preview_stroke,
        ) {
            draw_shape_vello(&mut scene, transform, &preview);
        }

        if self.tool == Tool::Spline && !self.draft.spline_points.is_empty() {
            for p in &self.draft.spline_points {
                let c = Circle::new((p.x as f64, p.y as f64), 3.5);
                scene.fill(
                    Fill::NonZero,
                    transform,
                    VelloColor::from_rgb8(0x2E, 0xB8, 0xE6),
                    None,
                    &c,
                );
            }
        }

        if let Some(cursor) = self.cursor_pos {
            draw_crosshair_vello(
                &mut scene,
                transform,
                *bounds,
                self.zoom,
                self.view_offset,
                cursor,
            );
        }

        if let Ok(mut renderer) = pipeline.vello.lock() {
            let _ = renderer.render_to_texture(
                device,
                queue,
                &scene,
                target,
                &RenderParams {
                    base_color: VelloColor::from_rgb8(0x12, 0x12, 0x14),
                    width,
                    height,
                    antialiasing_method: AaConfig::Msaa16,
                },
            );
        }
    }

    fn draw(&self, pipeline: &Self::Pipeline, render_pass: &mut wgpu::RenderPass<'_>) -> bool {
        let Some(bind_group) = pipeline.blit_bind_group.as_ref() else {
            return true;
        };

        render_pass.set_pipeline(&pipeline.blit_pipeline);
        render_pass.set_bind_group(0, bind_group, &[]);
        render_pass.draw(0..3, 0..1);
        true
    }
}

fn iced_to_vello(color: IcedColor) -> VelloColor {
    fn f32_to_u8(v: f32) -> u8 {
        (v.clamp(0.0, 1.0) * 255.0).round() as u8
    }

    VelloColor::from_rgba8(
        f32_to_u8(color.r),
        f32_to_u8(color.g),
        f32_to_u8(color.b),
        f32_to_u8(color.a),
    )
}

fn draw_grid_vello(
    scene: &mut vello::Scene,
    transform: Affine,
    bounds: Rectangle,
    zoom: f32,
    view_offset: Point,
) {
    let spacing: f32 = 24.0;
    let minor = VelloColor::from_rgba8(0xFF, 0xFF, 0xFF, 0x0F);
    let major = VelloColor::from_rgba8(0xFF, 0xFF, 0xFF, 0x1F);

    let world_min = Point {
        x: (-view_offset.x) / zoom,
        y: (-view_offset.y) / zoom,
    };
    let world_max = Point {
        x: (bounds.width - view_offset.x) / zoom,
        y: (bounds.height - view_offset.y) / zoom,
    };

    let x0 = world_min.x.min(world_max.x);
    let x1 = world_min.x.max(world_max.x);
    let y0 = world_min.y.min(world_max.y);
    let y1 = world_min.y.max(world_max.y);

    let mut x = (x0 / spacing).floor() * spacing;
    while x <= x1 {
        let index = (x / spacing).round() as i64;
        let color = if index.rem_euclid(5) == 0 { major } else { minor };
        let mut path = BezPath::new();
        path.move_to((x as f64, y0 as f64));
        path.line_to((x as f64, y1 as f64));
        scene.stroke(&Stroke::new(1.0), transform, color, None, &path);
        x += spacing;
    }

    let mut y = (y0 / spacing).floor() * spacing;
    while y <= y1 {
        let index = (y / spacing).round() as i64;
        let color = if index.rem_euclid(5) == 0 { major } else { minor };
        let mut path = BezPath::new();
        path.move_to((x0 as f64, y as f64));
        path.line_to((x1 as f64, y as f64));
        scene.stroke(&Stroke::new(1.0), transform, color, None, &path);
        y += spacing;
    }
}

fn draw_crosshair_vello(
    scene: &mut vello::Scene,
    transform: Affine,
    bounds: Rectangle,
    zoom: f32,
    view_offset: Point,
    cursor: Point,
) {
    let world_min = Point {
        x: (-view_offset.x) / zoom,
        y: (-view_offset.y) / zoom,
    };
    let world_max = Point {
        x: (bounds.width - view_offset.x) / zoom,
        y: (bounds.height - view_offset.y) / zoom,
    };

    let color = VelloColor::from_rgba8(0xFF, 0xFF, 0xFF, 0x59);

    let mut h = BezPath::new();
    h.move_to((world_min.x as f64, cursor.y as f64));
    h.line_to((world_max.x as f64, cursor.y as f64));
    scene.stroke(&Stroke::new(1.0), transform, color, None, &h);

    let mut v = BezPath::new();
    v.move_to((cursor.x as f64, world_min.y as f64));
    v.line_to((cursor.x as f64, world_max.y as f64));
    scene.stroke(&Stroke::new(1.0), transform, color, None, &v);
}

fn draw_shape_vello(scene: &mut vello::Scene, transform: Affine, shape: &Shape) {
    match shape {
        Shape::Line {
            from,
            to,
            stroke_color,
            stroke_width,
        } => {
            let mut path = BezPath::new();
            path.move_to((from.x as f64, from.y as f64));
            path.line_to((to.x as f64, to.y as f64));

            scene.stroke(
                &Stroke::new(*stroke_width as f64),
                transform,
                iced_to_vello(*stroke_color),
                None,
                &path,
            );
        }
        Shape::Rect {
            from,
            to,
            stroke_color,
            fill_color,
            stroke_width,
        } => {
            let x0 = from.x.min(to.x) as f64;
            let y0 = from.y.min(to.y) as f64;
            let x1 = from.x.max(to.x) as f64;
            let y1 = from.y.max(to.y) as f64;
            let rect = KRect::new(x0, y0, x1, y1);

            if let Some(fill) = *fill_color {
                scene.fill(Fill::NonZero, transform, iced_to_vello(fill), None, &rect);
            }

            scene.stroke(
                &Stroke::new(*stroke_width as f64),
                transform,
                iced_to_vello(*stroke_color),
                None,
                &rect,
            );
        }
        Shape::Circle {
            center,
            radius,
            stroke_color,
            fill_color,
            stroke_width,
        } => {
            let circle = Circle::new((center.x as f64, center.y as f64), *radius as f64);

            if let Some(fill) = *fill_color {
                scene.fill(Fill::NonZero, transform, iced_to_vello(fill), None, &circle);
            }

            scene.stroke(
                &Stroke::new(*stroke_width as f64),
                transform,
                iced_to_vello(*stroke_color),
                None,
                &circle,
            );
        }
        Shape::Spline {
            points,
            stroke_color,
            stroke_width,
        } => {
            let samples = catmull_rom_samples(points, 24);
            if samples.len() < 2 {
                return;
            }

            let mut path = BezPath::new();
            path.move_to((samples[0].x as f64, samples[0].y as f64));
            for p in samples.iter().skip(1) {
                path.line_to((p.x as f64, p.y as f64));
            }

            scene.stroke(
                &Stroke::new(*stroke_width as f64),
                transform,
                iced_to_vello(*stroke_color),
                None,
                &path,
            );
        }
    }
}

fn preview_shape(
    tool: Tool,
    draft: &Draft,
    stroke_width: f32,
    _stroke_color: IcedColor,
    fill_color: Option<IcedColor>,
    preview_stroke_color: IcedColor,
) -> Option<Shape> {
    match tool {
        Tool::Select => None,
        Tool::Line => {
            if !draft.drawing {
                return None;
            }
            Some(Shape::Line {
                from: draft.start?,
                to: draft.current?,
                stroke_color: preview_stroke_color,
                stroke_width,
            })
        }
        Tool::Rect => {
            if !draft.drawing {
                return None;
            }
            Some(Shape::Rect {
                from: draft.start?,
                to: draft.current?,
                stroke_color: preview_stroke_color,
                fill_color,
                stroke_width,
            })
        }
        Tool::Circle => {
            if !draft.drawing {
                return None;
            }

            let start = draft.start?;
            let end = draft.current?;
            let radius = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
            Some(Shape::Circle {
                center: start,
                radius,
                stroke_color: preview_stroke_color,
                fill_color,
                stroke_width,
            })
        }
        Tool::Spline => {
            if draft.spline_points.len() < 2 {
                return None;
            }

            Some(Shape::Spline {
                points: draft.spline_points.clone(),
                stroke_color: preview_stroke_color,
                stroke_width,
            })
        }
    }
}

fn catmull_rom_samples(points: &[Point], subdiv: usize) -> Vec<Point> {
    if points.len() < 2 {
        return Vec::new();
    }

    let mut out = Vec::new();

    let get = |i: isize| -> Point {
        if i < 0 {
            points[0]
        } else if (i as usize) >= points.len() {
            *points.last().unwrap()
        } else {
            points[i as usize]
        }
    };

    for i in 0..(points.len() - 1) {
        let i = i as isize;
        let p0 = get(i - 1);
        let p1 = get(i);
        let p2 = get(i + 1);
        let p3 = get(i + 2);

        for s in 0..subdiv {
            let t = (s as f32) / (subdiv as f32);
            out.push(catmull_rom(p0, p1, p2, p3, t));
        }
    }

    out.push(*points.last().unwrap());
    out
}

fn catmull_rom(p0: Point, p1: Point, p2: Point, p3: Point, t: f32) -> Point {
    let t2 = t * t;
    let t3 = t2 * t;

    let x = 0.5
        * ((2.0 * p1.x)
            + (-p0.x + p2.x) * t
            + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
            + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

    let y = 0.5
        * ((2.0 * p1.y)
            + (-p0.y + p2.y) * t
            + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
            + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

    Point { x, y }
}