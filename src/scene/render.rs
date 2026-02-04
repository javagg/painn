use glam::{Mat4, Vec3};
use wgpu::util::DeviceExt;

use crate::scene::{
    AxesVertex, GridPlane, GridVertex, SceneRect, SceneView, Uniforms, Vertex, build_grid_vertices, build_scene_mesh_with_faces, camera_from_params, mesh_to_vertex_index
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct GridKey {
    enabled: bool,
    plane: GridPlane,
    extent: f32,
    step: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct PreviewKey {
    start: Vec3,
    end: Vec3,
}


#[derive(Debug)]
pub struct Pipeline {
    pub(crate) background_pipeline: wgpu::RenderPipeline,
    pub(crate) background_uniforms: wgpu::Buffer,
    pub(crate) background_bind_group: wgpu::BindGroup,
    pub(crate) background_color: [f32; 4],
    pub(crate) pipeline: wgpu::RenderPipeline,
    pub(crate) vertices: wgpu::Buffer,
    pub(crate) indices: wgpu::Buffer,
    pub(crate) index_count: u32,
    pub(crate) grid_pipeline: wgpu::RenderPipeline,
    pub(crate) grid_vertices: wgpu::Buffer,
    pub(crate) grid_vertex_count: u32,
    pub(crate) grid_key: Option<GridKey>,
    pub(crate) preview_pipeline: wgpu::RenderPipeline,
    pub(crate) preview_vertices: wgpu::Buffer,
    pub(crate) preview_vertex_count: u32,
    pub(crate) preview_key: Option<PreviewKey>,
    pub(crate) preview_version: u64,
    pub(crate) sketch_version: u64,
    pub(crate) face_highlight_version: u64,
    pub(crate) highlight_pipeline: wgpu::RenderPipeline,
    pub(crate) highlight_vertices: wgpu::Buffer,
    pub(crate) highlight_vertex_count: u32,
    pub(crate) selected_entities_version: u64,
    pub(crate) entities_version: u64,
    pub(crate) sketch_faces_version: u64,
    pub(crate) uniforms: wgpu::Buffer,
    pub(crate) bind_group: wgpu::BindGroup,
    pub(crate) depth: wgpu::TextureView,
    pub(crate) depth_size: (u32, u32),
    pub(crate) last_bounds: (f32, f32, f32, f32),
    // Axes overlay
    pub(crate) axes_pipeline: wgpu::RenderPipeline,
    pub(crate) axes_vertices: wgpu::Buffer,
    pub(crate) axes_vertex_count: u32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct BackgroundUniform {
    color: [f32; 4],
}

impl Pipeline {
    pub fn create(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
    ) -> Self {
        const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

        let background_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene_background_shader"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
struct BgUniform {
    color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> bg: BgUniform;

@vertex
fn vs_main(@builtin(vertex_index) index: u32) -> @builtin(position) vec4<f32> {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0)
    );
    let p = positions[index];
    return vec4<f32>(p, 0.0, 1.0);
}

@fragment
fn fs_main() -> @location(0) vec4<f32> {
    return bg.color;
}
"#
                    .into(),
            ),
        });

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
    return vec4<f32>(0.05, 1.0, 1.0, 1.0);
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

        let background_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("scene_background_bind_group_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
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

        let initial_background = BackgroundUniform {
            color: [0.0, 0.0, 0.0, 1.0],
        };
        let background_uniforms = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("scene_background_uniforms"),
            contents: bytemuck::bytes_of(&initial_background),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let background_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scene_background_bind_group"),
            layout: &background_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: background_uniforms.as_entire_binding(),
            }],
        });

        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scene_mesh_vertices"),
            size: 4,
            usage: wgpu::BufferUsages::VERTEX,
            mapped_at_creation: false,
        });

        let index_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scene_mesh_indices"),
            size: 4,
            usage: wgpu::BufferUsages::INDEX,
            mapped_at_creation: false,
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene_mesh_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let background_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("scene_background_pipeline_layout"),
                bind_group_layouts: &[&background_bind_group_layout],
                push_constant_ranges: &[],
            });

        let background_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("scene_background_pipeline"),
            layout: Some(&background_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &background_shader,
                entry_point: Some("vs_main"),
                buffers: &[],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
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
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &background_shader,
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
            background_pipeline,
            background_uniforms,
            background_bind_group,
            background_color: [0.0, 0.0, 0.0, 1.0],
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
            sketch_version: 0,
            face_highlight_version: 0,
            highlight_pipeline,
            highlight_vertices: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("scene_highlight_vertices"),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX,
                mapped_at_creation: false,
            }),
            highlight_vertex_count: 0,
            selected_entities_version: 0,
            entities_version: 0,
            sketch_faces_version: 0,
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

impl SceneView {
    pub fn do_prepare(
        &self,
        pipeline: &mut Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bounds: SceneRect,
        physical_size: (u32, u32),
        scale_factor: f32,
    ) {
        // Ensure the depth buffer matches the swapchain (frame) size.
        let target_w = physical_size.0.max(1);
        let target_h = physical_size.1.max(1);

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
        pipeline.last_bounds = (
            bounds.x * scale_factor,
            bounds.y * scale_factor,
            (bounds.width * scale_factor).max(1.0),
            (bounds.height * scale_factor).max(1.0),
        );

        // CAD-like orbit camera around a target point.
        let camera = camera_from_params(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            bounds,
            self.camera_mode,
        );

        let view = Mat4::look_at_rh(camera.eye, camera.eye + camera.forward, Vec3::Y);

        let proj = match camera.mode {
            crate::scene::CameraMode::Perspective => {
                Mat4::perspective_rh(camera.fovy, camera.aspect, camera.near, camera.far)
            }
            crate::scene::CameraMode::Orthographic => {
                let half_h = camera.ortho_half_h;
                let half_w = half_h * camera.aspect;
                Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, camera.near, camera.far)
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

        // Build axes overlay vertices in clip space (-1..1) for a mini viewport.
        // Project world axes onto camera's screen axes using camera.right/up.
        let len = 0.9_f32;
        let mut axes_vertices: Vec<AxesVertex> = Vec::with_capacity(32);

        let mut add_axis_with_label = |v: Vec3, color: [f32; 3], ch: char| {
            let x = v.dot(camera.right);
            let y = v.dot(camera.up);
            let ex = x * len;
            let ey = y * len;

            // Axis line from origin to endpoint in clip space
            axes_vertices.push(AxesVertex { position: [0.0, 0.0, 0.0], color });
            axes_vertices.push(AxesVertex { position: [ex, ey, 0.0], color });

            // Label near endpoint, screen-aligned, small size in clip units
            let mut nx = x;
            let mut ny = y;
            let mag = (nx * nx + ny * ny).sqrt();
            if mag > 1.0e-4 {
                nx /= mag;
                ny /= mag;
            } else {
                nx = 1.0;
                ny = 0.0;
            }
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

        add_axis_with_label(Vec3::X, [0.95, 0.3, 0.3], 'X'); // X - red
        add_axis_with_label(Vec3::Y, [0.4, 0.9, 0.5], 'Y');   // Y - green
        add_axis_with_label(Vec3::Z, [0.35, 0.6, 0.95], 'Z'); // Z - blue

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

        if pipeline.entities_version != self.entities_version
            || pipeline.sketch_faces_version != self.sketch_faces_version
        {
            pipeline.entities_version = self.entities_version;
            pipeline.sketch_faces_version = self.sketch_faces_version;

            if let Some(mesh) = build_scene_mesh_with_faces(
                self.entities.as_slice(),
                self.sketch_faces.as_slice(),
            ) {
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

        if pipeline.preview_version != self.preview_version
            || pipeline.sketch_version != self.sketch_version
            || pipeline.face_highlight_version != self.face_highlight_version
        {
            pipeline.preview_version = self.preview_version;
            pipeline.sketch_version = self.sketch_version;
            pipeline.face_highlight_version = self.face_highlight_version;

            let preview_segments = self.preview_segments.as_ref();
            let sketch_segments = self.sketch_segments.as_ref();
            let highlight_segments = self.face_highlight_segments.as_ref();
            let total = preview_segments.len() + sketch_segments.len() + highlight_segments.len();

            if total == 0 {
                pipeline.preview_key = None;
                pipeline.preview_vertex_count = 0;
            } else {
                let mut vertices: Vec<GridVertex> = Vec::with_capacity(total * 2);
                for (a, b) in highlight_segments
                    .iter()
                    .chain(sketch_segments.iter())
                    .chain(preview_segments.iter())
                {
                    vertices.push(GridVertex {
                        position: a.to_array(),
                    });
                    vertices.push(GridVertex {
                        position: b.to_array(),
                    });
                }

                pipeline.preview_vertices =
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("scene_preview_vertices"),
                        contents: bytemuck::cast_slice(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    });
                pipeline.preview_vertex_count = vertices.len() as u32;
                pipeline.preview_key = None;
            }
        }

        if pipeline.selected_entities_version != self.selected_entities_version
            || pipeline.entities_version != self.entities_version
        {
            pipeline.selected_entities_version = self.selected_entities_version;

            let mut vertices: Vec<GridVertex> = Vec::new();
            let selected = self.selected_entities.as_ref();
            for id in selected {
                if let Some(e) = self.entities.iter().find(|ent| ent.id == *id) {
                    let size = e.size.x.max(e.size.y).max(e.size.z);
                    let s = (size * 0.6).max(0.05);
                    let p = e.position;
                    vertices.push(GridVertex {
                        position: [p[0] - s, p[1], p[2]],
                    });
                    vertices.push(GridVertex {
                        position: [p[0] + s, p[1], p[2]],
                    });
                    vertices.push(GridVertex {
                        position: [p[0], p[1] - s, p[2]],
                    });
                    vertices.push(GridVertex {
                        position: [p[0], p[1] + s, p[2]],
                    });
                    vertices.push(GridVertex {
                        position: [p[0], p[1], p[2] - s],
                    });
                    vertices.push(GridVertex {
                        position: [p[0], p[1], p[2] + s],
                    });
                }
            }

            if vertices.is_empty() {
                pipeline.highlight_vertex_count = 0;
            } else {
                pipeline.highlight_vertices =
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                        label: Some("scene_highlight_vertices"),
                        contents: bytemuck::cast_slice(&vertices),
                        usage: wgpu::BufferUsages::VERTEX,
                    });
                pipeline.highlight_vertex_count = vertices.len() as u32;
            }
        }

        if pipeline.background_color != self.background {
            pipeline.background_color = self.background;
            let uniforms = BackgroundUniform {
                color: self.background,
            };
            queue.write_buffer(
                &pipeline.background_uniforms,
                0,
                bytemuck::bytes_of(&uniforms),
            );
        }
    }

    pub fn do_render(
        &self,
        pipeline: &Pipeline,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        clip_bounds: &iced::Rectangle<u32>,
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

        render_pass.set_pipeline(&pipeline.background_pipeline);
        render_pass.set_bind_group(0, &pipeline.background_bind_group, &[]);
        render_pass.draw(0..3, 0..1);

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