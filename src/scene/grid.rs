use iced::wgpu;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct GridVertex {
    pub position: [f32; 3],
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

pub(crate) fn create_grid_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    pipeline_layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
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

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("scene_grid_pipeline"),
        layout: Some(pipeline_layout),
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
            format: wgpu::TextureFormat::Depth24Plus,
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
    })
}
