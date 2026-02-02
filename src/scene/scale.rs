use iced::wgpu;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct ScaleVertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

impl ScaleVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub(crate) fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<ScaleVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

pub(crate) fn create_scale_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    let scale_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("scene_scale_shader"),
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

    let scale_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("scene_scale_pipeline_layout"),
        bind_group_layouts: &[],
        push_constant_ranges: &[],
    });

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("scene_scale_pipeline"),
        layout: Some(&scale_pipeline_layout),
        vertex: wgpu::VertexState {
            module: &scale_shader,
            entry_point: Some("vs_main"),
            buffers: &[ScaleVertex::layout()],
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
            depth_compare: wgpu::CompareFunction::Always,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: &scale_shader,
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
    })
}

pub(crate) fn build_scale_vertices() -> Vec<ScaleVertex> {
    let color = [0.92, 0.92, 0.92];
    let bar = 0.8_f32;
    let y = -0.15_f32;
    let tick_big = 0.35_f32;
    let tick_mid = 0.25_f32;

    let mut vertices = Vec::with_capacity(12);

    // Baseline
    vertices.push(ScaleVertex {
        position: [-bar, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [bar, y, 0.0],
        color,
    });

    // End ticks
    vertices.push(ScaleVertex {
        position: [-bar, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [-bar, y + tick_big, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [bar, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [bar, y + tick_big, 0.0],
        color,
    });

    // Middle tick
    vertices.push(ScaleVertex {
        position: [0.0, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [0.0, y + tick_mid, 0.0],
        color,
    });

    // Quarter ticks
    let q = bar * 0.5;
    vertices.push(ScaleVertex {
        position: [-q, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [-q, y + tick_mid, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [q, y, 0.0],
        color,
    });
    vertices.push(ScaleVertex {
        position: [q, y + tick_mid, 0.0],
        color,
    });

    vertices
}
