use glam::Vec3;
use wgpu;

use crate::camera::Camera;

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(crate) struct AxesVertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
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

pub(crate) fn create_axes_pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
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

    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
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
            format: wgpu::TextureFormat::Depth24Plus,
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
    })
}

pub(crate) fn create_empty_axes_buffer(device: &wgpu::Device) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("scene_axes_vertices"),
        size: 4,
        usage: wgpu::BufferUsages::VERTEX,
        mapped_at_creation: false,
    })
}

pub(crate) fn build_axes_vertices(camera: &Camera) -> Vec<AxesVertex> {
    let len = 0.9_f32;
    let mut axes_vertices: Vec<AxesVertex> = Vec::with_capacity(32);

    let mut add_axis_with_label = |v: Vec3, color: [f32; 3], ch: char| {
        let x = v.dot(camera.right);
        let y = v.dot(camera.up);
        let ex = x * len;
        let ey = y * len;

        axes_vertices.push(AxesVertex { position: [0.0, 0.0, 0.0], color });
        axes_vertices.push(AxesVertex { position: [ex, ey, 0.0], color });

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
        let pad = 0.08_f32;
        let px = ex + nx * pad;
        let py = ey + ny * pad;
        let s = 0.12_f32;

        match ch {
            'X' => {
                axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py - s, 0.0], color });
            }
            'Y' => {
                axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                axes_vertices.push(AxesVertex { position: [px,     py,     0.0], color });
                axes_vertices.push(AxesVertex { position: [px,     py - s, 0.0], color });
            }
            'Z' => {
                axes_vertices.push(AxesVertex { position: [px - s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py + s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px - s, py - s, 0.0], color });
                axes_vertices.push(AxesVertex { position: [px + s, py - s, 0.0], color });
            }
            _ => {}
        }
    };

    add_axis_with_label(Vec3::X, [0.95, 0.3, 0.3], 'X');
    add_axis_with_label(Vec3::Y, [0.4, 0.9, 0.5], 'Y');
    add_axis_with_label(Vec3::Z, [0.35, 0.6, 0.95], 'Z');

    axes_vertices
}
