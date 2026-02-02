use iced::widget::{button, column, container, pick_list, row, scrollable, slider, text};
use iced::widget::shader::{self, Viewport};
use iced::{alignment, Color, Element, Event, Length, Pixels, Point, Rectangle, Size, Theme};
use iced::keyboard as iced_keyboard;
use iced::mouse as iced_mouse;
use iced::wgpu;
use iced::advanced::{
    layout, mouse, renderer, text as advanced_text, widget, Clipboard, Layout, Shell, Widget,
    Renderer as AdvancedRenderer,
};
use glam::Mat4;
use std::collections::HashSet;
use truck_modeling::Solid;
use wgpu::util::DeviceExt;
use iced::advanced::text::Renderer as _;

use crate::cad;
use crate::scene::{
    AxesVertex, CameraMode, CameraPreset, GridPlane, GridVertex, Pipeline, SceneConfig,
    SceneEntityInfo, SceneInput, SceneModel, ScenePoint, SceneRect, SceneRequests, SceneTool,
    SceneUpdateResult, SceneView, SolidKind, Uniforms, Vertex,
};
use iced_aw::menu;

// Menu bar builder
pub fn build_menu_bar<'a>(_app: &crate::App) -> Element<'a, crate::Message> {
    // File submenu
    let file_sub = menu::Menu::new(vec![
        menu::Item::new(
            button(text("New").size(14)).width(Length::Shrink).on_press(crate::Message::Clear),
        ),
        menu::Item::new(
            button(text("Open...").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::OpenFile),
        ),
        menu::Item::new(
            button(text("Save").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::SaveFile),
        ),
        menu::Item::new(
            button(text("Save As...").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::SaveFileAs),
        ),
        menu::Item::new(
            button(text("Quit").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Quit),
        ),
    ]);
    let file_menu = menu::Item::with_menu(text("File").size(14), file_sub);

    // Edit submenu
    let edit_sub = menu::Menu::new(vec![
        menu::Item::new(
            button(text("Undo").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Undo),
        ),
        menu::Item::new(
            button(text("Redo").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Redo),
        ),
        menu::Item::new(
            button(text("Cut").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Cut),
        ),
        menu::Item::new(
            button(text("Copy").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Copy),
        ),
        menu::Item::new(
            button(text("Paste").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Paste),
        ),
        menu::Item::new(
            button(text("Delete Selected").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::DeleteSelected),
        ),
        menu::Item::new(
            button(text("Clear").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::Clear),
        ),
    ]);
    let edit_menu = menu::Item::with_menu(text("Edit").size(14), edit_sub);

    // View submenu
    let view_sub = menu::Menu::new(vec![
        menu::Item::new(
            button(text("Toggle 2D Grid").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::ToggleGrid),
        ),
        menu::Item::new(
            button(text("Reset Zoom").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::ResetZoom),
        ),
        menu::Item::new(
            button(text("Toggle Scene Grid").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::SceneGridToggle),
        ),
        menu::Item::new(
            button(text("Toggle Axes").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::SceneAxesToggle),
        ),
        menu::Item::new(
            button(text("Clear Camera Align").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::SceneCameraPresetCleared),
        ),
    ]);
    let view_menu = menu::Item::with_menu(text("View").size(14), view_sub);

    // Help submenu
    let help_sub = menu::Menu::new(vec![
        menu::Item::new(
            button(text("About").size(14))
                .width(Length::Shrink)
                .on_press(crate::Message::About),
        ),
    ]);
    let help_menu = menu::Item::with_menu(text("Help").size(14), help_sub);

    menu::MenuBar::new(vec![file_menu, edit_menu, view_menu, help_menu]).into()
}

// Tab button builder
pub fn tab_button<'a>(
    label: &'a str,
    tab: crate::Tab,
    active: crate::Tab,
) -> iced::widget::Button<'a, crate::Message> {
    if tab == active {
        button(text(format!("● {label}"))).on_press_maybe(None)
    } else {
        button(text(label)).on_press(crate::Message::TabChanged(tab))
    }
}

// Scene sidebar: entity list with actions
pub fn build_scene_sidebar<'a>(entities: &'a [SceneEntityInfo]) -> Element<'a, crate::Message> {
    let tree = EntityTree::new(entities);
    container(scrollable(tree))
        .width(Length::Fixed(360.0))
        .height(Length::Fill)
        .into()
}

const ROW_HEIGHT: f32 = 22.0;
const PADDING: f32 = 8.0;
const INDENT: f32 = 16.0;
const TOGGLE_SIZE: f32 = 12.0;
const ACTION_W: f32 = 44.0;
const ACTION_H: f32 = 18.0;
const ACTION_GAP: f32 = 6.0;
const MAX_CHILD_ITEMS: usize = 80;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum NodeKey {
    Solid(u64),
    Shell(u64, usize),
    Face(u64, usize, usize),
}

#[derive(Debug, Default)]
struct EntityTreeState {
    expanded: HashSet<NodeKey>,
}

#[derive(Debug, Clone, Copy)]
enum RowType {
    Solid { id: u64 },
    Section,
    Leaf,
}

#[derive(Debug, Clone)]
struct Row {
    label: String,
    indent: u16,
    toggle: Option<(NodeKey, bool)>,
    row_type: RowType,
}

struct EntityTree<'a> {
    entities: &'a [SceneEntityInfo],
}

impl<'a> EntityTree<'a> {
    fn new(entities: &'a [SceneEntityInfo]) -> Self {
        Self { entities }
    }

    fn build_rows(&self, state: &EntityTreeState) -> Vec<Row> {
        let mut rows = Vec::new();

        for entity in self.entities {
            let solid = entity_solid(entity);
            let solid_key = NodeKey::Solid(entity.id);
            let solid_expanded = state.expanded.contains(&solid_key);

            rows.push(Row {
                label: format!(
                    "Solid #{} {} (size {:.2})",
                    entity.id,
                    solid_label(entity.kind),
                    entity.size
                ),
                indent: 0,
                toggle: Some((solid_key, solid_expanded)),
                row_type: RowType::Solid { id: entity.id },
            });

            if solid_expanded {
                for (shell_index, shell) in solid.boundaries().iter().enumerate() {
                    let faces_count = shell.face_iter().count();
                    let edges_count = shell.edge_iter().count();
                    let shell_key = NodeKey::Shell(entity.id, shell_index);
                    let shell_expanded = state.expanded.contains(&shell_key);

                    rows.push(Row {
                        label: format!(
                            "Shell {} (Face {}, Edge {})",
                            shell_index + 1,
                            faces_count,
                            edges_count
                        ),
                        indent: 1,
                        toggle: Some((shell_key, shell_expanded)),
                        row_type: RowType::Section,
                    });

                    if shell_expanded {
                        for (face_index, face) in shell.face_iter().enumerate() {
                            let edge_count = face
                                .boundaries()
                                .iter()
                                .map(|wire| wire.edge_iter().count())
                                .sum::<usize>();
                            let face_key = NodeKey::Face(entity.id, shell_index, face_index);
                            let face_expanded = state.expanded.contains(&face_key);

                            rows.push(Row {
                                label: format!("Face {} (Edge {})", face_index + 1, edge_count),
                                indent: 2,
                                toggle: Some((face_key, face_expanded)),
                                row_type: RowType::Section,
                            });

                            if face_expanded {
                                rows.extend(build_leaf_rows("Edge", edge_count, 3));
                            }
                        }
                    }
                }
            }
        }

        rows
    }
}

impl<'a> Widget<crate::Message, Theme, iced::Renderer> for EntityTree<'a> {
    fn size(&self) -> Size<Length> {
        Size::new(Length::Fill, Length::Shrink)
    }

    fn layout(
        &mut self,
        tree: &mut widget::Tree,
        _renderer: &iced::Renderer,
        limits: &layout::Limits,
    ) -> layout::Node {
        let state = tree.state.downcast_ref::<EntityTreeState>();
        let row_count = self.build_rows(state).len();
        let height = PADDING * 2.0 + row_count as f32 * ROW_HEIGHT;
        let limits = limits.width(Length::Fill).height(Length::Shrink);
        let size = limits.resolve(Length::Fill, Length::Shrink, Size::new(0.0, height));
        layout::Node::new(size)
    }

    fn draw(
        &self,
        tree: &widget::Tree,
        renderer: &mut iced::Renderer,
        _theme: &Theme,
        style: &renderer::Style,
        layout: Layout<'_>,
        _cursor: mouse::Cursor,
        viewport: &Rectangle,
    ) {
        let bounds = layout.bounds();
        let state = tree.state.downcast_ref::<EntityTreeState>();
        let rows = self.build_rows(state);

        let text_color = style.text_color;
        let font = renderer.default_font();
        let text_size = renderer.default_size();

        for (index, row) in rows.iter().enumerate() {
            let y = bounds.y + PADDING + index as f32 * ROW_HEIGHT;
            let row_bounds = Rectangle {
                x: bounds.x,
                y,
                width: bounds.width,
                height: ROW_HEIGHT,
            };

            if row_bounds.y + row_bounds.height < viewport.y
                || row_bounds.y > viewport.y + viewport.height
            {
                continue;
            }

            let indent_px = row.indent as f32 * INDENT;
            let mut x = bounds.x + PADDING + indent_px;

            if let Some((_, expanded)) = row.toggle {
                let toggle_text = if expanded { "▾" } else { "▸" };
                let toggle = advanced_text::Text {
                    content: toggle_text.to_string(),
                    bounds: Size::new(TOGGLE_SIZE, ROW_HEIGHT),
                    size: text_size,
                    line_height: advanced_text::LineHeight::Relative(1.0),
                    font,
                    align_x: advanced_text::Alignment::Left,
                    align_y: alignment::Vertical::Center,
                    shaping: advanced_text::Shaping::Auto,
                    wrapping: advanced_text::Wrapping::None,
                };
                renderer.fill_text(toggle, Point::new(x, y), text_color, *viewport);
                x += TOGGLE_SIZE + 4.0;
            }

            let label = advanced_text::Text {
                content: row.label.clone(),
                bounds: Size::new(bounds.width - x - PADDING, ROW_HEIGHT),
                size: text_size,
                line_height: advanced_text::LineHeight::Relative(1.1),
                font,
                align_x: advanced_text::Alignment::Left,
                align_y: alignment::Vertical::Center,
                shaping: advanced_text::Shaping::Auto,
                wrapping: advanced_text::Wrapping::None,
            };
            renderer.fill_text(label, Point::new(x, y), text_color, *viewport);

            if let RowType::Solid { .. } = row.row_type {
                let right = bounds.x + bounds.width - PADDING;
                let ay = y + (ROW_HEIGHT - ACTION_H) * 0.5;
                let focus_rect = Rectangle {
                    x: right - ACTION_W,
                    y: ay,
                    width: ACTION_W,
                    height: ACTION_H,
                };
                let select_rect = Rectangle {
                    x: right - ACTION_W * 2.0 - ACTION_GAP,
                    y: ay,
                    width: ACTION_W,
                    height: ACTION_H,
                };

                draw_action(renderer, focus_rect, "聚焦", text_color, *viewport);
                draw_action(renderer, select_rect, "选中", text_color, *viewport);
            }
        }
    }

    fn tag(&self) -> widget::tree::Tag {
        widget::tree::Tag::of::<EntityTreeState>()
    }

    fn state(&self) -> widget::tree::State {
        let mut expanded = HashSet::new();
        for entity in self.entities {
            expanded.insert(NodeKey::Solid(entity.id));
        }
        widget::tree::State::new(EntityTreeState { expanded })
    }

    fn diff(&self, tree: &mut widget::Tree) {
        let state = tree.state.downcast_mut::<EntityTreeState>();
        for entity in self.entities {
            state.expanded.insert(NodeKey::Solid(entity.id));
        }
    }

    fn update(
        &mut self,
        tree: &mut widget::Tree,
        event: &iced::Event,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _renderer: &iced::Renderer,
        _clipboard: &mut dyn Clipboard,
        shell: &mut Shell<'_, crate::Message>,
        _viewport: &Rectangle,
    ) {
        let state = tree.state.downcast_mut::<EntityTreeState>();
        let bounds = layout.bounds();

        let cursor_pos = match cursor.position_in(bounds) {
            Some(pos) => pos,
            None => return,
        };

        if let iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event {
            let rows = self.build_rows(state);
            let y = cursor_pos.y - bounds.y - PADDING;
            if y < 0.0 {
                return;
            }
            let index = (y / ROW_HEIGHT).floor() as usize;
            if index >= rows.len() {
                return;
            }

            let row = &rows[index];
            let row_y = bounds.y + PADDING + index as f32 * ROW_HEIGHT;
            let indent_px = row.indent as f32 * INDENT;
            let toggle_x = bounds.x + PADDING + indent_px;
            let toggle_rect = Rectangle {
                x: toggle_x,
                y: row_y,
                width: TOGGLE_SIZE,
                height: ROW_HEIGHT,
            };

            if let Some((key, expanded)) = row.toggle {
                if toggle_rect.contains(cursor_pos) {
                    if expanded {
                        state.expanded.remove(&key);
                    } else {
                        state.expanded.insert(key);
                    }
                    shell.invalidate_layout();
                    shell.request_redraw();
                    shell.capture_event();
                    return;
                }
            }

            if let RowType::Solid { id } = row.row_type {
                let right = bounds.x + bounds.width - PADDING;
                let ay = row_y + (ROW_HEIGHT - ACTION_H) * 0.5;
                let focus_rect = Rectangle {
                    x: right - ACTION_W,
                    y: ay,
                    width: ACTION_W,
                    height: ACTION_H,
                };
                let select_rect = Rectangle {
                    x: right - ACTION_W * 2.0 - ACTION_GAP,
                    y: ay,
                    width: ACTION_W,
                    height: ACTION_H,
                };

                if select_rect.contains(cursor_pos) {
                    shell.publish(crate::Message::SceneSelectEntity(id));
                    shell.capture_event();
                    return;
                }

                if focus_rect.contains(cursor_pos) {
                    shell.publish(crate::Message::SceneFocusEntity(id));
                    shell.capture_event();
                    return;
                }
            }
        }
    }

    fn mouse_interaction(
        &self,
        _tree: &widget::Tree,
        layout: Layout<'_>,
        cursor: mouse::Cursor,
        _viewport: &Rectangle,
        _renderer: &iced::Renderer,
    ) -> mouse::Interaction {
        if cursor.position_in(layout.bounds()).is_some() {
            mouse::Interaction::Pointer
        } else {
            mouse::Interaction::default()
        }
    }
}

impl<'a> From<EntityTree<'a>> for Element<'a, crate::Message> {
    fn from(widget: EntityTree<'a>) -> Self {
        Element::new(widget)
    }
}

fn draw_action(
    renderer: &mut iced::Renderer,
    bounds: Rectangle,
    label: &str,
    text_color: Color,
    viewport: Rectangle,
) {
    let quad = renderer::Quad {
        bounds,
        border: iced::Border {
            color: Color::from_rgba(0.4, 0.4, 0.45, 0.6),
            width: 1.0,
            radius: 4.0.into(),
        },
        shadow: iced::Shadow::default(),
        snap: true,
    };
    renderer.fill_quad(quad, Color::from_rgba(0.2, 0.2, 0.25, 0.08));

    let text = advanced_text::Text {
        content: label.to_string(),
        bounds: Size::new(bounds.width, bounds.height),
        size: Pixels::from(12.0),
        line_height: advanced_text::LineHeight::Relative(1.0),
        font: renderer.default_font(),
        align_x: advanced_text::Alignment::Center,
        align_y: alignment::Vertical::Center,
        shaping: advanced_text::Shaping::Auto,
        wrapping: advanced_text::Wrapping::None,
    };
    renderer.fill_text(text, Point::new(bounds.x, bounds.y), text_color, viewport);
}

fn build_leaf_rows(label: &str, total: usize, indent: u16) -> Vec<Row> {
    let mut rows = Vec::new();
    let max_items = total.min(MAX_CHILD_ITEMS);
    for i in 0..max_items {
        rows.push(Row {
            label: format!("{} {}", label, i + 1),
            indent,
            toggle: None,
            row_type: RowType::Leaf,
        });
    }
    if total > max_items {
        rows.push(Row {
            label: format!("... 还有 {} 个", total - max_items),
            indent,
            toggle: None,
            row_type: RowType::Leaf,
        });
    }
    rows
}

fn solid_label(kind: SolidKind) -> &'static str {
    match kind {
        SolidKind::Box => "Box",
        SolidKind::Sphere => "Sphere",
        SolidKind::Cylinder => "Cylinder",
        SolidKind::Cone => "Cone",
        SolidKind::Torus => "Torus",
    }
}

fn entity_solid(entity: &SceneEntityInfo) -> Solid {
    let size = entity.size as f64;
    let solid = match entity.kind {
        SolidKind::Box => cad::box_solid(size, size, size),
        SolidKind::Sphere => cad::sphere(size * 0.5),
        SolidKind::Cylinder => cad::cylinder_solid(size, size * 0.35),
        SolidKind::Cone => cad::cone_solid(size, size * 0.4),
        SolidKind::Torus => cad::torus_solid(size * 0.7, size * 0.25),
    };
    solid
}

fn to_scene_point(p: Point) -> ScenePoint {
    ScenePoint { x: p.x, y: p.y }
}

fn to_scene_rect(r: Rectangle) -> SceneRect {
    SceneRect::new(r.x, r.y, r.width, r.height)
}

impl shader::Pipeline for Pipeline {
    fn new(
        device: &wgpu::Device,
        _queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
    ) -> Self {
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
    view: SceneView,
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
        self.view.do_prepare(
            pipeline,
            device,
            queue,
            to_scene_rect(*bounds),
            {
                let size = viewport.physical_size();
                (size.width, size.height)
            },
            viewport.scale_factor(),
        );
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
        self.view
            .do_render(pipeline, encoder, target, clip_bounds);
    }
}

#[derive(Clone)]
struct Scene<Message> {
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
    on_entities_snapshot: Option<std::rc::Rc<dyn Fn(Vec<SceneEntityInfo>) -> Message + 'static>>,
    request_select_id: Option<u64>,
    request_focus_id: Option<u64>,
}

#[derive(Debug, Clone)]
struct SceneState {
    model: SceneModel,
}

impl Default for SceneState {
    fn default() -> Self {
        Self {
            model: SceneModel::default(),
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
        cursor: iced_mouse::Cursor,
    ) -> Option<shader::Action<Message>> {
        let config = SceneConfig {
            show_grid: self.show_grid,
            grid_plane: self.grid_plane,
            grid_extent: self.grid_extent,
            grid_step: self.grid_step,
            tool: self.tool,
            camera_mode: self.camera_mode,
            camera_preset: self.camera_preset,
            axes_enabled: self.axes_enabled,
            axes_size: self.axes_size,
            axes_margin: self.axes_margin,
        };
        let requests = SceneRequests {
            select_id: self.request_select_id,
            focus_id: self.request_focus_id,
        };

        if let Event::Keyboard(iced_keyboard::Event::ModifiersChanged(mods)) = event {
            let result = state.model.update(
                SceneInput::ModifiersChanged {
                    shift: mods.shift(),
                },
                to_scene_rect(bounds),
                config,
                requests,
            );
            return scene_action(result, &self.on_entities_snapshot);
        }

        if let Event::Keyboard(iced_keyboard::Event::KeyPressed { key, .. }) = event {
            if matches!(
                key,
                iced_keyboard::Key::Named(iced_keyboard::key::Named::Delete)
                    | iced_keyboard::Key::Named(iced_keyboard::key::Named::Backspace)
            ) {
                let result = state.model.update(
                    SceneInput::KeyDelete,
                    to_scene_rect(bounds),
                    config,
                    requests,
                );
                return scene_action(result, &self.on_entities_snapshot);
            }
        }

        let cursor_pos = cursor.position_in(bounds);
        let scene_bounds = to_scene_rect(bounds);

        let input = match (event, cursor_pos) {
            (Event::Mouse(iced_mouse::Event::WheelScrolled { delta }), Some(_)) => {
                let scroll_y = match *delta {
                    iced_mouse::ScrollDelta::Lines { y, .. } => y,
                    iced_mouse::ScrollDelta::Pixels { y, .. } => y / 120.0,
                };
                Some(SceneInput::MouseWheel {
                    delta_lines: scroll_y,
                })
            }
            (Event::Mouse(iced_mouse::Event::ButtonPressed(iced_mouse::Button::Left)), Some(pos)) => {
                Some(SceneInput::MouseDownLeft {
                    pos: to_scene_point(pos),
                })
            }
            (Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Left)), _) => {
                Some(SceneInput::MouseUpLeft)
            }
            (Event::Mouse(iced_mouse::Event::ButtonPressed(iced_mouse::Button::Right)), Some(pos)) => {
                Some(SceneInput::MouseDownRight {
                    pos: to_scene_point(pos),
                })
            }
            (Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Right)), _) => {
                Some(SceneInput::MouseUpRight)
            }
            (Event::Mouse(iced_mouse::Event::CursorMoved { .. }), Some(pos)) => {
                Some(SceneInput::MouseMove {
                    pos: to_scene_point(pos),
                })
            }
            _ => None,
        };

        if let Some(input) = input {
            let result = state.model.update(input, scene_bounds, config, requests);
            return scene_action(result, &self.on_entities_snapshot);
        }

        None
    }

    fn draw(&self, state: &Self::State, _cursor: iced_mouse::Cursor, _bounds: Rectangle) -> Primitive {
        let config = SceneConfig {
            show_grid: self.show_grid,
            grid_plane: self.grid_plane,
            grid_extent: self.grid_extent,
            grid_step: self.grid_step,
            tool: self.tool,
            camera_mode: self.camera_mode,
            camera_preset: self.camera_preset,
            axes_enabled: self.axes_enabled,
            axes_size: self.axes_size,
            axes_margin: self.axes_margin,
        };
        Primitive {
            view: state.model.view(config),
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: iced_mouse::Cursor,
    ) -> iced_mouse::Interaction {
        if cursor.position_in(bounds).is_some() && state.model.is_dragging() {
            iced_mouse::Interaction::Grabbing
        } else if cursor.position_in(bounds).is_some() {
            iced_mouse::Interaction::Grab
        } else {
            iced_mouse::Interaction::default()
        }
    }
}

fn scene_action<Message>(
    result: SceneUpdateResult,
    on_entities_snapshot: &Option<std::rc::Rc<dyn Fn(Vec<SceneEntityInfo>) -> Message + 'static>>,
) -> Option<shader::Action<Message>> {
    if !result.handled && result.publish_entities.is_none() && !result.request_redraw {
        return None;
    }

    let mut action = if let Some(list) = result.publish_entities {
        if let Some(cb) = on_entities_snapshot {
            shader::Action::publish(cb(list))
        } else {
            shader::Action::request_redraw()
        }
    } else if result.request_redraw {
        shader::Action::request_redraw()
    } else {
        return None;
    };

    if result.capture {
        action = action.and_capture();
    }

    Some(action)
}

pub fn scene_widget<'a, Message>(
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
        on_entities_snapshot: Some(std::rc::Rc::new(on_entities_snapshot)),
        request_select_id,
        request_focus_id,
    })
    .width(Length::Fill)
    .height(Length::Fill)
    .into()
}

// Draw tab toolbar
pub fn draw_toolbar<'a>(
    tool: crate::Tool,
    mode: crate::BooleanMode,
    stroke_width: f32,
    show_grid: bool,
    show_menubar: bool,
    selected: Option<usize>,
) -> Element<'a, crate::Message> {
    row![
        text("Tool"),
        pick_list(crate::Tool::ALL.as_slice(), Some(tool), crate::Message::ToolChanged)
            .width(Length::Fixed(120.0)),
        text("Mode"),
        pick_list(crate::BooleanMode::ALL.as_slice(), Some(mode), crate::Message::ModeChanged)
            .width(Length::Fixed(120.0)),
        text("Stroke"),
        slider(1.0..=16.0, stroke_width, crate::Message::StrokeWidthChanged)
            .width(Length::Fixed(180.0)),
        text(format!("{:.1}", stroke_width)),
        button(if show_grid { "Grid: On" } else { "Grid: Off" })
            .on_press(crate::Message::ToggleGrid),
        button("Reset Zoom").on_press(crate::Message::ResetZoom),
        button(if show_menubar { "MenuBar: On" } else { "MenuBar: Off" })
            .on_press(crate::Message::ToggleMenuBar),
        button("Undo").on_press(crate::Message::Undo),
        button("Delete Selected")
            .on_press_maybe(selected.map(|_| crate::Message::DeleteSelected)),
        button("Clear").on_press(crate::Message::Clear),
    ]
    .spacing(10)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn stroke_row<'a>() -> Element<'a, crate::Message> {
    row![
        text("Stroke"),
        button("Black").on_press(crate::Message::StrokeColorChanged(iced::Color::BLACK)),
        button("Red")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0xE6, 0x2E, 0x2E))),
        button("Green")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0x2E, 0xE6, 0x6B))),
        button("Blue")
            .on_press(crate::Message::StrokeColorChanged(iced::Color::from_rgb8(0x2E, 0xB8, 0xE6))),
    ]
    .spacing(8)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn fill_row<'a>() -> Element<'a, crate::Message> {
    row![
        text("Fill"),
        button("None").on_press(crate::Message::FillColorChanged(None)),
        button("Light Blue")
            .on_press(crate::Message::FillColorChanged(Some(iced::Color::from_rgb8(0xB3, 0xD9, 0xFF)))),
        button("Light Red")
            .on_press(crate::Message::FillColorChanged(Some(iced::Color::from_rgb8(0xFF, 0xB3, 0xB3)))),
    ]
    .spacing(8)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn status_row<'a>(
    tool: crate::Tool,
    zoom: f32,
    cursor_pos: Option<iced::Point>,
) -> Element<'a, crate::Message> {
    let hint = match tool {
        crate::Tool::Select => "Select: left click a shape; delete selected",
        crate::Tool::Spline => "Spline: left click to add points, right click to finish",
        crate::Tool::Polygon => "Polygon: left click to add points, right click to close",
        _ => "Drag to draw: press-move-release",
    };

    row![
        text(format!("Zoom: {:.0}%", zoom * 100.0)),
        text("|"),
        text(match cursor_pos {
            Some(p) => format!("Coords: {:.1}, {:.1}", p.x, p.y),
            None => String::from("Coords: -, -"),
        }),
        text("|"),
        text(hint),
    ]
    .spacing(10)
    .align_y(iced::Alignment::Center)
    .into()
}

pub fn scene_controls<'a>(
    scene_show_grid: bool,
    scene_grid_plane: crate::GridPlane,
    scene_tool: crate::SceneTool,
    scene_camera_mode: crate::CameraMode,
    scene_camera_preset: Option<crate::CameraPreset>,
    scene_axes_enabled: bool,
    scene_axes_size: f32,
    scene_axes_margin: f32,
    scene_grid_extent: f32,
    scene_grid_step: f32,
    gmsh_status: Option<&'a str>,
) -> Element<'a, crate::Message> {
    column![
        row![
            button("Load Gmsh...").on_press(crate::Message::LoadGmsh),
            text(gmsh_status.unwrap_or("No Gmsh mesh loaded")),
            button("Zoom In").on_press(crate::Message::SceneZoomIn),
            button("Zoom Out").on_press(crate::Message::SceneZoomOut),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            button(if scene_show_grid { "Grid: On" } else { "Grid: Off" })
                .on_press(crate::Message::SceneGridToggle),
            text("Plane"),
            pick_list(
                crate::GridPlane::ALL.as_slice(),
                Some(scene_grid_plane),
                crate::Message::SceneGridPlaneChanged,
            )
            .width(Length::Fixed(120.0)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Primitive"),
            pick_list(
                crate::SceneTool::ALL.as_slice(),
                Some(scene_tool),
                crate::Message::SceneToolChanged,
            )
            .width(Length::Fixed(160.0)),
            text("Create with left click + drag"),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Camera"),
            pick_list(
                crate::CameraMode::ALL.as_slice(),
                Some(scene_camera_mode),
                crate::Message::SceneCameraModeChanged,
            )
            .width(Length::Fixed(160.0)),
            text("Align"),
            pick_list(
                crate::CameraPreset::ALL.as_slice(),
                scene_camera_preset,
                crate::Message::SceneCameraPresetChanged,
            )
            .width(Length::Fixed(160.0)),
            button("Clear Align").on_press(crate::Message::SceneCameraPresetCleared),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            button(if scene_axes_enabled { "Axes: On" } else { "Axes: Off" })
                .on_press(crate::Message::SceneAxesToggle),
            text("Size"),
            slider(24.0..=240.0, scene_axes_size, crate::Message::SceneAxesSizeChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.0}", scene_axes_size)),
            text("Margin"),
            slider(0.0..=64.0, scene_axes_margin, crate::Message::SceneAxesMarginChanged)
                .width(Length::Fixed(160.0)),
            text(format!("{:.0}", scene_axes_margin)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
        row![
            text("Range"),
            slider(0.5..=10.0, scene_grid_extent, crate::Message::SceneGridExtentChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.2}", scene_grid_extent)),
            text("Density"),
            slider(0.05..=2.0, scene_grid_step, crate::Message::SceneGridStepChanged)
                .width(Length::Fixed(200.0)),
            text(format!("{:.2}", scene_grid_step)),
        ]
        .spacing(10)
        .align_y(iced::Alignment::Center),
    ]
    .spacing(8)
    .into()
}
