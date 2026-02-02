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
use iced::advanced::text::Renderer as TextRenderer;
use glam::{Mat4, Vec3};
use std::collections::HashSet;
use std::sync::Arc;
use truck_modeling::Solid;
use wgpu::util::DeviceExt;

use crate::cad;
use crate::scene::{
    build_grid_vertices, build_scene_mesh, camera_from_params, intersect_plane, mesh_to_vertex_index,
    pick_entity, ray_from_cursor, AxesVertex, CameraMode, CameraPreset, GridPlane, GridVertex,
    SceneEntity, SceneEntityInfo, ScenePoint, SceneRect, SceneTool, SolidKind, Uniforms, Vertex,
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
            button(text("Open...").size(14)).width(Length::Shrink).on_press(crate::Message::OpenFile),
        ),
        menu::Item::new(
            button(text("Save").size(14)).width(Length::Shrink).on_press(crate::Message::SaveFile),
        ),
        menu::Item::new(
            button(text("Save As...").size(14)).width(Length::Shrink).on_press(crate::Message::SaveFileAs),
        ),
        menu::Item::new(
            button(text("Quit").size(14)).width(Length::Shrink).on_press(crate::Message::Quit),
        ),
    ]);
    let file_menu = menu::Item::with_menu(text("File").size(14), file_sub);

    // Edit submenu
    let edit_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("Undo").size(14)).width(Length::Shrink).on_press(crate::Message::Undo)),
        menu::Item::new(button(text("Redo").size(14)).width(Length::Shrink).on_press(crate::Message::Redo)),
        menu::Item::new(button(text("Cut").size(14)).width(Length::Shrink).on_press(crate::Message::Cut)),
        menu::Item::new(button(text("Copy").size(14)).width(Length::Shrink).on_press(crate::Message::Copy)),
        menu::Item::new(button(text("Paste").size(14)).width(Length::Shrink).on_press(crate::Message::Paste)),
        menu::Item::new(button(text("Delete Selected").size(14)).width(Length::Shrink).on_press(crate::Message::DeleteSelected)),
        menu::Item::new(button(text("Clear").size(14)).width(Length::Shrink).on_press(crate::Message::Clear)),
    ]);
    let edit_menu = menu::Item::with_menu(text("Edit").size(14), edit_sub);

    // View submenu
    let view_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("Toggle 2D Grid").size(14)).width(Length::Shrink).on_press(crate::Message::ToggleGrid)),
        menu::Item::new(button(text("Reset Zoom").size(14)).width(Length::Shrink).on_press(crate::Message::ResetZoom)),
        menu::Item::new(button(text("Toggle Scene Grid").size(14)).width(Length::Shrink).on_press(crate::Message::SceneGridToggle)),
        menu::Item::new(button(text("Toggle Axes").size(14)).width(Length::Shrink).on_press(crate::Message::SceneAxesToggle)),
        menu::Item::new(button(text("Clear Camera Align").size(14)).width(Length::Shrink).on_press(crate::Message::SceneCameraPresetCleared)),
    ]);
    let view_menu = menu::Item::with_menu(text("View").size(14), view_sub);

    // Help submenu
    let help_sub = menu::Menu::new(vec![
        menu::Item::new(button(text("About").size(14)).width(Length::Shrink).on_press(crate::Message::About)),
    ]);
    let help_menu = menu::Item::with_menu(text("Help").size(14), help_sub);

    menu::MenuBar::new(vec![file_menu, edit_menu, view_menu, help_menu]).into()
}

// Tab button builder
pub fn tab_button<'a>(label: &'a str, tab: crate::Tab, active: crate::Tab) -> iced::widget::Button<'a, crate::Message> {
    if tab == active {
        button(text(format!("● {label}"))).on_press_maybe(None)
    } else {
        button(text(label)).on_press(crate::Message::TabChanged(tab))
    }
}

// Scene sidebar: entity list with actions
pub fn build_scene_sidebar<'a>(
    entities: &'a [crate::scene::SceneEntityInfo],
) -> Element<'a, crate::Message> {
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
    entities: &'a [crate::scene::SceneEntityInfo],
}

impl<'a> EntityTree<'a> {
    fn new(entities: &'a [crate::scene::SceneEntityInfo]) -> Self {
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
                renderer.fill_text(
                    toggle,
                    Point::new(x, y),
                    text_color,
                    *viewport,
                );
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

fn solid_label(kind: crate::scene::SolidKind) -> &'static str {
    match kind {
        crate::scene::SolidKind::Box => "Box",
        crate::scene::SolidKind::Sphere => "Sphere",
        crate::scene::SolidKind::Cylinder => "Cylinder",
        crate::scene::SolidKind::Cone => "Cone",
        crate::scene::SolidKind::Torus => "Torus",
    }
}

fn entity_solid(entity: &crate::scene::SceneEntityInfo) -> Solid {
    let size = entity.size as f64;
    let solid = match entity.kind {
        crate::scene::SolidKind::Box => cad::box_solid(size, size, size),
        crate::scene::SolidKind::Sphere => cad::sphere(size * 0.5),
        crate::scene::SolidKind::Cylinder => cad::cylinder_solid(size, size * 0.35),
        crate::scene::SolidKind::Cone => cad::cone_solid(size, size * 0.4),
        crate::scene::SolidKind::Torus => cad::torus_solid(size * 0.7, size * 0.25),
    };
    solid
}

fn to_scene_point(p: Point) -> ScenePoint {
    ScenePoint { x: p.x, y: p.y }
}

fn to_scene_rect(r: Rectangle) -> SceneRect {
    SceneRect::new(r.x, r.y, r.width, r.height)
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
    entities_version: u64,
    uniforms: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    depth: wgpu::TextureView,
    depth_size: (u32, u32),
    last_bounds: (f32, f32, f32, f32),
    // Axes overlay
    axes_pipeline: wgpu::RenderPipeline,
    axes_vertices: wgpu::Buffer,
    axes_vertex_count: u32,
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
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
    selected: Option<u64>,
    axes_enabled: bool,
    axes_size: f32,
    axes_margin: f32,
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

        let scene_bounds = to_scene_rect(*bounds);

        // CAD-like orbit camera around a target point.
        let camera = camera_from_params(
            self.target,
            self.distance,
            self.yaw,
            self.pitch,
            scene_bounds,
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
            if mag > 1.0e-4 { nx /= mag; ny /= mag; } else { nx = 1.0; ny = 0.0; }
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

        if pipeline.entities_version != self.entities_version {
            pipeline.entities_version = self.entities_version;

            if let Some(mesh) = build_scene_mesh(self.entities.as_slice()) {
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
    modifiers: iced_keyboard::Modifiers,
    sphere_center: Option<Vec3>,
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
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
            modifiers: iced_keyboard::Modifiers::default(),
            sphere_center: None,
            preview_line: None,
            preview_version: 0,
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
        if let Event::Keyboard(iced_keyboard::Event::ModifiersChanged(mods)) = event {
            state.modifiers = *mods;
        }

        if let Event::Keyboard(iced_keyboard::Event::KeyPressed { key, .. }) = event {
            if matches!(
                key,
                iced_keyboard::Key::Named(iced_keyboard::key::Named::Delete)
                    | iced_keyboard::Key::Named(iced_keyboard::key::Named::Backspace)
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
                Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Right))
                    | Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Left))
            ) {
                state.dragging = Dragging::None;
                state.last_cursor = None;
                state.drag_start = None;
                return Some(shader::Action::request_redraw().and_capture());
            }

            return None;
        };

        let scene_bounds = to_scene_rect(bounds);

        // Apply camera preset if present
        let (yaw, pitch) = if let Some(p) = self.camera_preset {
            crate::scene::preset_angles(p)
        } else {
            (state.yaw, state.pitch)
        };
        let camera = camera_from_params(state.target, state.distance, yaw, pitch, scene_bounds, self.camera_mode);

        match event {
            Event::Mouse(iced_mouse::Event::WheelScrolled { delta }) => {
                let scroll_y = match *delta {
                    iced_mouse::ScrollDelta::Lines { y, .. } => y,
                    iced_mouse::ScrollDelta::Pixels { y, .. } => y / 120.0,
                };

                if scroll_y.abs() > f32::EPSILON {
                    let factor = 1.1_f32.powf(scroll_y);
                    state.distance = (state.distance / factor).clamp(0.1, 200.0);
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(iced_mouse::Event::ButtonPressed(iced_mouse::Button::Left)) => {
                if self.tool == SceneTool::Sphere {
                    if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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

                let hit = pick_entity(to_scene_point(cursor_pos), scene_bounds, &camera, &state.entities);
                if let Some(id) = hit {
                    state.selected = Some(id);
                    state.dragging = Dragging::MoveEntity;
                } else if let Some(create_kind) = self.tool.create_kind() {
                    if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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

                if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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
            Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Left)) => {
                if matches!(state.dragging, Dragging::MoveEntity | Dragging::CreateEntity) {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    state.drag_start = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(iced_mouse::Event::ButtonPressed(iced_mouse::Button::Right)) => {
                if self.tool == SceneTool::Sphere {
                    if let Some(center) = state.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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
            Event::Mouse(iced_mouse::Event::ButtonReleased(iced_mouse::Button::Right)) => {
                if matches!(state.dragging, Dragging::Pan | Dragging::Rotate) {
                    state.dragging = Dragging::None;
                    state.last_cursor = None;
                    return Some(shader::Action::request_redraw().and_capture());
                }
            }
            Event::Mouse(iced_mouse::Event::CursorMoved { .. }) => {
                if self.tool == SceneTool::Sphere {
                    if let Some(center) = state.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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
                        if let Some((origin, dir)) = ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera) {
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
                            (state.drag_start, ray_from_cursor(to_scene_point(cursor_pos), scene_bounds, &camera))
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

    fn draw(&self, state: &Self::State, _cursor: iced_mouse::Cursor, _bounds: Rectangle) -> Primitive {
        Primitive {
            target: state.target,
            distance: state.distance,
            yaw: if let Some(p) = self.camera_preset { crate::scene::preset_angles(p).0 } else { state.yaw },
            pitch: if let Some(p) = self.camera_preset { crate::scene::preset_angles(p).1 } else { state.pitch },
            camera_mode: self.camera_mode,
            show_grid: self.show_grid,
            grid_plane: self.grid_plane,
            grid_extent: self.grid_extent,
            grid_step: self.grid_step,
            entities: Arc::new(state.entities.clone()),
            entities_version: state.entities_version,
            preview_line: state.preview_line,
            preview_version: state.preview_version,
            selected: state.selected,
            axes_enabled: self.axes_enabled,
            axes_size: self.axes_size,
            axes_margin: self.axes_margin,
        }
    }

    fn mouse_interaction(
        &self,
        state: &Self::State,
        bounds: Rectangle,
        cursor: iced_mouse::Cursor,
    ) -> iced_mouse::Interaction {
        if cursor.position_in(bounds).is_some() && state.dragging != Dragging::None {
            iced_mouse::Interaction::Grabbing
        } else if cursor.position_in(bounds).is_some() {
            iced_mouse::Interaction::Grab
        } else {
            iced_mouse::Interaction::default()
        }
    }
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
