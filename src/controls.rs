use iced::widget::{button, column, container, pick_list, row, scrollable, slider, text};
use iced::{alignment, Color, Element, Length, Pixels, Point, Rectangle, Size, Theme};
use iced::advanced::{
    layout, mouse, renderer, text as advanced_text, widget, Clipboard, Layout, Shell, Widget,
    Renderer as AdvancedRenderer,
};
use iced::advanced::text::Renderer as TextRenderer;
use std::collections::HashSet;
use truck_modeling::Solid;

use crate::cad;
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
