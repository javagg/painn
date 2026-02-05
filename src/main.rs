mod drawing;
#[path = "scene/mod.rs"]
mod scene;
mod canvas;
mod cad;
mod controls;
mod camera;
mod ui;


use drawing::{CanvasEvent, Draft, Shape, Tool};
use scene::{CameraMode, CameraPreset, GridPlane, SceneEntityInfo, SceneTool};
use ui::{ribbon, RibbonAction, RibbonTab};
use iced::widget::{column, container, row};
use iced::{Alignment, Color, Element, Length, Size, Task};
#[cfg(not(target_arch = "wasm32"))]
use std::path::PathBuf;
use std::sync::Arc;
use truck_polymesh::PolygonMesh;

use i_float::int::point::IntPoint;
use i_overlay::core::fill_rule::FillRule;
use i_overlay::core::overlay::Overlay;
use i_overlay::core::overlay_rule::OverlayRule;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BooleanMode {
    Add,
    Merge,
    Mask,
    Diff,
}

impl BooleanMode {
    const ALL: [BooleanMode; 4] = [
        BooleanMode::Add,
        BooleanMode::Merge,
        BooleanMode::Mask,
        BooleanMode::Diff,
    ];

    fn label(self) -> &'static str {
        match self {
            BooleanMode::Add => "Add",
            BooleanMode::Merge => "Merge",
            BooleanMode::Mask => "Mask",
            BooleanMode::Diff => "Diff",
        }
    }
}

impl std::fmt::Display for BooleanMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Draw,
    Scene,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SceneSidebarTab {
    Entities,
}

#[derive(Debug, Clone)]
enum Message {
    TabChanged(Tab),
    ToolChanged(Tool),
    ModeChanged(BooleanMode),
    SceneGridToggle,
    SceneGridPlaneChanged(GridPlane),
    SceneGridExtentChanged(f32),
    SceneGridStepChanged(f32),
    SceneToolChanged(SceneTool),
    SceneCameraModeChanged(CameraMode),
    SceneCameraPresetChanged(CameraPreset),
    SceneCameraPresetCleared,
    SceneAxesToggle,
    SceneAxesSizeChanged(f32),
    SceneAxesMarginChanged(f32),
    SceneBackgroundPickerOpened,
    SceneBackgroundPickerCancelled,
    SceneBackgroundChanged(Color),
    StrokeWidthChanged(f32),
    StrokeColorChanged(Color),
    FillColorChanged(Option<Color>),
    ToggleGrid,
    ResetZoom,
    Undo,
    DeleteSelected,
    Clear,
    // File
    OpenFile,
    SaveFile,
    SaveFileAs,
    Quit,
    // Edit
    Redo,
    Cut,
    Copy,
    Paste,
    // Help
    About,
    // Scene entities snapshot
    SceneEntitiesSnapshot(Vec<SceneEntityInfo>),
    // Scene actions from sidebar
    SceneSelectEntity(u64),
    SceneFocusEntity(u64),
    SceneSidebarTabSelected(SceneSidebarTab),
    SceneZoomIn,
    SceneZoomOut,
    SceneToolCompleted(SceneTool),
    GmshLoaded(Result<Option<GmshLoadResult>, String>),
    Canvas(CanvasEvent),
    RibbonTabChanged(RibbonTab),
    RibbonAction(RibbonAction),
}

fn wrap_canvas_event(e: CanvasEvent) -> Message {
    Message::Canvas(e)
}

#[derive(Debug, Clone)]
struct GmshLoadResult {
    mesh: PolygonMesh,
    label: String,
}

#[cfg(not(target_arch = "wasm32"))]
async fn pick_and_load_gmsh() -> Result<Option<GmshLoadResult>, String> {
    let file = rfd::AsyncFileDialog::new()
        .add_filter("Gmsh", &["msh"])
        .set_title("Open Gmsh Mesh")
        .pick_file()
        .await;

    let Some(handle) = file else {
        return Ok(None);
    };

    let path = handle.path().to_path_buf();

    let mesh = cad::load_gmsh_mesh(&path)?;
    Ok(Some(GmshLoadResult {
        mesh,
        label: path_label(&path),
    }))
}

#[cfg(target_arch = "wasm32")]
async fn pick_and_load_gmsh() -> Result<Option<GmshLoadResult>, String> {
    let file = rfd::AsyncFileDialog::new()
        .add_filter("Gmsh", &["msh"])
        .set_title("Open Gmsh Mesh")
        .pick_file()
        .await;

    let Some(handle) = file else {
        return Ok(None);
    };

    let bytes = handle.read().await;
    let mesh = cad::load_gmsh_mesh_from_bytes(&bytes)?;
    let label = handle.file_name();

    Ok(Some(GmshLoadResult { mesh, label }))
}

#[cfg(not(target_arch = "wasm32"))]
fn path_label(path: &PathBuf) -> String {
    path.file_name()
        .and_then(|s| s.to_str())
        .map(|s| s.to_string())
        .unwrap_or_else(|| path.display().to_string())
}

#[derive(Debug)]
struct App {
    active_tab: Tab,
    tool: Tool,
    mode: BooleanMode,
    scene_show_grid: bool,
    scene_grid_plane: GridPlane,
    scene_grid_extent: f32,
    scene_grid_step: f32,
    scene_tool: SceneTool,
    scene_camera_mode: CameraMode,
    scene_camera_preset: Option<CameraPreset>,
    scene_axes_enabled: bool,
    scene_axes_size: f32,
    scene_axes_margin: f32,
    scene_bg_color: Color,
    scene_bg_picker_open: bool,
    stroke_width: f32,
    stroke_color: Color,
    fill_color: Option<Color>,
    show_grid: bool,
    zoom: f32,
    view_offset: iced::Point,
    cursor_pos: Option<iced::Point>,
    shapes: Vec<Shape>,
    selected: Option<usize>,
    dragging: bool,
    drag_last: Option<iced::Point>,
    draft: Draft,
    show_menubar: bool,
    scene_entities: Vec<SceneEntityInfo>,
    ribbon_active_action: Option<RibbonAction>,
    scene_request_select_id: Option<u64>,
    scene_request_focus_id: Option<u64>,
    scene_zoom_factor: f32,
    scene_zoom_version: u64,
    scene_unite_version: u64,
    gmsh_mesh: Option<Arc<PolygonMesh>>,
    gmsh_mesh_version: u64,
    gmsh_status: Option<String>,
    ribbon_tab: RibbonTab,
}

impl App {
    fn new() -> Self {
        Self {
            active_tab: Tab::Scene,
            tool: Tool::Line,
            mode: BooleanMode::Add,
            scene_show_grid: true,
            scene_grid_plane: GridPlane::XZ,
            scene_grid_extent: 2.5,
            scene_grid_step: 0.25,
            scene_tool: SceneTool::Select,
            scene_camera_mode: CameraMode::Orthographic,
            scene_camera_preset: Some(CameraPreset::Isometric),
            scene_axes_enabled: true,
            scene_axes_size: 100.0,
            scene_axes_margin: 8.0,
            scene_bg_color: Color::from_rgb8(0x1E, 0x1F, 0x24),
            scene_bg_picker_open: false,
            stroke_width: 3.0,
            stroke_color: Color::from_rgb8(0xE6, 0xE6, 0xE6),
            fill_color: None,
            show_grid: true,
            zoom: 1.0,
            view_offset: iced::Point::ORIGIN,
            cursor_pos: None,
            shapes: Vec::new(),
            selected: None,
            dragging: false,
            drag_last: None,
            draft: Draft::default(),
            show_menubar: true,
            scene_entities: Vec::new(),
            scene_request_select_id: None,
            scene_request_focus_id: None,
            scene_zoom_factor: 1.0,
            scene_zoom_version: 0,
            scene_unite_version: 0,
            gmsh_mesh: None,
            gmsh_mesh_version: 0,
            gmsh_status: None,
            ribbon_tab: RibbonTab::Main,
            ribbon_active_action: None,
        }
    }

    fn invalidate(&mut self) {
        // Vello-backed canvas does not use iced::canvas caching.
    }

    fn reset_draft(&mut self) {
        self.draft = Draft::default();
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::TabChanged(tab) => {
                self.active_tab = tab;
                self.invalidate();
            }
            Message::ToolChanged(tool) => {
                self.tool = tool;
                self.reset_draft();
                self.dragging = false;
                self.drag_last = None;
                self.invalidate();
            }
            Message::ModeChanged(mode) => {
                self.mode = mode;
                self.invalidate();
            }
            Message::SceneGridToggle => {
                self.scene_show_grid = !self.scene_show_grid;
                self.invalidate();
            }
            Message::SceneGridPlaneChanged(plane) => {
                self.scene_grid_plane = plane;
                self.invalidate();
            }
            Message::SceneGridExtentChanged(value) => {
                self.scene_grid_extent = value.max(0.5);
                self.invalidate();
            }
            Message::SceneGridStepChanged(value) => {
                self.scene_grid_step = value.max(0.05);
                self.invalidate();
            }
            Message::SceneToolChanged(tool) => {
                self.scene_tool = tool;
                self.invalidate();
            }
            Message::SceneCameraModeChanged(mode) => {
                self.scene_camera_mode = mode;
                self.invalidate();
            }
            Message::SceneCameraPresetChanged(p) => {
                self.scene_camera_preset = Some(p);
                self.invalidate();
            }
            Message::SceneCameraPresetCleared => {
                self.scene_camera_preset = None;
                self.invalidate();
            }
            Message::SceneAxesToggle => {
                self.scene_axes_enabled = !self.scene_axes_enabled;
                self.invalidate();
            }
            Message::SceneAxesSizeChanged(v) => {
                self.scene_axes_size = v.clamp(24.0, 240.0);
                self.invalidate();
            }
            Message::SceneAxesMarginChanged(v) => {
                self.scene_axes_margin = v.clamp(0.0, 64.0);
                self.invalidate();
            }
            Message::SceneBackgroundPickerOpened => {
                self.scene_bg_picker_open = true;
            }
            Message::SceneBackgroundPickerCancelled => {
                self.scene_bg_picker_open = false;
            }
            Message::SceneBackgroundChanged(color) => {
                self.scene_bg_color = color;
                self.scene_bg_picker_open = false;
                self.invalidate();
            }
            Message::StrokeWidthChanged(w) => {
                self.stroke_width = w;
                if let Some(index) = self.selected {
                    apply_stroke_width(&mut self.shapes[index], w);
                }
                self.invalidate();
            }
            Message::StrokeColorChanged(c) => {
                self.stroke_color = c;
                if let Some(index) = self.selected {
                    apply_stroke_color(&mut self.shapes[index], c);
                }
                self.invalidate();
            }
            Message::FillColorChanged(c) => {
                self.fill_color = c;
                if let Some(index) = self.selected {
                    apply_fill_color(&mut self.shapes[index], c);
                }
                self.invalidate();
            }
            Message::ToggleGrid => {
                self.show_grid = !self.show_grid;
                self.invalidate();
            }
            Message::ResetZoom => {
                if (self.zoom - 1.0).abs() > f32::EPSILON {
                    self.zoom = 1.0;
                    self.view_offset = iced::Point::ORIGIN;
                    self.invalidate();
                }
            }
            Message::Undo => {
                self.shapes.pop();
                self.selected = None;
                self.dragging = false;
                self.drag_last = None;
                self.reset_draft();
                self.invalidate();
            }
            Message::Redo => {
                // Not implemented yet
            }
            Message::DeleteSelected => {
                if let Some(index) = self.selected {
                    self.shapes.remove(index);
                    self.selected = None;
                    self.dragging = false;
                    self.drag_last = None;
                    self.invalidate();
                }
            }
            Message::Clear => {
                self.shapes.clear();
                self.selected = None;
                self.dragging = false;
                self.drag_last = None;
                self.reset_draft();
                self.invalidate();
            }
            Message::OpenFile => {
                return Task::perform(pick_and_load_gmsh(), Message::GmshLoaded);
            }
            Message::SaveFile => {
                // TODO: save current document
            }
            Message::SaveFileAs => {
                // TODO: save as dialog
            }
            Message::Cut => {
                // TODO: cut selection
            }
            Message::Copy => {
                // TODO: copy selection
            }
            Message::Paste => {
                // TODO: paste from clipboard
            }
            Message::About => {
                // TODO: show about dialog
            }
            Message::SceneEntitiesSnapshot(list) => {
                self.scene_entities = list;
                self.invalidate();
            }
            Message::SceneSelectEntity(id) => {
                self.scene_request_select_id = Some(id);
                self.invalidate();
            }
            Message::SceneFocusEntity(id) => {
                self.scene_request_focus_id = Some(id);
                self.invalidate();
            }
            Message::SceneSidebarTabSelected(_tab) => {
                // Single-tab sidebar currently; no state update required.
            }
            Message::SceneZoomIn => {
                self.scene_zoom_factor = 1.1;
                self.scene_zoom_version = self.scene_zoom_version.wrapping_add(1);
                self.invalidate();
            }
            Message::SceneZoomOut => {
                self.scene_zoom_factor = 1.0 / 1.1;
                self.scene_zoom_version = self.scene_zoom_version.wrapping_add(1);
                self.invalidate();
            }
            Message::SceneToolCompleted(tool) => {
                if tool.is_sketch() {
                    self.scene_tool = SceneTool::Select;
                    self.ribbon_active_action = None;
                    self.invalidate();
                }
            }
            // Message::LoadGmsh => {
            //     return Task::perform(pick_and_load_gmsh(), Message::GmshLoaded);
            // }
            Message::GmshLoaded(result) => {
                match result {
                    Ok(Some(loaded)) => {
                        self.gmsh_mesh = Some(Arc::new(loaded.mesh));
                        self.gmsh_mesh_version = self.gmsh_mesh_version.wrapping_add(1);
                        self.gmsh_status = Some(format!("Gmsh: {}", loaded.label));
                        self.invalidate();
                    }
                    Ok(None) => {}
                    Err(err) => {
                        self.gmsh_status = Some(format!("Gmsh load failed: {err}"));
                        self.invalidate();
                    }
                }
            }
            Message::RibbonTabChanged(tab) => {
                self.ribbon_tab = tab;
                self.invalidate();
            }
            Message::RibbonAction(action) => {
                match action {
                    RibbonAction::Open => {
                        return Task::perform(pick_and_load_gmsh(), Message::GmshLoaded);
                    }
                    RibbonAction::Save => {
                        return self.update(Message::SaveFile);
                    }
                    RibbonAction::SaveAs => {
                        return self.update(Message::SaveFileAs);
                    }
                    RibbonAction::ZoomIn => {
                        return self.update(Message::SceneZoomIn);
                    }
                    RibbonAction::ZoomOut => {
                        return self.update(Message::SceneZoomOut);
                    }
                    RibbonAction::Background => {
                        return self.update(Message::SceneBackgroundPickerOpened);
                    }
                    RibbonAction::GridToggle => {
                        return self.update(Message::SceneGridToggle);
                    }
                    RibbonAction::GridPlaneNext => {
                        let next = match self.scene_grid_plane {
                            GridPlane::XY => GridPlane::YZ,
                            GridPlane::YZ => GridPlane::XZ,
                            GridPlane::XZ => GridPlane::XY,
                        };
                        return self.update(Message::SceneGridPlaneChanged(next));
                    }
                    RibbonAction::Unite => {
                        self.scene_unite_version =
                            self.scene_unite_version.wrapping_add(1);
                    }
                    RibbonAction::Point => {
                        self.scene_tool = SceneTool::SketchPoint;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Line => {
                        self.scene_tool = SceneTool::SketchLine;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Rect => {
                        self.scene_tool = SceneTool::SketchRect;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Polygon => {
                        self.scene_tool = SceneTool::SketchPolygon;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Circle => {
                        self.scene_tool = SceneTool::SketchCircle;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Ellipse => {
                        self.scene_tool = SceneTool::SketchEllipse;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Arc => {
                        self.scene_tool = SceneTool::SketchArc;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Bezier => {
                        self.scene_tool = SceneTool::SketchBezier;
                        self.ribbon_active_action = Some(action);
                    }
                    RibbonAction::Box => {
                        self.scene_tool = SceneTool::Box;
                        self.ribbon_active_action = None;
                    }
                    RibbonAction::Sphere => {
                        self.scene_tool = SceneTool::Sphere;
                        self.ribbon_active_action = None;
                    }
                    RibbonAction::Cylinder => {
                        self.scene_tool = SceneTool::Cylinder;
                        self.ribbon_active_action = None;
                    }
                    RibbonAction::Cone => {
                        self.scene_tool = SceneTool::Cone;
                        self.ribbon_active_action = None;
                    }
                    RibbonAction::Torus => {
                        self.scene_tool = SceneTool::Torus;
                        self.ribbon_active_action = None;
                    }
                    RibbonAction::TopView => {
                        self.scene_camera_preset = Some(CameraPreset::Top);
                    }
                    RibbonAction::Isometric => {
                        self.scene_camera_preset = Some(CameraPreset::Isometric);
                    }
                    _ => {}
                }
                self.invalidate();
            }
            Message::Quit => {
                // TODO: request app quit (platform dependent)
            }
            Message::Canvas(event) => {
                match event {
                    CanvasEvent::ZoomAt { delta, cursor } => {
                        if delta.abs() > f32::EPSILON {
                            let factor = if delta > 0.0 { 1.1 } else { 0.9 };
                            let next = (self.zoom * factor).clamp(0.25, 4.0);
                            if (next - self.zoom).abs() > f32::EPSILON {
                                let world_x = (cursor.x - self.view_offset.x) / self.zoom;
                                let world_y = (cursor.y - self.view_offset.y) / self.zoom;

                                self.zoom = next;
                                self.view_offset = iced::Point {
                                    x: cursor.x - world_x * self.zoom,
                                    y: cursor.y - world_y * self.zoom,
                                };
                                self.invalidate();
                            }
                        }
                    }
                    _ => self.handle_canvas_event(event),
                }
            }
        }

        Task::none()
    }

    fn handle_canvas_event(&mut self, event: CanvasEvent) {
        match event {
            CanvasEvent::PressedLeft(p) => match self.tool {
                Tool::Select => {
                    self.cursor_pos = Some(p);
                    self.selected = None;
                    self.dragging = false;
                    self.drag_last = None;
                    for (i, shape) in self.shapes.iter().enumerate().rev() {
                        if shape.hit_test(p) {
                            self.selected = Some(i);
                            self.dragging = true;
                            self.drag_last = Some(p);
                            break;
                        }
                    }
                    self.invalidate();
                }
                Tool::Line | Tool::Rect | Tool::Circle => {
                    self.cursor_pos = Some(p);
                    self.selected = None;
                    self.draft.drawing = true;
                    self.draft.start = Some(p);
                    self.draft.current = Some(p);
                    self.invalidate();
                }
                Tool::Spline => {
                    self.cursor_pos = Some(p);
                    self.selected = None;
                    self.draft.current = Some(p);
                    self.draft.spline_points.push(p);
                    self.invalidate();
                }
                Tool::Polygon => {
                    self.cursor_pos = Some(p);
                    self.selected = None;
                    self.draft.current = Some(p);
                    self.draft.polygon_points.push(p);
                    self.invalidate();
                }
            },
            CanvasEvent::Moved(p) => {
                self.cursor_pos = Some(p);
                self.draft.current = Some(p);
                if self.tool == Tool::Select && self.dragging {
                    if let (Some(index), Some(last)) = (self.selected, self.drag_last) {
                        let dx = p.x - last.x;
                        let dy = p.y - last.y;
                        if let Some(shape) = self.shapes.get_mut(index) {
                            shape.translate(dx, dy);
                        }
                        self.drag_last = Some(p);
                    }
                }
                self.invalidate();
            }
            CanvasEvent::ReleasedLeft(p) => match self.tool {
                Tool::Select => {
                    self.cursor_pos = Some(p);
                    self.dragging = false;
                    self.drag_last = None;
                }
                Tool::Line | Tool::Rect | Tool::Circle => {
                    self.cursor_pos = Some(p);
                    if !self.draft.drawing {
                        return;
                    }

                    let Some(start) = self.draft.start else {
                        self.reset_draft();
                        return;
                    };

                    if let Some(shape) = finish_drag_shape(
                        self.tool,
                        start,
                        p,
                        self.stroke_width,
                        self.stroke_color,
                        self.fill_color,
                    ) {
                        self.commit_new_shape(shape);
                    }

                    self.reset_draft();
                    self.invalidate();
                }
                Tool::Spline | Tool::Polygon => {
                    self.cursor_pos = Some(p);
                }
            },
            CanvasEvent::PressedRight(_p) => match self.tool {
                Tool::Spline => {
                    if self.draft.spline_points.len() >= 2 {
                        let points = std::mem::take(&mut self.draft.spline_points);
                        self.shapes.push(Shape::Spline {
                            points,
                            stroke_color: self.stroke_color,
                            stroke_width: self.stroke_width,
                        });
                    }

                    self.reset_draft();
                    self.invalidate();
                }
                Tool::Polygon => {
                    if self.draft.polygon_points.len() >= 3 {
                        let points = std::mem::take(&mut self.draft.polygon_points);
                        self.commit_new_shape(Shape::Polygon {
                            outer: points,
                            holes: Vec::new(),
                            stroke_color: self.stroke_color,
                            fill_color: self.fill_color,
                            stroke_width: self.stroke_width,
                        });
                    }

                    self.reset_draft();
                    self.invalidate();
                }
                _ => {}
            },
            CanvasEvent::ZoomAt { .. } => {
                // Zoom handled in update
            }
        }
    }

    fn commit_new_shape(&mut self, shape: Shape) {
        match self.mode {
            BooleanMode::Add => {
                self.shapes.push(shape);
                self.invalidate();
            }
            BooleanMode::Merge | BooleanMode::Mask | BooleanMode::Diff => {
                if !Self::shape_is_region(&shape) {
                    self.shapes.push(shape);
                    self.invalidate();
                    return;
                }

                let stroke_color = self.stroke_color;
                let fill_color = self.fill_color;
                let stroke_width = self.stroke_width;

                let new_contours = Self::shape_to_int_contours(&shape);
                if new_contours.is_empty() {
                    return;
                }

                let mut non_region = Vec::new();
                let mut existing_region_contours: Vec<Vec<IntPoint>> = Vec::new();
                for s in self.shapes.drain(..) {
                    if Self::shape_is_region(&s) {
                        existing_region_contours.extend(Self::shape_to_int_contours(&s));
                    } else {
                        non_region.push(s);
                    }
                }

                let rule = match self.mode {
                    BooleanMode::Merge => OverlayRule::Union,
                    BooleanMode::Mask => OverlayRule::Intersect,
                    BooleanMode::Diff => OverlayRule::Difference,
                    BooleanMode::Add => unreachable!(),
                };

                let result = if existing_region_contours.is_empty() {
                    match self.mode {
                        BooleanMode::Merge => Self::int_shapes_to_shapes(
                            vec![new_contours.clone()],
                            stroke_color,
                            fill_color,
                            stroke_width,
                        ),
                        BooleanMode::Mask | BooleanMode::Diff => Vec::new(),
                        BooleanMode::Add => unreachable!(),
                    }
                } else {
                    let capacity = existing_region_contours
                        .iter()
                        .map(|c| c.len())
                        .sum::<usize>()
                        + new_contours.iter().map(|c| c.len()).sum::<usize>();
                    let mut overlay = Overlay::new(capacity);
                    for c in &existing_region_contours {
                        overlay.add_contour(c, i_overlay::core::overlay::ShapeType::Subject);
                    }
                    for c in &new_contours {
                        overlay.add_contour(c, i_overlay::core::overlay::ShapeType::Clip);
                    }

                    let int_shapes = overlay.overlay(rule, FillRule::EvenOdd);
                    Self::int_shapes_to_shapes(int_shapes, stroke_color, fill_color, stroke_width)
                };

                self.shapes = non_region;
                self.shapes.extend(result);
                self.selected = None;
                self.dragging = false;
                self.drag_last = None;
                self.invalidate();
            }
        }
    }

    const BOOL_SCALE: f32 = 1000.0;

    fn shape_is_region(shape: &Shape) -> bool {
        matches!(shape, Shape::Rect { .. } | Shape::Circle { .. } | Shape::Polygon { .. })
    }

    fn p_to_int(p: iced::Point) -> IntPoint {
        let x = (p.x * Self::BOOL_SCALE).round();
        let y = (p.y * Self::BOOL_SCALE).round();
        IntPoint::new(
            x.clamp(i32::MIN as f32, i32::MAX as f32) as i32,
            y.clamp(i32::MIN as f32, i32::MAX as f32) as i32,
        )
    }

    fn int_to_p(p: IntPoint) -> iced::Point {
        iced::Point {
            x: (p.x as f32) / Self::BOOL_SCALE,
            y: (p.y as f32) / Self::BOOL_SCALE,
        }
    }

    fn shape_to_int_contours(shape: &Shape) -> Vec<Vec<IntPoint>> {
        match shape {
            Shape::Rect { from, to, .. } => {
                let x0 = from.x.min(to.x);
                let y0 = from.y.min(to.y);
                let x1 = from.x.max(to.x);
                let y1 = from.y.max(to.y);
                vec![vec![
                    Self::p_to_int(iced::Point { x: x0, y: y0 }),
                    Self::p_to_int(iced::Point { x: x1, y: y0 }),
                    Self::p_to_int(iced::Point { x: x1, y: y1 }),
                    Self::p_to_int(iced::Point { x: x0, y: y1 }),
                ]]
            }
            Shape::Circle { center, radius, .. } => {
                let steps = 64usize;
                let mut contour = Vec::with_capacity(steps);
                for i in 0..steps {
                    let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
                    contour.push(Self::p_to_int(iced::Point {
                        x: center.x + radius * t.cos(),
                        y: center.y + radius * t.sin(),
                    }));
                }
                vec![contour]
            }
            Shape::Polygon { outer, holes, .. } => {
                if outer.len() < 3 {
                    return Vec::new();
                }
                let mut out = Vec::new();
                out.push(outer.iter().map(|p| Self::p_to_int(*p)).collect());
                for hole in holes {
                    if hole.len() < 3 {
                        continue;
                    }
                    out.push(hole.iter().map(|p| Self::p_to_int(*p)).collect());
                }
                out
            }
            Shape::Line { .. } | Shape::Spline { .. } => Vec::new(),
        }
    }

    fn int_shapes_to_shapes(
        int_shapes: Vec<Vec<Vec<IntPoint>>>,
        stroke_color: Color,
        fill_color: Option<Color>,
        stroke_width: f32,
    ) -> Vec<Shape> {
        let mut out = Vec::new();
        for shape in int_shapes {
            if shape.is_empty() {
                continue;
            }
            let outer = shape[0]
                .iter()
                .copied()
                .map(Self::int_to_p)
                .collect::<Vec<_>>();
            if outer.len() < 3 {
                continue;
            }
            let holes = shape
                .iter()
                .skip(1)
                .map(|c| c.iter().copied().map(Self::int_to_p).collect::<Vec<_>>())
                .filter(|c| c.len() >= 3)
                .collect::<Vec<_>>();
            out.push(Shape::Polygon {
                outer,
                holes,
                stroke_color,
                fill_color,
                stroke_width,
            });
        }
        out
    }

    fn view(&self) -> Element<'_, Message> {
        let tab_bar = row![
            controls::tab_button("Render", Tab::Scene, self.active_tab),
            controls::tab_button("Draw", Tab::Draw, self.active_tab),
        ]
        .spacing(8)
        .align_y(Alignment::Center);

        let menu_bar: Option<Element<'_, Message>> = if self.show_menubar {
            Some(controls::build_menu_bar(self))
        } else {
            None
        };

        let toolbar = controls::draw_toolbar(
            self.tool,
            self.mode,
            self.stroke_width,
            self.show_grid,
            self.selected,
        );
        let stroke_row = controls::stroke_row();
        let fill_row = controls::fill_row();
        let board = canvas::canvas(
            self.tool,
            self.shapes.as_slice(),
            &self.draft,
            self.stroke_width,
            self.stroke_color,
            self.fill_color,
            self.show_grid,
            self.zoom,
            self.view_offset,
            self.cursor_pos,
            wrap_canvas_event,
        );

        let draw_tab = column![
            toolbar, stroke_row, fill_row, board]
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill);

        let scene_view = controls::scene_widget::<Message>(
            self.scene_show_grid,
            self.scene_grid_plane,
            self.scene_grid_extent,
            self.scene_grid_step,
            self.scene_tool,
            self.scene_camera_mode,
            self.scene_camera_preset,
            self.scene_axes_enabled,
            self.scene_axes_size,
            self.scene_axes_margin,
            self.scene_bg_color,
            self.scene_zoom_factor,
            self.scene_zoom_version,
            self.scene_unite_version,
            self.scene_request_select_id,
            self.scene_request_focus_id,
            |list| Message::SceneEntitiesSnapshot(list),
            Message::SceneToolCompleted,
        );

        let sidebar = controls::build_scene_sidebar(&self.scene_entities);
        let scene_tab = row![sidebar, column![scene_view].spacing(10)]
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill);

        let content: Element<Message> = match self.active_tab {
            Tab::Draw => draw_tab.into(),
            Tab::Scene => scene_tab.into(),
        };

        let scene_ribbon: Option<Element<Message>> = match self.active_tab {
            Tab::Scene => Some(
                column![
                    ribbon(
                        self.ribbon_tab,
                        self.ribbon_active_action,
                        Message::RibbonTabChanged,
                        Message::RibbonAction,
                    ),
                    controls::scene_ribbon_controls(
                        self.ribbon_tab,
                        self.scene_show_grid,
                        self.scene_grid_plane,
                        self.scene_tool,
                        self.scene_camera_mode,
                        self.scene_camera_preset,
                        self.scene_axes_enabled,
                        self.scene_axes_size,
                        self.scene_axes_margin,
                        self.scene_grid_extent,
                        self.scene_grid_step,
                        self.scene_bg_color,
                        self.scene_bg_picker_open,
                    ),
                ]
                .spacing(8)
                .into(),
            ),
            _ => None,
        };

        let status_bar: Element<Message> = match self.active_tab {
            Tab::Draw => controls::status_row(self.tool, self.zoom, self.cursor_pos),
            Tab::Scene => controls::scene_status_row(
                self.scene_tool,
                self.scene_camera_mode,
                self.scene_camera_preset,
                self.scene_show_grid,
                self.gmsh_status.as_deref(),
                self.scene_grid_plane,
            ),
        };

        let mut top = column![menu_bar, scene_ribbon, tab_bar];
        top = top.push(content);
        top = top.push(container(status_bar).padding(8).width(Length::Fill));
        top
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

// UI widget helpers are now in `controls.rs`

fn finish_drag_shape(
    tool: Tool,
    start: iced::Point,
    end: iced::Point,
    stroke_width: f32,
    stroke_color: Color,
    fill_color: Option<Color>,
) -> Option<Shape> {
    match tool {
        Tool::Select => None,
        Tool::Line => Some(Shape::Line {
            from: start,
            to: end,
            stroke_color,
            stroke_width,
        }),
        Tool::Rect => Some(Shape::Rect {
            from: start,
            to: end,
            stroke_color,
            fill_color,
            stroke_width,
        }),
        Tool::Circle => {
            let radius = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
            Some(Shape::Circle {
                center: start,
                radius,
                stroke_color,
                fill_color,
                stroke_width,
            })
        }
        Tool::Spline | Tool::Polygon => None,
    }
}

fn apply_stroke_color(shape: &mut Shape, color: Color) {
    match shape {
        Shape::Line { stroke_color, .. } => *stroke_color = color,
        Shape::Rect { stroke_color, .. } => *stroke_color = color,
        Shape::Circle { stroke_color, .. } => *stroke_color = color,
        Shape::Spline { stroke_color, .. } => *stroke_color = color,
        Shape::Polygon { stroke_color, .. } => *stroke_color = color,
    }
}

fn apply_fill_color(shape: &mut Shape, color: Option<Color>) {
    match shape {
        Shape::Rect { fill_color, .. } => *fill_color = color,
        Shape::Circle { fill_color, .. } => *fill_color = color,
        Shape::Polygon { fill_color, .. } => *fill_color = color,
        Shape::Line { .. } | Shape::Spline { .. } => {}
    }
}

fn apply_stroke_width(shape: &mut Shape, width: f32) {
    match shape {
        Shape::Line { stroke_width, .. } => *stroke_width = width,
        Shape::Rect { stroke_width, .. } => *stroke_width = width,
        Shape::Circle { stroke_width, .. } => *stroke_width = width,
        Shape::Spline { stroke_width, .. } => *stroke_width = width,
        Shape::Polygon { stroke_width, .. } => *stroke_width = width,
    }
}

fn main() -> iced::Result {
    iced::application(App::new, App::update, App::view)
        .title("2D Vector Drawing Tool")
        .window_size(Size::new(1200.0, 800.0))
        .run()
}
