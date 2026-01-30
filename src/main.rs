mod drawing;
mod scene;
mod canvas;
mod cad;

use drawing::{CanvasEvent, Draft, Shape, Tool};
use iced::widget::{button, column, pick_list, row, slider, text};
use iced::{Alignment, Color, Element, Length, Size, Task};

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
    About,
}

#[derive(Debug, Clone)]
enum Message {
    TabChanged(Tab),
    ToolChanged(Tool),
    ModeChanged(BooleanMode),
    StrokeWidthChanged(f32),
    StrokeColorChanged(Color),
    FillColorChanged(Option<Color>),
    ToggleGrid,
    ResetZoom,
    Undo,
    DeleteSelected,
    Clear,
    Canvas(CanvasEvent),
}

fn wrap_canvas_event(e: CanvasEvent) -> Message {
    Message::Canvas(e)
}

#[derive(Debug)]
struct App {
    active_tab: Tab,
    tool: Tool,
    mode: BooleanMode,
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
}

impl App {
    fn new() -> Self {
        Self {
            active_tab: Tab::Draw,
            tool: Tool::Line,
            mode: BooleanMode::Add,
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
            CanvasEvent::PressedRight(_p) => {
                match self.tool {
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
                }
            }
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
            tab_button("画图", Tab::Draw, self.active_tab),
            tab_button("渲染", Tab::Scene, self.active_tab),
            tab_button("说明", Tab::About, self.active_tab),
        ]
        .spacing(8)
        .align_y(Alignment::Center);

        let toolbar = || {
            row![
                text("工具"),
                pick_list(Tool::ALL.as_slice(), Some(self.tool), Message::ToolChanged)
                    .width(Length::Fixed(120.0)),
                text("模式"),
                pick_list(BooleanMode::ALL.as_slice(), Some(self.mode), Message::ModeChanged)
                    .width(Length::Fixed(120.0)),
                text("线宽"),
                slider(1.0..=16.0, self.stroke_width, Message::StrokeWidthChanged)
                    .width(Length::Fixed(180.0)),
                text(format!("{:.1}", self.stroke_width)),
                button(if self.show_grid { "网格：开" } else { "网格：关" })
                    .on_press(Message::ToggleGrid),
                button("缩放重置").on_press(Message::ResetZoom),
                button("撤销").on_press(Message::Undo),
                button("删除选中")
                    .on_press_maybe(self.selected.map(|_| Message::DeleteSelected)),
                button("清空").on_press(Message::Clear),
            ]
            .spacing(10)
            .align_y(Alignment::Center)
        };

        let stroke_row = || {
            row![
                text("描边"),
                button("黑").on_press(Message::StrokeColorChanged(Color::BLACK)),
                button("红")
                    .on_press(Message::StrokeColorChanged(Color::from_rgb8(0xE6, 0x2E, 0x2E))),
                button("绿")
                    .on_press(Message::StrokeColorChanged(Color::from_rgb8(0x2E, 0xE6, 0x6B))),
                button("蓝")
                    .on_press(Message::StrokeColorChanged(Color::from_rgb8(0x2E, 0xB8, 0xE6))),
            ]
            .spacing(8)
            .align_y(Alignment::Center)
        };

        let fill_row = || {
            row![
                text("填充"),
                button("无").on_press(Message::FillColorChanged(None)),
                button("浅蓝")
                    .on_press(Message::FillColorChanged(Some(Color::from_rgb8(0xB3, 0xD9, 0xFF)))),
                button("浅红")
                    .on_press(Message::FillColorChanged(Some(Color::from_rgb8(0xFF, 0xB3, 0xB3)))),
            ]
            .spacing(8)
            .align_y(Alignment::Center)
        };

        let hint = match self.tool {
            Tool::Select => "选择：左键点击图形；可删除选中",
            Tool::Spline => "样条：左键加点，右键结束",
            Tool::Polygon => "多边形：左键加点，右键闭合结束",
            _ => "拖拽绘制：按下-移动-松开",
        };

        let status = || {
            row![
                text(format!("缩放：{:.0}%", self.zoom * 100.0)),
                text("|"),
                text(match self.cursor_pos {
                    Some(p) => format!("坐标：{:.1}, {:.1}", p.x, p.y),
                    None => "坐标：--".to_string(),
                }),
                text("|"),
                text(hint),
            ]
            .spacing(10)
            .align_y(Alignment::Center)
        };

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

        let draw_tab = column![toolbar(), stroke_row(), fill_row(), board, status()]
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill);

        let about_tab = column![
            text("操作提示"),
            text("- 选择：左键点击图形，可拖拽移动"),
            text("- 线/矩形/圆：拖拽绘制"),
            text("- 样条：左键加点，右键结束"),
            text("- 多边形：左键加点，右键结束"),
            text("- 滚轮缩放，按钮可重置"),
        ]
        .spacing(6)
        .padding(12)
        .width(Length::Fill)
        .height(Length::Fill);

        let scene_tab = column![scene::widget::<Message>()]
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill);

        let content = match self.active_tab {
            Tab::Draw => draw_tab,
            Tab::Scene => scene_tab,
            Tab::About => about_tab,
        };

        column![tab_bar, content]
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
    }
}

fn tab_button(label: &str, tab: Tab, active: Tab) -> iced::widget::Button<'_, Message> {
    if tab == active {
        button(text(format!("● {label}"))).on_press_maybe(None)
    } else {
        button(text(label)).on_press(Message::TabChanged(tab))
    }
}

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
