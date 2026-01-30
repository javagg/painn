mod drawing;

use drawing::{CanvasEvent, Draft, Shape, Tool};
use iced::widget::{button, column, pick_list, row, slider, text};
use iced::{Alignment, Color, Element, Length, Size, Task};

#[derive(Debug, Clone)]
enum Message {
    ToolChanged(Tool),
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
    tool: Tool,
    stroke_width: f32,
    stroke_color: Color,
    fill_color: Option<Color>,
    show_grid: bool,
    zoom: f32,
    view_offset: iced::Point,
    shapes: Vec<Shape>,
    selected: Option<usize>,
    draft: Draft,
    cache: iced::widget::canvas::Cache,
}

impl App {
    fn new() -> Self {
        Self {
            tool: Tool::Line,
            stroke_width: 3.0,
            stroke_color: Color::from_rgb8(0xE6, 0xE6, 0xE6),
            fill_color: None,
            show_grid: true,
            zoom: 1.0,
            view_offset: iced::Point::ORIGIN,
            shapes: Vec::new(),
            selected: None,
            draft: Draft::default(),
            cache: iced::widget::canvas::Cache::default(),
        }
    }

    fn invalidate(&mut self) {
        self.cache.clear();
    }

    fn reset_draft(&mut self) {
        self.draft = Draft::default();
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::ToolChanged(tool) => {
                self.tool = tool;
                self.reset_draft();
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
                self.reset_draft();
                self.invalidate();
            }
            Message::DeleteSelected => {
                if let Some(index) = self.selected {
                    self.shapes.remove(index);
                    self.selected = None;
                    self.invalidate();
                }
            }
            Message::Clear => {
                self.shapes.clear();
                self.selected = None;
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
                    self.selected = None;
                    for (i, shape) in self.shapes.iter().enumerate().rev() {
                        if shape.hit_test(p) {
                            self.selected = Some(i);
                            break;
                        }
                    }
                    self.invalidate();
                }
                Tool::Line | Tool::Rect | Tool::Circle => {
                    self.selected = None;
                    self.draft.drawing = true;
                    self.draft.start = Some(p);
                    self.draft.current = Some(p);
                    self.invalidate();
                }
                Tool::Spline => {
                    self.selected = None;
                    self.draft.current = Some(p);
                    self.draft.spline_points.push(p);
                    self.invalidate();
                }
            },
            CanvasEvent::Moved(p) => {
                self.draft.current = Some(p);
                if self.draft.drawing || self.tool == Tool::Spline {
                    self.invalidate();
                }
            }
            CanvasEvent::ReleasedLeft(p) => match self.tool {
                Tool::Select => {}
                Tool::Line | Tool::Rect | Tool::Circle => {
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
                        self.shapes.push(shape);
                    }

                    self.reset_draft();
                    self.invalidate();
                }
                Tool::Spline => {
                    let _ = p;
                }
            },
            CanvasEvent::PressedRight(_p) => {
                if self.tool != Tool::Spline {
                    return;
                }

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
            CanvasEvent::ZoomAt { .. } => {
                // Zoom handled in update
            }
        }
    }

    fn view(&self) -> Element<'_, Message> {
        let toolbar = row![
            text("工具"),
            pick_list(Tool::ALL.as_slice(), Some(self.tool), Message::ToolChanged)
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
        .align_y(Alignment::Center);

        let stroke_row = row![
            text("描边"),
            button("黑").on_press(Message::StrokeColorChanged(Color::BLACK)),
            button("红").on_press(Message::StrokeColorChanged(Color::from_rgb8(0xE6, 0x2E, 0x2E))),
            button("绿").on_press(Message::StrokeColorChanged(Color::from_rgb8(0x2E, 0xE6, 0x6B))),
            button("蓝").on_press(Message::StrokeColorChanged(Color::from_rgb8(0x2E, 0xB8, 0xE6))),
        ]
        .spacing(8)
        .align_y(Alignment::Center);

        let fill_row = row![
            text("填充"),
            button("无")
                .on_press(Message::FillColorChanged(None)),
            button("浅蓝")
                .on_press(Message::FillColorChanged(Some(Color::from_rgb8(0xB3, 0xD9, 0xFF)))),
            button("浅红")
                .on_press(Message::FillColorChanged(Some(Color::from_rgb8(0xFF, 0xB3, 0xB3)))),
        ]
        .spacing(8)
        .align_y(Alignment::Center);

        let hint = match self.tool {
            Tool::Select => "选择：左键点击图形；可删除选中",
            Tool::Spline => "样条：左键加点，右键结束",
            _ => "拖拽绘制：按下-移动-松开",
        };

        let status = row![
            text(format!("缩放：{:.0}%", self.zoom * 100.0)),
            text("|"),
            text(hint),
        ]
        .spacing(10)
        .align_y(Alignment::Center);

        let board = drawing::canvas(
            &self.cache,
            self.tool,
            self.shapes.as_slice(),
            &self.draft,
            self.stroke_width,
            self.stroke_color,
            self.fill_color,
            self.show_grid,
            self.zoom,
            self.view_offset,
            wrap_canvas_event,
        );

        column![toolbar, stroke_row, fill_row, board, status]
            .spacing(10)
            .padding(12)
            .width(Length::Fill)
            .height(Length::Fill)
            .into()
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
        Tool::Spline => None,
    }
}

fn apply_stroke_color(shape: &mut Shape, color: Color) {
    match shape {
        Shape::Line { stroke_color, .. } => *stroke_color = color,
        Shape::Rect { stroke_color, .. } => *stroke_color = color,
        Shape::Circle { stroke_color, .. } => *stroke_color = color,
        Shape::Spline { stroke_color, .. } => *stroke_color = color,
    }
}

fn apply_fill_color(shape: &mut Shape, color: Option<Color>) {
    match shape {
        Shape::Rect { fill_color, .. } => *fill_color = color,
        Shape::Circle { fill_color, .. } => *fill_color = color,
        Shape::Line { .. } | Shape::Spline { .. } => {}
    }
}

fn apply_stroke_width(shape: &mut Shape, width: f32) {
    match shape {
        Shape::Line { stroke_width, .. } => *stroke_width = width,
        Shape::Rect { stroke_width, .. } => *stroke_width = width,
        Shape::Circle { stroke_width, .. } => *stroke_width = width,
        Shape::Spline { stroke_width, .. } => *stroke_width = width,
    }
}

fn main() -> iced::Result {
    iced::application(App::new, App::update, App::view)
        .title("2D 矢量绘图")
        .window_size(Size::new(1200.0, 800.0))
        .run()
}
