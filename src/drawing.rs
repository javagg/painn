#![cfg_attr(target_arch = "wasm32", allow(dead_code))]
#![allow(dead_code)]

use iced::mouse;
use iced::widget::canvas::{self, Cache, Canvas, Frame, Geometry, Path, Program, Stroke};
use iced::{Color, Element, Length, Point, Rectangle, Size, Theme};
use iced_widget::Action;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Select,
    Line,
    Rect,
    Circle,
    Spline,
    Polygon,
}

impl Tool {
    pub const ALL: [Tool; 6] = [
        Tool::Select,
        Tool::Line,
        Tool::Rect,
        Tool::Circle,
        Tool::Spline,
        Tool::Polygon,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Tool::Select => "Select",
            Tool::Line => "Line",
            Tool::Rect => "Rectangle",
            Tool::Circle => "Circle",
            Tool::Spline => "Spline",
            Tool::Polygon => "Polygon",
        }
    }
}

impl std::fmt::Display for Tool {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone)]
pub enum Shape {
    Line {
        from: Point,
        to: Point,
        stroke_color: Color,
        stroke_width: f32,
    },
    Rect {
        from: Point,
        to: Point,
        stroke_color: Color,
        fill_color: Option<Color>,
        stroke_width: f32,
    },
    Circle {
        center: Point,
        radius: f32,
        stroke_color: Color,
        fill_color: Option<Color>,
        stroke_width: f32,
    },
    /// A spline represented as control points; rendered as a Catmull–Rom sampled polyline.
    Spline {
        points: Vec<Point>,
        stroke_color: Color,
        stroke_width: f32,
    },
    Polygon {
        outer: Vec<Point>,
        holes: Vec<Vec<Point>>,
        stroke_color: Color,
        fill_color: Option<Color>,
        stroke_width: f32,
    },
}

impl Shape {
    pub fn hit_test(&self, point: Point) -> bool {
        match self {
            Shape::Line {
                from,
                to,
                stroke_width,
                ..
            } => distance_to_segment(point, *from, *to) <= (*stroke_width * 2.0),
            Shape::Rect {
                from,
                to,
                stroke_width,
                ..
            } => {
                let (pos, size) = rect_from_points(*from, *to);
                let pad = *stroke_width;
                point.x >= pos.x - pad
                    && point.x <= pos.x + size.width + pad
                    && point.y >= pos.y - pad
                    && point.y <= pos.y + size.height + pad
            }
            Shape::Circle {
                center,
                radius,
                stroke_width,
                ..
            } => {
                let dx = point.x - center.x;
                let dy = point.y - center.y;
                let dist = (dx * dx + dy * dy).sqrt();
                dist <= (*radius + *stroke_width)
            }
            Shape::Spline {
                points,
                stroke_width,
                ..
            } => {
                let samples = catmull_rom_samples(points, 24);
                if samples.len() < 2 {
                    return false;
                }

                for window in samples.windows(2) {
                    if distance_to_segment(point, window[0], window[1]) <= (*stroke_width * 2.0)
                    {
                        return true;
                    }
                }

                false
            }
            Shape::Polygon {
                outer,
                holes,
                stroke_width,
                ..
            } => {
                if outer.len() < 3 {
                    return false;
                }

                if point_in_polygon_with_holes(point, outer, holes.as_slice()) {
                    return true;
                }

                hit_test_polygon_outline(point, outer, holes.as_slice(), *stroke_width * 2.0)
            }
        }
    }

    pub fn translate(&mut self, dx: f32, dy: f32) {
        match self {
            Shape::Line { from, to, .. } => {
                from.x += dx;
                from.y += dy;
                to.x += dx;
                to.y += dy;
            }
            Shape::Rect { from, to, .. } => {
                from.x += dx;
                from.y += dy;
                to.x += dx;
                to.y += dy;
            }
            Shape::Circle { center, .. } => {
                center.x += dx;
                center.y += dy;
            }
            Shape::Spline { points, .. } => {
                for p in points.iter_mut() {
                    p.x += dx;
                    p.y += dy;
                }
            }
            Shape::Polygon { outer, holes, .. } => {
                for p in outer.iter_mut() {
                    p.x += dx;
                    p.y += dy;
                }
                for hole in holes.iter_mut() {
                    for p in hole.iter_mut() {
                        p.x += dx;
                        p.y += dy;
                    }
                }
            }
        }
    }
}

fn point_in_polygon_with_holes(point: Point, outer: &[Point], holes: &[Vec<Point>]) -> bool {
    if !point_in_polygon(point, outer) {
        return false;
    }

    for hole in holes {
        if hole.len() >= 3 && point_in_polygon(point, hole) {
            return false;
        }
    }

    true
}

fn hit_test_polygon_outline(
    point: Point,
    outer: &[Point],
    holes: &[Vec<Point>],
    tolerance: f32,
) -> bool {
    if hit_test_polyline_closed(point, outer, tolerance) {
        return true;
    }

    for hole in holes {
        if hit_test_polyline_closed(point, hole, tolerance) {
            return true;
        }
    }

    false
}

fn hit_test_polyline_closed(point: Point, points: &[Point], tolerance: f32) -> bool {
    if points.len() < 2 {
        return false;
    }

    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        if distance_to_segment(point, a, b) <= tolerance {
            return true;
        }
    }

    false
}

#[derive(Debug, Clone)]
pub enum CanvasEvent {
    PressedLeft(Point),
    ReleasedLeft(Point),
    Moved(Point),
    PressedRight(Point),
    ZoomAt { delta: f32, cursor: Point },
}

#[derive(Debug, Default, Clone)]
pub struct Draft {
    pub drawing: bool,
    pub start: Option<Point>,
    pub current: Option<Point>,
    pub spline_points: Vec<Point>,
    pub polygon_points: Vec<Point>,
}

pub fn canvas<'a, Message>(
    cache: &'a Cache,
    tool: Tool,
    shapes: &'a [Shape],
    draft: &'a Draft,
    stroke_width: f32,
    stroke_color: Color,
    fill_color: Option<Color>,
    show_grid: bool,
    zoom: f32,
    view_offset: Point,
    cursor_pos: Option<Point>,
    on_event: fn(CanvasEvent) -> Message,
) -> Element<'a, Message>
where
    Message: 'a,
{
    Canvas::new(Board {
        cache,
        tool,
        shapes,
        draft,
        stroke_width,
        stroke_color,
        fill_color,
        show_grid,
        zoom,
        view_offset,
        cursor_pos,
        on_event,
    })
    .width(Length::Fill)
    .height(Length::Fill)
    .into()
}

struct Board<'a, Message> {
    cache: &'a Cache,
    tool: Tool,
    shapes: &'a [Shape],
    draft: &'a Draft,
    stroke_width: f32,
    stroke_color: Color,
    fill_color: Option<Color>,
    show_grid: bool,
    zoom: f32,
    view_offset: Point,
    cursor_pos: Option<Point>,
    on_event: fn(CanvasEvent) -> Message,
}

impl<Message> Program<Message> for Board<'_, Message> {
    type State = ();

    fn update(
        &self,
        _state: &mut Self::State,
        event: &canvas::Event,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> Option<Action<Message>> {
        use iced::mouse::{Button, Event as MouseEvent};

        let Some(pos) = cursor.position_in(bounds) else {
            return None;
        };

        let world_pos = Point {
            x: (pos.x - self.view_offset.x) / self.zoom,
            y: (pos.y - self.view_offset.y) / self.zoom,
        };

        let message = match event {
            canvas::Event::Mouse(MouseEvent::ButtonPressed(Button::Left)) => {
                (self.on_event)(CanvasEvent::PressedLeft(world_pos))
            }
            canvas::Event::Mouse(MouseEvent::ButtonReleased(Button::Left)) => {
                (self.on_event)(CanvasEvent::ReleasedLeft(world_pos))
            }
            canvas::Event::Mouse(MouseEvent::ButtonPressed(Button::Right)) => {
                (self.on_event)(CanvasEvent::PressedRight(world_pos))
            }
            canvas::Event::Mouse(MouseEvent::CursorMoved { .. }) => {
                (self.on_event)(CanvasEvent::Moved(world_pos))
            }
            canvas::Event::Mouse(MouseEvent::WheelScrolled { delta }) => {
                let amount = match delta {
                    mouse::ScrollDelta::Lines { y, .. } => *y,
                    mouse::ScrollDelta::Pixels { y, .. } => *y,
                };
                (self.on_event)(CanvasEvent::ZoomAt { delta: amount, cursor: pos })
            }
            _ => return None,
        };

        Some(Action::publish(message).and_capture())
    }

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &iced::Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let preview_stroke_color = Color::from_rgb8(0xE6, 0xB8, 0x2E);

        let geo = self.cache.draw(renderer, bounds.size(), |frame: &mut Frame| {
            frame.fill_rectangle(Point::ORIGIN, frame.size(), Color::from_rgb8(0x12, 0x12, 0x14));

            frame.translate(iced::Vector {
                x: self.view_offset.x,
                y: self.view_offset.y,
            });
            frame.scale(self.zoom);

            if self.show_grid {
                draw_grid(frame);
            }

            for shape in self.shapes {
                draw_shape(frame, shape);
            }

            if let Some(preview) = preview_shape(
                self.tool,
                self.draft,
                self.stroke_width,
                self.stroke_color,
                self.fill_color,
                preview_stroke_color,
            ) {
                draw_shape(frame, &preview);
            }

            if self.tool == Tool::Spline && !self.draft.spline_points.is_empty() {
                for p in &self.draft.spline_points {
                    let r = 3.5;
                    let rect = Rectangle {
                        x: p.x - r,
                        y: p.y - r,
                        width: 2.0 * r,
                        height: 2.0 * r,
                    };
                    frame.fill_rectangle(rect.position(), rect.size(), Color::from_rgb8(0x2E, 0xB8, 0xE6));
                }
            }

            if let Some(cursor) = self.cursor_pos {
                draw_crosshair(frame, self.zoom, self.view_offset, cursor);
            }
        });

        vec![geo]
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            if self.tool == Tool::Select {
                mouse::Interaction::Pointer
            } else {
                mouse::Interaction::Crosshair
            }
        } else {
            mouse::Interaction::default()
        }
    }
}

fn draw_shape(frame: &mut Frame, shape: &Shape) {
    match shape {
        Shape::Line {
            from,
            to,
            stroke_color,
            stroke_width,
        } => {
            let stroke = Stroke {
                width: *stroke_width,
                style: canvas::Style::Solid(*stroke_color),
                ..Stroke::default()
            };
            frame.stroke(&Path::line(*from, *to), stroke);
        }
        Shape::Rect {
            from,
            to,
            stroke_color,
            fill_color,
            stroke_width,
        } => {
            let (pos, size) = rect_from_points(*from, *to);
            let path = Path::rectangle(pos, size);

            if let Some(fill) = fill_color {
                frame.fill(&path, *fill);
            }

            let stroke = Stroke {
                width: *stroke_width,
                style: canvas::Style::Solid(*stroke_color),
                ..Stroke::default()
            };
            frame.stroke(&path, stroke);
        }
        Shape::Circle {
            center,
            radius,
            stroke_color,
            fill_color,
            stroke_width,
        } => {
            let path = Path::circle(*center, *radius);

            if let Some(fill) = fill_color {
                frame.fill(&path, *fill);
            }

            let stroke = Stroke {
                width: *stroke_width,
                style: canvas::Style::Solid(*stroke_color),
                ..Stroke::default()
            };
            frame.stroke(&path, stroke);
        }
        Shape::Spline {
            points,
            stroke_color,
            stroke_width,
        } => {
            let stroke = Stroke {
                width: *stroke_width,
                style: canvas::Style::Solid(*stroke_color),
                ..Stroke::default()
            };
            let samples = catmull_rom_samples(points, 24);
            if samples.len() >= 2 {
                let path = Path::new(|b| {
                    b.move_to(samples[0]);
                    for p in &samples[1..] {
                        b.line_to(*p);
                    }
                });
                frame.stroke(&path, stroke);
            }
        }
        Shape::Polygon {
            outer,
            holes,
            stroke_color,
            fill_color,
            stroke_width,
        } => {
            if outer.len() < 3 {
                return;
            }

            let path = Path::new(|b| {
                b.move_to(outer[0]);
                for p in outer.iter().skip(1) {
                    b.line_to(*p);
                }
                b.close();

                for hole in holes {
                    if hole.len() < 3 {
                        continue;
                    }
                    b.move_to(hole[0]);
                    for p in hole.iter().skip(1) {
                        b.line_to(*p);
                    }
                    b.close();
                }
            });

            if let Some(fill) = fill_color {
                frame.fill(&path, *fill);
            }

            let stroke = Stroke {
                width: *stroke_width,
                style: canvas::Style::Solid(*stroke_color),
                ..Stroke::default()
            };
            frame.stroke(&path, stroke);
        }
    }
}

fn draw_grid(frame: &mut Frame) {
    let size = frame.size();
    let spacing = 24.0;
    let minor = Color::from_rgba8(0xFF, 0xFF, 0xFF, 0.06);
    let major = Color::from_rgba8(0xFF, 0xFF, 0xFF, 0.12);

    let mut x = 0.0;
    let mut index = 0usize;
    while x <= size.width {
        let color = if index % 5 == 0 { major } else { minor };
        let stroke = Stroke {
            width: 1.0,
            style: canvas::Style::Solid(color),
            ..Stroke::default()
        };
        frame.stroke(&Path::line(Point { x, y: 0.0 }, Point { x, y: size.height }), stroke);
        x += spacing;
        index += 1;
    }

    let mut y = 0.0;
    let mut index = 0usize;
    while y <= size.height {
        let color = if index % 5 == 0 { major } else { minor };
        let stroke = Stroke {
            width: 1.0,
            style: canvas::Style::Solid(color),
            ..Stroke::default()
        };
        frame.stroke(&Path::line(Point { x: 0.0, y }, Point { x: size.width, y }), stroke);
        y += spacing;
        index += 1;
    }
}

fn draw_crosshair(frame: &mut Frame, zoom: f32, view_offset: Point, cursor: Point) {
    let size = frame.size();
    let world_min = Point {
        x: (-view_offset.x) / zoom,
        y: (-view_offset.y) / zoom,
    };
    let world_max = Point {
        x: (size.width - view_offset.x) / zoom,
        y: (size.height - view_offset.y) / zoom,
    };

    let stroke = Stroke {
        width: 1.0,
        style: canvas::Style::Solid(Color::from_rgba8(0xFF, 0xFF, 0xFF, 0.35)),
        ..Stroke::default()
    };

    frame.stroke(
        &Path::line(Point { x: world_min.x, y: cursor.y }, Point { x: world_max.x, y: cursor.y }),
        stroke,
    );
    frame.stroke(
        &Path::line(Point { x: cursor.x, y: world_min.y }, Point { x: cursor.x, y: world_max.y }),
        stroke,
    );
}

fn preview_shape(
    tool: Tool,
    draft: &Draft,
    stroke_width: f32,
    _stroke_color: Color,
    fill_color: Option<Color>,
    preview_stroke_color: Color,
) -> Option<Shape> {
    match tool {
        Tool::Select => None,
        Tool::Line => {
            if !draft.drawing {
                return None;
            }
            Some(Shape::Line {
                from: draft.start?,
                to: draft.current?,
                stroke_color: preview_stroke_color,
                stroke_width,
            })
        }
        Tool::Rect => {
            if !draft.drawing {
                return None;
            }
            Some(Shape::Rect {
                from: draft.start?,
                to: draft.current?,
                stroke_color: preview_stroke_color,
                fill_color,
                stroke_width,
            })
        }
        Tool::Circle => {
            if !draft.drawing {
                return None;
            }
            let center = draft.start?;
            let edge = draft.current?;
            let radius = ((edge.x - center.x).powi(2) + (edge.y - center.y).powi(2)).sqrt();
            Some(Shape::Circle {
                center,
                radius,
                stroke_color: preview_stroke_color,
                fill_color,
                stroke_width,
            })
        }
        Tool::Spline => {
            if draft.spline_points.len() < 2 {
                return None;
            }

            // For preview, append the current cursor point as a temporary endpoint.
            let mut pts = draft.spline_points.clone();
            if let Some(current) = draft.current {
                if pts.last().copied() != Some(current) {
                    pts.push(current);
                }
            }

            if pts.len() >= 2 {
                Some(Shape::Spline {
                    points: pts,
                    stroke_color: preview_stroke_color,
                    stroke_width,
                })
            } else {
                None
            }
        }
        Tool::Polygon => {
            if draft.polygon_points.len() < 2 {
                return None;
            }

            let mut pts = draft.polygon_points.clone();
            if let Some(current) = draft.current {
                if pts.last().copied() != Some(current) {
                    pts.push(current);
                }
            }

            if pts.len() < 3 {
                return Some(Shape::Line {
                    from: pts[0],
                    to: *pts.last().unwrap(),
                    stroke_color: preview_stroke_color,
                    stroke_width,
                });
            }

            Some(Shape::Polygon {
                outer: pts,
                holes: Vec::new(),
                stroke_color: preview_stroke_color,
                fill_color,
                stroke_width,
            })
        }
    }
}

fn rect_from_points(a: Point, b: Point) -> (Point, Size) {
    let x0 = a.x.min(b.x);
    let y0 = a.y.min(b.y);
    let x1 = a.x.max(b.x);
    let y1 = a.y.max(b.y);

    (
        Point { x: x0, y: y0 },
        Size {
            width: x1 - x0,
            height: y1 - y0,
        },
    )
}

fn distance_to_segment(p: Point, a: Point, b: Point) -> f32 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let apx = p.x - a.x;
    let apy = p.y - a.y;

    let ab_len2 = abx * abx + aby * aby;
    if ab_len2 <= f32::EPSILON {
        return (apx * apx + apy * apy).sqrt();
    }

    let t = (apx * abx + apy * aby) / ab_len2;
    let t = t.clamp(0.0, 1.0);
    let cx = a.x + t * abx;
    let cy = a.y + t * aby;
    let dx = p.x - cx;
    let dy = p.y - cy;
    (dx * dx + dy * dy).sqrt()
}

fn point_in_polygon(point: Point, points: &[Point]) -> bool {
    // Ray casting algorithm (even-odd rule)
    if points.len() < 3 {
        return false;
    }

    let mut inside = false;
    let mut j = points.len() - 1;
    for i in 0..points.len() {
        let pi = points[i];
        let pj = points[j];

        let intersect = ((pi.y > point.y) != (pj.y > point.y))
            && (point.x
                < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y + f32::EPSILON) + pi.x);

        if intersect {
            inside = !inside;
        }

        j = i;
    }

    inside
}


/// Catmull–Rom spline sampled to a polyline.
///
/// `subdiv` is the number of segments per span.
fn catmull_rom_samples(points: &[Point], subdiv: usize) -> Vec<Point> {
    if points.len() < 2 {
        return Vec::new();
    }

    let mut out = Vec::new();

    // Duplicate endpoints for boundary conditions.
    let get = |i: isize| -> Point {
        if i <= 0 {
            points[0]
        } else if (i as usize) >= points.len() {
            points[points.len() - 1]
        } else {
            points[i as usize]
        }
    };

    for i in 0..(points.len() - 1) {
        let p0 = get(i as isize - 1);
        let p1 = get(i as isize);
        let p2 = get(i as isize + 1);
        let p3 = get(i as isize + 2);

        for s in 0..subdiv {
            let t = s as f32 / subdiv as f32;
            out.push(catmull_rom(p0, p1, p2, p3, t));
        }
    }

    out.push(*points.last().unwrap());
    out
}

fn catmull_rom(p0: Point, p1: Point, p2: Point, p3: Point, t: f32) -> Point {
    // Standard Catmull–Rom with tension = 0.5
    let t2 = t * t;
    let t3 = t2 * t;

    let x = 0.5
        * ((2.0 * p1.x)
            + (-p0.x + p2.x) * t
            + (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2
            + (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);

    let y = 0.5
        * ((2.0 * p1.y)
            + (-p0.y + p2.y) * t
            + (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2
            + (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);

    Point { x, y }
}
