use glam::{Mat4, Vec3};
use std::time::{Duration, Instant};

use crate::scene::{
    camera_from_params, entity_to_solid, intersect_plane, mesh_bounds, pick_entity,
    ray_from_cursor, preset_angles, CameraMode, CameraPreset, GridPlane, SceneEntity,
    SceneEntityInfo, ScenePoint, SceneRect, SceneTool, SelectionMode, SketchFace, SolidKind,
    project_point,
};
use crate::cad;

#[derive(Debug, Clone, Copy, Default)]
pub struct SceneModifiers {
    pub shift: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dragging {
    None,
    Pan,
    Rotate,
    MoveEntity,
    CreateEntity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BoxCreateStep {
    BaseStart,
    BaseSize,
    Height,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CylinderCreateStep {
    BaseCenter,
    BaseRadius,
    Height,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ConeCreateStep {
    BaseCenter,
    BaseRadius,
    TopRadius,
    Height,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TorusCreateStep {
    Center,
    OuterRadius,
    InnerRadius,
}

fn height_from_cursor(
    origin: Vec3,
    dir: Vec3,
    base_center: Vec3,
    plane_normal: Vec3,
    camera_forward: Vec3,
) -> f32 {
    let denom = dir.dot(camera_forward);
    if denom.abs() <= 1.0e-6 {
        return 0.0;
    }
    let t = (base_center - origin).dot(camera_forward) / denom;
    let p = origin + dir * t;
    (p - base_center).dot(plane_normal).abs()
}

fn grid_axes(plane: GridPlane) -> (Vec3, Vec3, Vec3) {
    match plane {
        GridPlane::XY => (Vec3::X, Vec3::Y, Vec3::Z),
        GridPlane::XZ => (Vec3::X, Vec3::Z, Vec3::Y),
        GridPlane::YZ => (Vec3::Y, Vec3::Z, Vec3::X),
    }
}

fn rect_corners(plane: GridPlane, a: Vec3, b: Vec3) -> [Vec3; 4] {
    match plane {
        GridPlane::XY => {
            let z = a.z;
            let min_x = a.x.min(b.x);
            let max_x = a.x.max(b.x);
            let min_y = a.y.min(b.y);
            let max_y = a.y.max(b.y);
            [
                Vec3::new(min_x, min_y, z),
                Vec3::new(max_x, min_y, z),
                Vec3::new(max_x, max_y, z),
                Vec3::new(min_x, max_y, z),
            ]
        }
        GridPlane::XZ => {
            let y = a.y;
            let min_x = a.x.min(b.x);
            let max_x = a.x.max(b.x);
            let min_z = a.z.min(b.z);
            let max_z = a.z.max(b.z);
            [
                Vec3::new(min_x, y, min_z),
                Vec3::new(max_x, y, min_z),
                Vec3::new(max_x, y, max_z),
                Vec3::new(min_x, y, max_z),
            ]
        }
        GridPlane::YZ => {
            let x = a.x;
            let min_y = a.y.min(b.y);
            let max_y = a.y.max(b.y);
            let min_z = a.z.min(b.z);
            let max_z = a.z.max(b.z);
            [
                Vec3::new(x, min_y, min_z),
                Vec3::new(x, max_y, min_z),
                Vec3::new(x, max_y, max_z),
                Vec3::new(x, min_y, max_z),
            ]
        }
    }
}

fn base_size_and_center(plane: GridPlane, a: Vec3, b: Vec3) -> (Vec3, Vec3) {
    match plane {
        GridPlane::XY => {
            let min_x = a.x.min(b.x);
            let max_x = a.x.max(b.x);
            let min_y = a.y.min(b.y);
            let max_y = a.y.max(b.y);
            let z = a.z;
            let size = Vec3::new((max_x - min_x).abs(), (max_y - min_y).abs(), 0.0);
            let center = Vec3::new((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, z);
            (size, center)
        }
        GridPlane::XZ => {
            let min_x = a.x.min(b.x);
            let max_x = a.x.max(b.x);
            let min_z = a.z.min(b.z);
            let max_z = a.z.max(b.z);
            let y = a.y;
            let size = Vec3::new((max_x - min_x).abs(), 0.0, (max_z - min_z).abs());
            let center = Vec3::new((min_x + max_x) * 0.5, y, (min_z + max_z) * 0.5);
            (size, center)
        }
        GridPlane::YZ => {
            let min_y = a.y.min(b.y);
            let max_y = a.y.max(b.y);
            let min_z = a.z.min(b.z);
            let max_z = a.z.max(b.z);
            let x = a.x;
            let size = Vec3::new(0.0, (max_y - min_y).abs(), (max_z - min_z).abs());
            let center = Vec3::new(x, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5);
            (size, center)
        }
    }
}

fn line_segments_for_rect(corners: [Vec3; 4]) -> Vec<(Vec3, Vec3)> {
    vec![
        (corners[0], corners[1]),
        (corners[1], corners[2]),
        (corners[2], corners[3]),
        (corners[3], corners[0]),
    ]
}

fn box_frame_segments(plane: GridPlane, base_a: Vec3, base_b: Vec3, height: f32) -> Vec<(Vec3, Vec3)> {
    let base_corners = rect_corners(plane, base_a, base_b);
    let (_, _, n) = grid_axes(plane);
    let top_corners = base_corners.map(|p| p + n * height);
    let mut segments = line_segments_for_rect(base_corners);
    segments.extend(line_segments_for_rect(top_corners));
    for i in 0..4 {
        segments.push((base_corners[i], top_corners[i]));
    }
    segments
}

fn circle_segments(plane: GridPlane, center: Vec3, radius: f32, steps: usize) -> Vec<(Vec3, Vec3)> {
    let (u, v, _) = grid_axes(plane);
    let steps = steps.max(8);
    let mut points: Vec<Vec3> = Vec::with_capacity(steps);
    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius) + v * (s * radius));
    }
    let mut segments = Vec::with_capacity(steps);
    for i in 0..steps {
        let a = points[i];
        let b = points[(i + 1) % steps];
        segments.push((a, b));
    }
    segments
}

fn circle_points(plane: GridPlane, center: Vec3, radius: f32, steps: usize) -> Vec<Vec3> {
    let (u, v, _) = grid_axes(plane);
    let steps = steps.max(8);
    let mut points: Vec<Vec3> = Vec::with_capacity(steps);
    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius) + v * (s * radius));
    }
    points
}

fn dot_segments(plane: GridPlane, center: Vec3, dot_radius: f32) -> Vec<(Vec3, Vec3)> {
    circle_segments(plane, center, dot_radius.max(0.005), 12)
}

fn arc_segments(
    plane: GridPlane,
    center: Vec3,
    start: Vec3,
    end: Vec3,
    steps: usize,
) -> Vec<(Vec3, Vec3)> {
    let (u, v, _) = grid_axes(plane);
    let a = start - center;
    let b = end - center;
    let ax = a.dot(u);
    let ay = a.dot(v);
    let bx = b.dot(u);
    let by = b.dot(v);
    let start_angle = ay.atan2(ax);
    let mut end_angle = by.atan2(bx);
    if end_angle < start_angle {
        end_angle += std::f32::consts::TAU;
    }
    let radius = (ax * ax + ay * ay).sqrt().max(0.02);
    let steps = steps.max(8);
    let mut points: Vec<Vec3> = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = (i as f32) / (steps as f32);
        let angle = start_angle + (end_angle - start_angle) * t;
        let (s, c) = angle.sin_cos();
        points.push(center + u * (c * radius) + v * (s * radius));
    }
    let mut segments = Vec::with_capacity(steps);
    for i in 0..steps {
        segments.push((points[i], points[i + 1]));
    }
    segments
}

fn polyline_preview_segments(points: &[Vec3], current: Option<Vec3>) -> Vec<(Vec3, Vec3)> {
    let mut segments: Vec<(Vec3, Vec3)> = Vec::new();
    if points.len() >= 2 {
        for i in 0..(points.len() - 1) {
            segments.push((points[i], points[i + 1]));
        }
    }
    if let (Some(last), Some(cur)) = (points.last().copied(), current) {
        if last != cur {
            segments.push((last, cur));
        }
    }
    segments
}

fn outline_segments(points: &[Vec3]) -> Vec<(Vec3, Vec3)> {
    if points.len() < 2 {
        return Vec::new();
    }
    let mut segments = Vec::with_capacity(points.len());
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        segments.push((a, b));
    }
    segments
}

fn screen_distance_to_segment(p: (f32, f32), a: (f32, f32), b: (f32, f32)) -> f32 {
    let (px, py) = p;
    let (ax, ay) = a;
    let (bx, by) = b;
    let dx = bx - ax;
    let dy = by - ay;
    if dx.abs() + dy.abs() < 1.0e-6 {
        return ((px - ax).powi(2) + (py - ay).powi(2)).sqrt();
    }
    let t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
    let t = t.clamp(0.0, 1.0);
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
}

fn point_in_polygon_2d(point: (f32, f32), poly: &[(f32, f32)]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let (x, y) = point;
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (xi, yi) = poly[i];
        let (xj, yj) = poly[j];
        let intersect = ((yi > y) != (yj > y))
            && (x < (xj - xi) * (y - yi) / (yj - yi + 1.0e-6) + xi);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn entity_aabb(entity: &SceneEntity) -> (Vec3, Vec3) {
    let half = entity.size * 0.5;
    let min = entity.position - half;
    let max = entity.position + half;
    (min, max)
}

fn entity_corners(min: Vec3, max: Vec3) -> [Vec3; 8] {
    [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, max.z),
        Vec3::new(min.x, max.y, max.z),
    ]
}

fn entity_edges(corners: &[Vec3; 8]) -> Vec<(Vec3, Vec3)> {
    let idx = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    idx.iter().map(|(a, b)| (corners[*a], corners[*b])).collect()
}

fn entity_face_edges(corners: &[Vec3; 8], face: usize) -> Vec<(Vec3, Vec3)> {
    let faces = [
        [0, 1, 2, 3],
        [4, 5, 6, 7],
        [0, 1, 5, 4],
        [2, 3, 7, 6],
        [1, 2, 6, 5],
        [0, 3, 7, 4],
    ];
    let f = faces[face % 6];
    vec![
        (corners[f[0]], corners[f[1]]),
        (corners[f[1]], corners[f[2]]),
        (corners[f[2]], corners[f[3]]),
        (corners[f[3]], corners[f[0]]),
    ]
}

fn ray_box_face(origin: Vec3, dir: Vec3, min: Vec3, max: Vec3) -> Option<usize> {
    let mut tmin = (min.x - origin.x) / dir.x;
    let mut tmax = (max.x - origin.x) / dir.x;
    let mut face = if tmin < tmax { 0 } else { 1 };
    if tmin > tmax {
        std::mem::swap(&mut tmin, &mut tmax);
    }

    let mut tymin = (min.y - origin.y) / dir.y;
    let mut tymax = (max.y - origin.y) / dir.y;
    let ty_face = if tymin < tymax { 2 } else { 3 };
    if tymin > tymax {
        std::mem::swap(&mut tymin, &mut tymax);
    }
    if (tmin > tymax) || (tymin > tmax) {
        return None;
    }
    if tymin > tmin {
        tmin = tymin;
        face = ty_face;
    }
    if tymax < tmax {
        tmax = tymax;
    }

    let mut tzmin = (min.z - origin.z) / dir.z;
    let mut tzmax = (max.z - origin.z) / dir.z;
    let tz_face = if tzmin < tzmax { 4 } else { 5 };
    if tzmin > tzmax {
        std::mem::swap(&mut tzmin, &mut tzmax);
    }
    if (tmin > tzmax) || (tzmin > tmax) {
        return None;
    }
    if tzmin > tmin {
        face = tz_face;
    }
    Some(face)
}

fn build_mvp(camera: &crate::scene::Camera) -> Mat4 {
    let view = Mat4::look_at_rh(camera.eye, camera.eye + camera.forward, Vec3::Y);
    let proj = match camera.mode {
        CameraMode::Perspective => {
            Mat4::perspective_rh(camera.fovy, camera.aspect, camera.near, camera.far)
        }
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, camera.near, camera.far)
        }
    };
    proj * view
}

fn screen_point(mvp: Mat4, bounds: SceneRect, p: Vec3) -> Option<(f32, f32)> {
    project_point(mvp, bounds, p).map(|sp| (sp.x, sp.y))
}

fn vertex_marker_segments(p: Vec3, size: f32) -> Vec<(Vec3, Vec3)> {
    let s = size.max(0.01);
    vec![
        (p - Vec3::X * s, p + Vec3::X * s),
        (p - Vec3::Y * s, p + Vec3::Y * s),
        (p - Vec3::Z * s, p + Vec3::Z * s),
    ]
}

fn pick_2d_vertex(
    cursor: (f32, f32),
    mvp: Mat4,
    bounds: SceneRect,
    faces: &[SketchFace],
    tol: f32,
) -> Option<(usize, Vec3)> {
    let mut best: Option<(usize, Vec3, f32)> = None;
    for (fi, face) in faces.iter().enumerate() {
        for p in &face.points {
            let Some(sp) = screen_point(mvp, bounds, *p) else {
                continue;
            };
            let dist = ((cursor.0 - sp.0).powi(2) + (cursor.1 - sp.1).powi(2)).sqrt();
            if dist <= tol {
                if best.map(|b| dist < b.2).unwrap_or(true) {
                    best = Some((fi, *p, dist));
                }
            }
        }
    }
    best.map(|(fi, p, _)| (fi, p))
}

fn pick_2d_edge(
    cursor: (f32, f32),
    mvp: Mat4,
    bounds: SceneRect,
    faces: &[SketchFace],
    tol: f32,
) -> Option<(usize, (Vec3, Vec3))> {
    let mut best: Option<(usize, (Vec3, Vec3), f32)> = None;
    for (fi, face) in faces.iter().enumerate() {
        if face.points.len() < 2 {
            continue;
        }
        for i in 0..face.points.len() {
            let a = face.points[i];
            let b = face.points[(i + 1) % face.points.len()];
            let (Some(sa), Some(sb)) = (screen_point(mvp, bounds, a), screen_point(mvp, bounds, b)) else {
                continue;
            };
            let dist = screen_distance_to_segment(cursor, sa, sb);
            if dist <= tol {
                if best.map(|b| dist < b.2).unwrap_or(true) {
                    best = Some((fi, (a, b), dist));
                }
            }
        }
    }
    best.map(|(fi, seg, _)| (fi, seg))
}

fn pick_2d_face(
    cursor: (f32, f32),
    mvp: Mat4,
    bounds: SceneRect,
    faces: &[SketchFace],
) -> Option<usize> {
    for (fi, face) in faces.iter().enumerate() {
        let mut poly: Vec<(f32, f32)> = Vec::with_capacity(face.points.len());
        for p in &face.points {
            let Some(sp) = screen_point(mvp, bounds, *p) else {
                continue;
            };
            poly.push(sp);
        }
        if poly.len() >= 3 && point_in_polygon_2d(cursor, &poly) {
            return Some(fi);
        }
    }
    None
}

fn pick_3d_vertex(
    cursor: (f32, f32),
    mvp: Mat4,
    bounds: SceneRect,
    entities: &[SceneEntity],
    tol: f32,
) -> Option<(u64, Vec3)> {
    let mut best: Option<(u64, Vec3, f32)> = None;
    for e in entities {
        let (min, max) = entity_aabb(e);
        let corners = entity_corners(min, max);
        for c in corners.iter() {
            let Some(sp) = screen_point(mvp, bounds, *c) else {
                continue;
            };
            let dist = ((cursor.0 - sp.0).powi(2) + (cursor.1 - sp.1).powi(2)).sqrt();
            if dist <= tol {
                if best.map(|b| dist < b.2).unwrap_or(true) {
                    best = Some((e.id, *c, dist));
                }
            }
        }
    }
    best.map(|(id, p, _)| (id, p))
}

fn pick_3d_edge(
    cursor: (f32, f32),
    mvp: Mat4,
    bounds: SceneRect,
    entities: &[SceneEntity],
    tol: f32,
) -> Option<(u64, (Vec3, Vec3))> {
    let mut best: Option<(u64, (Vec3, Vec3), f32)> = None;
    for e in entities {
        let (min, max) = entity_aabb(e);
        let corners = entity_corners(min, max);
        let edges = entity_edges(&corners);
        for (a, b) in edges {
            let (Some(sa), Some(sb)) = (screen_point(mvp, bounds, a), screen_point(mvp, bounds, b)) else {
                continue;
            };
            let dist = screen_distance_to_segment(cursor, sa, sb);
            if dist <= tol {
                if best.map(|b| dist < b.2).unwrap_or(true) {
                    best = Some((e.id, (a, b), dist));
                }
            }
        }
    }
    best.map(|(id, seg, _)| (id, seg))
}

fn ellipse_segments(
    plane: GridPlane,
    center: Vec3,
    radius_u: f32,
    radius_v: f32,
    steps: usize,
) -> Vec<(Vec3, Vec3)> {
    let (u, v, _) = grid_axes(plane);
    let steps = steps.max(8);
    let mut points: Vec<Vec3> = Vec::with_capacity(steps);
    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius_u) + v * (s * radius_v));
    }
    let mut segments = Vec::with_capacity(steps);
    for i in 0..steps {
        let a = points[i];
        let b = points[(i + 1) % steps];
        segments.push((a, b));
    }
    segments
}

fn ellipse_points(
    plane: GridPlane,
    center: Vec3,
    radius_u: f32,
    radius_v: f32,
    steps: usize,
) -> Vec<Vec3> {
    let (u, v, _) = grid_axes(plane);
    let steps = steps.max(8);
    let mut points: Vec<Vec3> = Vec::with_capacity(steps);
    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius_u) + v * (s * radius_v));
    }
    points
}

fn regular_polygon_segments(
    plane: GridPlane,
    center: Vec3,
    radius: f32,
    sides: usize,
) -> Vec<(Vec3, Vec3)> {
    let (u, v, _) = grid_axes(plane);
    let sides = sides.max(3);
    let mut points: Vec<Vec3> = Vec::with_capacity(sides);
    for i in 0..sides {
        let t = (i as f32) / (sides as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius) + v * (s * radius));
    }
    let mut segments = Vec::with_capacity(sides);
    for i in 0..sides {
        let a = points[i];
        let b = points[(i + 1) % sides];
        segments.push((a, b));
    }
    segments
}

fn regular_polygon_points(
    plane: GridPlane,
    center: Vec3,
    radius: f32,
    sides: usize,
) -> Vec<Vec3> {
    let (u, v, _) = grid_axes(plane);
    let sides = sides.max(3);
    let mut points: Vec<Vec3> = Vec::with_capacity(sides);
    for i in 0..sides {
        let t = (i as f32) / (sides as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        points.push(center + u * (c * radius) + v * (s * radius));
    }
    points
}

fn bezier_segments(a: Vec3, b: Vec3, control: Vec3, steps: usize) -> Vec<(Vec3, Vec3)> {
    let steps = steps.max(8);
    let mut segments = Vec::with_capacity(steps);
    let mut prev = a;
    for i in 1..=steps {
        let t = (i as f32) / (steps as f32);
        let omt = 1.0 - t;
        let p = a * (omt * omt) + control * (2.0 * omt * t) + b * (t * t);
        segments.push((prev, p));
        prev = p;
    }
    segments
}

fn dashed_segments(a: Vec3, b: Vec3, dash_len: f32, gap_len: f32) -> Vec<(Vec3, Vec3)> {
    let delta = b - a;
    let dist = delta.length();
    if dist <= 1.0e-6 {
        return Vec::new();
    }

    let dash_len = dash_len.max(0.001);
    let gap_len = gap_len.max(0.001);
    let step = dash_len + gap_len;
    let dir = delta / dist;
    let mut segments = Vec::new();
    let mut t = 0.0;
    while t < dist {
        let start = a + dir * t;
        let end = a + dir * (t + dash_len).min(dist);
        segments.push((start, end));
        t += step;
    }
    segments
}

fn point_cross_segments(plane: GridPlane, center: Vec3, grid_step: f32) -> Vec<(Vec3, Vec3)> {
    let (u, v, _) = grid_axes(plane);
    let point_size = grid_step.max(0.05) * 0.3;
    vec![
        (center - u * point_size, center + u * point_size),
        (center - v * point_size, center + v * point_size),
    ]
}

fn sketch_segments_for_tool(
    tool: SceneTool,
    plane: GridPlane,
    grid_step: f32,
    start: Vec3,
    current: Vec3,
) -> Vec<(Vec3, Vec3)> {
    match tool {
        SceneTool::SketchPoint => point_cross_segments(plane, start, grid_step),
        SceneTool::SketchLine => vec![(start, current)],
        SceneTool::SketchRect => {
            let corners = rect_corners(plane, start, current);
            line_segments_for_rect(corners)
        }
        SceneTool::SketchPolygon => {
            let (u, v, _) = grid_axes(plane);
            let du = (current - start).dot(u).abs();
            let dv = (current - start).dot(v).abs();
            let radius = (du * du + dv * dv).sqrt().max(0.02);
            regular_polygon_segments(plane, start, radius, 6)
        }
        SceneTool::SketchCircle => {
            let (u, v, _) = grid_axes(plane);
            let du = (current - start).dot(u);
            let dv = (current - start).dot(v);
            let radius = (du * du + dv * dv).sqrt().max(0.02);
            circle_segments(plane, start, radius, 32)
        }
        SceneTool::SketchEllipse => {
            let (u, v, _) = grid_axes(plane);
            let radius_u = (current - start).dot(u).abs().max(0.02);
            let radius_v = (current - start).dot(v).abs().max(0.02);
            ellipse_segments(plane, start, radius_u, radius_v, 36)
        }
        SceneTool::SketchArc | SceneTool::SketchBezier => {
            let (u, v, _) = grid_axes(plane);
            let dir = current - start;
            let len = dir.length().max(0.02);
            let du = dir.dot(u);
            let dv = dir.dot(v);
            let perp = (-dv * u + du * v).normalize_or_zero();
            let mid = (start + current) * 0.5;
            let factor = if matches!(tool, SceneTool::SketchArc) { 0.6 } else { 0.3 };
            let control = mid + perp * (len * factor);
            bezier_segments(start, current, control, 32)
        }
        _ => Vec::new(),
    }
}

fn clamp_inner_radius(outer_radius: f32, inner_radius: f32, min_minor: f32) -> f32 {
    if outer_radius <= 0.0 {
        return 0.0;
    }
    let max_inner = (outer_radius - min_minor * 2.0).max(0.0);
    inner_radius.min(max_inner).max(0.0)
}

fn cylinder_frame_segments(
    plane: GridPlane,
    center: Vec3,
    radius: f32,
    height: f32,
    steps: usize,
) -> Vec<(Vec3, Vec3)> {
    let (u, v, n) = grid_axes(plane);
    let steps = steps.max(8);
    let mut base_points: Vec<Vec3> = Vec::with_capacity(steps);
    let mut top_points: Vec<Vec3> = Vec::with_capacity(steps);
    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        let base = center + u * (c * radius) + v * (s * radius);
        base_points.push(base);
        top_points.push(base + n * height);
    }

    let mut segments = Vec::with_capacity(steps * 3);
    for i in 0..steps {
        let next = (i + 1) % steps;
        segments.push((base_points[i], base_points[next]));
        segments.push((top_points[i], top_points[next]));
        segments.push((base_points[i], top_points[i]));
    }
    segments
}

fn cone_frame_segments(
    plane: GridPlane,
    center: Vec3,
    base_radius: f32,
    top_radius: f32,
    height: f32,
    steps: usize,
    dot_radius: f32,
) -> Vec<(Vec3, Vec3)> {
    let (u, v, n) = grid_axes(plane);
    let steps = steps.max(8);
    let mut base_points: Vec<Vec3> = Vec::with_capacity(steps);
    let mut top_points: Vec<Vec3> = Vec::with_capacity(steps);
    let top_center = center + n * height;

    for i in 0..steps {
        let t = (i as f32) / (steps as f32) * std::f32::consts::TAU;
        let (s, c) = t.sin_cos();
        let base = center + u * (c * base_radius) + v * (s * base_radius);
        let top = top_center + u * (c * top_radius) + v * (s * top_radius);
        base_points.push(base);
        top_points.push(top);
    }

    let mut segments = Vec::with_capacity(steps * 3);
    for i in 0..steps {
        let next = (i + 1) % steps;
        segments.push((base_points[i], base_points[next]));
        if top_radius > 0.0001 {
            segments.push((top_points[i], top_points[next]));
        }
        segments.push((base_points[i], top_points[i]));
    }

    if top_radius <= 0.0001 {
        segments.extend(dot_segments(plane, top_center, dot_radius));
    }

    segments
}

#[derive(Debug, Clone)]
pub struct SceneModel {
    target: Vec3,
    distance: f32,
    yaw: f32,
    pitch: f32,
    dragging: Dragging,
    last_cursor: Option<ScenePoint>,
    entities: Vec<SceneEntity>,
    entities_version: u64,
    selected: Option<u64>,
    selected_entities: Vec<u64>,
    selected_entities_version: u64,
    selected_face: Option<usize>,
    next_id: u64,
    drag_start: Option<Vec3>,
    drag_offset: Vec3,
    modifiers: SceneModifiers,
    sphere_center: Option<Vec3>,
    preview_segments: Vec<(Vec3, Vec3)>,
    preview_version: u64,
    sketch_segments: Vec<(Vec3, Vec3)>,
    sketch_version: u64,
    sketch_faces: Vec<SketchFace>,
    sketch_faces_version: u64,
    face_highlight_segments: Vec<(Vec3, Vec3)>,
    face_highlight_version: u64,
    hover_segments: Vec<(Vec3, Vec3)>,
    hover_version: u64,
    sketch_active: bool,
    sketch_start: Option<Vec3>,
    sketch_current: Option<Vec3>,
    sketch_points: Vec<Vec3>,
    last_left_click: Option<(ScenePoint, Instant)>,
    box_step: Option<BoxCreateStep>,
    box_base_start: Option<Vec3>,
    box_base_end: Option<Vec3>,
    box_base_size: Option<Vec3>,
    box_height: Option<f32>,
    cylinder_step: Option<CylinderCreateStep>,
    cylinder_center: Option<Vec3>,
    cylinder_radius: Option<f32>,
    cylinder_height: Option<f32>,
    cone_step: Option<ConeCreateStep>,
    cone_center: Option<Vec3>,
    cone_base_radius: Option<f32>,
    cone_top_radius: Option<f32>,
    cone_height: Option<f32>,
    torus_step: Option<TorusCreateStep>,
    torus_center: Option<Vec3>,
    torus_outer_radius: Option<f32>,
    torus_inner_radius: Option<f32>,
}

impl Default for SceneModel {
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
            selected_entities: Vec::new(),
            selected_entities_version: 0,
            selected_face: None,
            next_id: 1,
            drag_start: None,
            drag_offset: Vec3::ZERO,
            modifiers: SceneModifiers::default(),
            sphere_center: None,
            preview_segments: Vec::new(),
            preview_version: 0,
            sketch_segments: Vec::new(),
            sketch_version: 0,
            sketch_faces: Vec::new(),
            sketch_faces_version: 0,
            face_highlight_segments: Vec::new(),
            face_highlight_version: 0,
            hover_segments: Vec::new(),
            hover_version: 0,
            sketch_active: false,
            sketch_start: None,
            sketch_current: None,
            sketch_points: Vec::new(),
            last_left_click: None,
            box_step: None,
            box_base_start: None,
            box_base_end: None,
            box_base_size: None,
            box_height: None,
            cylinder_step: None,
            cylinder_center: None,
            cylinder_radius: None,
            cylinder_height: None,
            cone_step: None,
            cone_center: None,
            cone_base_radius: None,
            cone_top_radius: None,
            cone_height: None,
            torus_step: None,
            torus_center: None,
            torus_outer_radius: None,
            torus_inner_radius: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SceneConfig {
    pub show_grid: bool,
    pub grid_plane: GridPlane,
    pub grid_extent: f32,
    pub grid_step: f32,
    pub tool: SceneTool,
    pub selection_mode: SelectionMode,
    pub camera_mode: CameraMode,
    pub camera_preset: Option<CameraPreset>,
    pub axes_enabled: bool,
    pub axes_size: f32,
    pub axes_margin: f32,
    pub background: [f32; 4],
}

#[derive(Debug, Clone, Copy)]
pub struct SceneRequests {
    pub select_id: Option<u64>,
    pub focus_id: Option<u64>,
}

#[derive(Debug, Clone, Copy)]
pub enum SceneInput {
    ModifiersChanged { shift: bool },
    KeyDelete,
    KeyEscape,
    Unite,
    MouseWheel { delta_lines: f32 },
    MouseDownLeft { pos: ScenePoint },
    MouseUpLeft,
    MouseDownRight { pos: ScenePoint },
    MouseUpRight,
    MouseMove { pos: ScenePoint },
}

#[derive(Debug, Default, Clone)]
pub struct SceneUpdateResult {
    pub handled: bool,
    pub request_redraw: bool,
    pub capture: bool,
    pub publish_entities: Option<Vec<SceneEntityInfo>>,
    pub finished_tool: Option<SceneTool>,
    pub selection_label: Option<String>,
}

#[derive(Debug, Clone)]
pub struct SceneView {
    pub target: Vec3,
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub camera_mode: CameraMode,
    pub show_grid: bool,
    pub grid_plane: GridPlane,
    pub grid_extent: f32,
    pub grid_step: f32,
    pub entities: std::sync::Arc<Vec<SceneEntity>>,
    pub entities_version: u64,
    pub preview_segments: std::sync::Arc<Vec<(Vec3, Vec3)>>,
    pub preview_version: u64,
    pub sketch_segments: std::sync::Arc<Vec<(Vec3, Vec3)>>,
    pub sketch_version: u64,
    pub sketch_faces: std::sync::Arc<Vec<SketchFace>>,
    pub sketch_faces_version: u64,
    pub face_highlight_segments: std::sync::Arc<Vec<(Vec3, Vec3)>>,
    pub face_highlight_version: u64,
    pub hover_segments: std::sync::Arc<Vec<(Vec3, Vec3)>>,
    pub hover_version: u64,
    pub selected_entities: std::sync::Arc<Vec<u64>>,
    pub selected_entities_version: u64,
    pub axes_enabled: bool,
    pub axes_size: f32,
    pub axes_margin: f32,
    pub background: [f32; 4],
}

impl SceneModel {
    pub fn is_dragging(&self) -> bool {
        self.dragging != Dragging::None
    }

    pub fn apply_zoom_factor(&mut self, factor: f32) {
        if (factor - 1.0).abs() <= f32::EPSILON {
            return;
        }
        let safe = if factor.is_finite() && factor > 0.0 { factor } else { 1.0 };
        self.distance = (self.distance / safe).clamp(0.1, 200.0);
    }

    pub fn view(&self, config: SceneConfig) -> SceneView {
        let (yaw, pitch) = if let Some(p) = config.camera_preset {
            preset_angles(p)
        } else {
            (self.yaw, self.pitch)
        };

        SceneView {
            target: self.target,
            distance: self.distance,
            yaw,
            pitch,
            camera_mode: config.camera_mode,
            show_grid: config.show_grid,
            grid_plane: config.grid_plane,
            grid_extent: config.grid_extent,
            grid_step: config.grid_step,
            entities: std::sync::Arc::new(self.entities.clone()),
            entities_version: self.entities_version,
            preview_segments: std::sync::Arc::new(self.preview_segments.clone()),
            preview_version: self.preview_version,
            sketch_segments: std::sync::Arc::new(self.sketch_segments.clone()),
            sketch_version: self.sketch_version,
            sketch_faces: std::sync::Arc::new(self.sketch_faces.clone()),
            sketch_faces_version: self.sketch_faces_version,
            face_highlight_segments: std::sync::Arc::new(self.face_highlight_segments.clone()),
            face_highlight_version: self.face_highlight_version,
            hover_segments: std::sync::Arc::new(self.hover_segments.clone()),
            hover_version: self.hover_version,
            selected_entities: std::sync::Arc::new(self.selected_entities.clone()),
            selected_entities_version: self.selected_entities_version,
            axes_enabled: config.axes_enabled,
            axes_size: config.axes_size,
            axes_margin: config.axes_margin,
            background: config.background,
        }
    }

    pub fn update(
        &mut self,
        input: SceneInput,
        bounds: SceneRect,
        config: SceneConfig,
        requests: SceneRequests,
    ) -> SceneUpdateResult {
        let mut result = SceneUpdateResult::default();

        let mut changed = false;
        if let Some(id) = requests.select_id {
            if self.entities.iter().any(|e| e.id == id) && self.selected != Some(id) {
                self.selected = Some(id);
                self.selected_entities.clear();
                self.selected_entities.push(id);
                self.selected_entities_version =
                    self.selected_entities_version.wrapping_add(1);
                changed = true;
            }
        }
        if let Some(id) = requests.focus_id {
            if let Some(ent) = self.entities.iter().find(|e| e.id == id) {
                self.target = ent.position;
                changed = true;
            }
        }
        if changed {
            result.request_redraw = true;
        }

        match input {
            SceneInput::ModifiersChanged { shift } => {
                self.modifiers.shift = shift;
                result.handled = false;
                return result;
            }
            SceneInput::KeyEscape => {
                if self.sketch_active {
                    self.sketch_active = false;
                    self.sketch_start = None;
                    self.sketch_current = None;
                    self.sketch_points.clear();
                    self.dragging = Dragging::None;
                    self.last_cursor = None;
                    self.last_left_click = None;
                    self.preview_segments.clear();
                    self.preview_version = self.preview_version.wrapping_add(1);
                    result.finished_tool = Some(config.tool);
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                    return result;
                }

                if self.box_step.is_some()
                    || self.sphere_center.is_some()
                    || self.cylinder_step.is_some()
                    || self.cone_step.is_some()
                    || self.torus_step.is_some()
                {
                    self.box_step = None;
                    self.box_base_start = None;
                    self.box_base_end = None;
                    self.box_base_size = None;
                    self.box_height = None;
                    self.sphere_center = None;
                    self.cylinder_step = None;
                    self.cylinder_center = None;
                    self.cylinder_radius = None;
                    self.cylinder_height = None;
                    self.cone_step = None;
                    self.cone_center = None;
                    self.cone_base_radius = None;
                    self.cone_top_radius = None;
                    self.cone_height = None;
                    self.torus_step = None;
                    self.torus_center = None;
                    self.torus_outer_radius = None;
                    self.torus_inner_radius = None;
                    self.preview_segments.clear();
                    self.preview_version = self.preview_version.wrapping_add(1);
                    result.finished_tool = Some(config.tool);
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                }
                return result;
            }
            SceneInput::KeyDelete => {
                if !self.selected_entities.is_empty() {
                    let before = self.entities.len();
                    let selected = self.selected_entities.clone();
                    self.entities
                        .retain(|e| !selected.iter().any(|id| *id == e.id));
                    if self.entities.len() != before {
                        self.entities_version = self.entities_version.wrapping_add(1);
                        result.publish_entities = Some(
                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                        );
                        result.capture = true;
                    }
                    self.selected = None;
                    self.selected_entities.clear();
                    self.selected_entities_version =
                        self.selected_entities_version.wrapping_add(1);
                    result.request_redraw = true;
                    result.handled = true;
                    return result;
                }

                if let Some(selected) = self.selected {
                    let before = self.entities.len();
                    self.entities.retain(|e| e.id != selected);
                    if self.entities.len() != before {
                        self.entities_version = self.entities_version.wrapping_add(1);
                        result.publish_entities = Some(
                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                        );
                        result.capture = true;
                    }
                    self.selected = None;
                    self.selected_entities.clear();
                    self.selected_entities_version =
                        self.selected_entities_version.wrapping_add(1);
                    result.request_redraw = true;
                    result.handled = true;
                    return result;
                }
                return result;
            }
            SceneInput::MouseUpLeft => {
                if matches!(self.dragging, Dragging::MoveEntity | Dragging::CreateEntity) {
                    self.dragging = Dragging::None;
                    self.last_cursor = None;
                    self.drag_start = None;
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                }

                return result;
            }
            SceneInput::MouseUpRight => {
                if matches!(self.dragging, Dragging::Pan | Dragging::Rotate) {
                    self.dragging = Dragging::None;
                    self.last_cursor = None;
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                }
                return result;
            }
            _ => {}
        }

        let scene_bounds = bounds;
        let (yaw, pitch) = if let Some(p) = config.camera_preset {
            preset_angles(p)
        } else {
            (self.yaw, self.pitch)
        };
        let camera = camera_from_params(
            self.target,
            self.distance,
            yaw,
            pitch,
            scene_bounds,
            config.camera_mode,
        );

        match input {
            SceneInput::Unite => {
                let mut ids = if !self.selected_entities.is_empty() {
                    self.selected_entities.clone()
                } else if let Some(id) = self.selected {
                    vec![id]
                } else {
                    Vec::new()
                };

                ids.sort_unstable();
                ids.dedup();

                if ids.len() < 2 {
                    return result;
                }

                let mut selected = Vec::new();
                for id in &ids {
                    if let Some(entity) = self.entities.iter().find(|e| e.id == *id) {
                        selected.push(entity);
                    }
                }

                if selected.len() < 2 {
                    return result;
                }

                let solids = selected
                    .iter()
                    .map(|entity| entity_to_solid(entity))
                    .collect::<Vec<_>>();
                let solid = cad::solid_unite(solids.as_slice());
                let mesh = cad::to_mesh(&solid);

                let (min, max) = match mesh_bounds(&mesh) {
                    Some(bounds) => bounds,
                    None => return result,
                };

                let center = (min + max) * 0.5;
                let size = (max - min).max(Vec3::splat(0.01));

                let new_id = self.next_id;
                self.next_id += 1;

                self.entities.retain(|e| !ids.iter().any(|id| *id == e.id));
                self.entities.push(SceneEntity {
                    id: new_id,
                    kind: SolidKind::Custom,
                    position: center,
                    size,
                    plane: config.grid_plane,
                    mesh: Some(mesh),
                });

                self.entities_version = self.entities_version.wrapping_add(1);
                self.selected = Some(new_id);
                self.selected_entities.clear();
                self.selected_entities.push(new_id);
                self.selected_entities_version =
                    self.selected_entities_version.wrapping_add(1);

                result.publish_entities = Some(
                    self.entities.iter().map(SceneEntityInfo::from).collect(),
                );
                result.request_redraw = true;
                result.capture = true;
                result.handled = true;
                return result;
            }
            SceneInput::MouseWheel { delta_lines } => {
                if delta_lines.abs() > f32::EPSILON {
                    let factor = 1.1_f32.powf(delta_lines);
                    self.distance = (self.distance / factor).clamp(0.1, 200.0);
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                }
            }
            SceneInput::MouseDownLeft { pos } => {
                if config.tool.is_sketch() {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            let now = Instant::now();
                            let double_click = self
                                .last_left_click
                                .map(|(last_pos, t)| {
                                    now.duration_since(t) <= Duration::from_millis(350)
                                        && (last_pos.x - pos.x).abs() <= 6.0
                                        && (last_pos.y - pos.y).abs() <= 6.0
                                })
                                .unwrap_or(false);
                            self.last_left_click = Some((pos, now));

                            match config.tool {
                                SceneTool::SketchPoint => {
                                    let segments =
                                        point_cross_segments(config.grid_plane, hit_pos, config.grid_step);
                                    self.sketch_segments.extend(segments);
                                    self.sketch_version = self.sketch_version.wrapping_add(1);
                                    result.finished_tool = Some(config.tool);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                SceneTool::SketchLine => {
                                    if !self.sketch_active {
                                        self.sketch_active = true;
                                        self.sketch_points.clear();
                                        self.sketch_points.push(hit_pos);
                                        self.sketch_current = Some(hit_pos);
                                    } else if double_click {
                                        if self.sketch_points.last().copied() != Some(hit_pos) {
                                            self.sketch_points.push(hit_pos);
                                        }
                                        let segments =
                                            polyline_preview_segments(&self.sketch_points, None);
                                        if !segments.is_empty() {
                                            self.sketch_segments.extend(segments);
                                            self.sketch_version = self.sketch_version.wrapping_add(1);
                                        }
                                        self.sketch_active = false;
                                        self.sketch_points.clear();
                                        self.sketch_start = None;
                                        self.sketch_current = None;
                                        self.preview_segments.clear();
                                        self.preview_version = self.preview_version.wrapping_add(1);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    } else {
                                        self.sketch_points.push(hit_pos);
                                        self.sketch_current = Some(hit_pos);
                                    }

                                    self.preview_segments =
                                        polyline_preview_segments(&self.sketch_points, self.sketch_current);
                                    self.preview_version = self.preview_version.wrapping_add(1);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                SceneTool::SketchRect
                                | SceneTool::SketchCircle
                                | SceneTool::SketchEllipse
                                | SceneTool::SketchBezier
                                | SceneTool::SketchPolygon => {
                                    if !self.sketch_active {
                                        self.sketch_active = true;
                                        self.sketch_start = Some(hit_pos);
                                        self.sketch_current = Some(hit_pos);
                                    } else {
                                        self.sketch_current = Some(hit_pos);
                                        if let Some(start) = self.sketch_start {
                                            if matches!(
                                                config.tool,
                                                SceneTool::SketchRect
                                                    | SceneTool::SketchCircle
                                                    | SceneTool::SketchEllipse
                                                    | SceneTool::SketchPolygon
                                            ) {
                                                let face_points = match config.tool {
                                                    SceneTool::SketchRect => {
                                                        rect_corners(config.grid_plane, start, hit_pos).to_vec()
                                                    }
                                                    SceneTool::SketchCircle => {
                                                        let (u, v, _) = grid_axes(config.grid_plane);
                                                        let du = (hit_pos - start).dot(u);
                                                        let dv = (hit_pos - start).dot(v);
                                                        let radius = (du * du + dv * dv).sqrt().max(0.02);
                                                        circle_points(config.grid_plane, start, radius, 48)
                                                    }
                                                    SceneTool::SketchEllipse => {
                                                        let (u, v, _) = grid_axes(config.grid_plane);
                                                        let radius_u = (hit_pos - start).dot(u).abs().max(0.02);
                                                        let radius_v = (hit_pos - start).dot(v).abs().max(0.02);
                                                        ellipse_points(
                                                            config.grid_plane,
                                                            start,
                                                            radius_u,
                                                            radius_v,
                                                            48,
                                                        )
                                                    }
                                                    SceneTool::SketchPolygon => {
                                                        let (u, v, _) = grid_axes(config.grid_plane);
                                                        let du = (hit_pos - start).dot(u).abs();
                                                        let dv = (hit_pos - start).dot(v).abs();
                                                        let radius = (du * du + dv * dv).sqrt().max(0.02);
                                                        regular_polygon_points(
                                                            config.grid_plane,
                                                            start,
                                                            radius,
                                                            6,
                                                        )
                                                    }
                                                    _ => Vec::new(),
                                                };

                                                if face_points.len() >= 3 {
                                                    self.sketch_faces.push(SketchFace {
                                                        points: face_points,
                                                    });
                                                    self.sketch_faces_version =
                                                        self.sketch_faces_version.wrapping_add(1);
                                                    if let Some(index) = self.selected_face {
                                                        if let Some(face) =
                                                            self.sketch_faces.get(index)
                                                        {
                                                            self.face_highlight_segments =
                                                                outline_segments(&face.points);
                                                            self.face_highlight_version = self
                                                                .face_highlight_version
                                                                .wrapping_add(1);
                                                        }
                                                    }
                                                }
                                            }

                                            let segments = sketch_segments_for_tool(
                                                config.tool,
                                                config.grid_plane,
                                                config.grid_step,
                                                start,
                                                hit_pos,
                                            );
                                            if !segments.is_empty() {
                                                self.sketch_segments.extend(segments);
                                                self.sketch_version = self.sketch_version.wrapping_add(1);
                                            }
                                        }
                                        self.sketch_active = false;
                                        self.sketch_start = None;
                                        self.sketch_current = None;
                                        self.preview_segments.clear();
                                        self.preview_version = self.preview_version.wrapping_add(1);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }

                                    if let (Some(start), Some(cur)) =
                                        (self.sketch_start, self.sketch_current)
                                    {
                                        self.preview_segments = sketch_segments_for_tool(
                                            config.tool,
                                            config.grid_plane,
                                            config.grid_step,
                                            start,
                                            cur,
                                        );
                                        self.preview_version = self.preview_version.wrapping_add(1);
                                    }

                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                SceneTool::SketchArc => {
                                    if !self.sketch_active {
                                        self.sketch_active = true;
                                        self.sketch_points.clear();
                                        self.sketch_points.push(hit_pos); // center
                                        self.preview_segments =
                                            point_cross_segments(config.grid_plane, hit_pos, config.grid_step);
                                    } else if self.sketch_points.len() == 1 {
                                        self.sketch_points.push(hit_pos); // start
                                        self.preview_segments =
                                            vec![(self.sketch_points[0], self.sketch_points[1])];
                                    } else {
                                        let center = self.sketch_points[0];
                                        let start = self.sketch_points[1];
                                        let end = hit_pos;
                                        let segments =
                                            arc_segments(config.grid_plane, center, start, end, 36);
                                        if !segments.is_empty() {
                                            self.sketch_segments.extend(segments);
                                            self.sketch_version = self.sketch_version.wrapping_add(1);
                                        }
                                        self.sketch_active = false;
                                        self.sketch_points.clear();
                                        self.preview_segments.clear();
                                        self.preview_version = self.preview_version.wrapping_add(1);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }

                                    self.preview_version = self.preview_version.wrapping_add(1);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                _ => {}
                            }

                        }
                    }
                }

                if config.tool == SceneTool::Box {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            match self.box_step.unwrap_or(BoxCreateStep::BaseStart) {
                                BoxCreateStep::BaseStart => {
                                    self.box_step = Some(BoxCreateStep::BaseSize);
                                    self.box_base_start = Some(hit_pos);
                                    self.box_base_end = None;
                                    self.box_base_size = None;
                                    self.box_height = None;
                                    let (u, v, _) = grid_axes(config.grid_plane);
                                    let point_size = config.grid_step.max(0.05) * 0.3;
                                    self.preview_segments = vec![
                                        (hit_pos - u * point_size, hit_pos + u * point_size),
                                        (hit_pos - v * point_size, hit_pos + v * point_size),
                                    ];
                                    self.preview_version =
                                        self.preview_version.wrapping_add(1);
                                    self.last_cursor = Some(pos);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                BoxCreateStep::BaseSize => {
                                    if let Some(start) = self.box_base_start {
                                        let (size, _) =
                                            base_size_and_center(config.grid_plane, start, hit_pos);
                                        self.box_base_size = Some(size);
                                        self.box_base_end = Some(hit_pos);
                                        self.box_height = Some(0.0);
                                        self.box_step = Some(BoxCreateStep::Height);

                                        let segments =
                                            box_frame_segments(config.grid_plane, start, hit_pos, 0.0);
                                        self.preview_segments = segments;
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                                BoxCreateStep::Height => {
                                    if let (Some(start), Some(end)) =
                                        (self.box_base_start, self.box_base_end)
                                    {
                                        let (_, _, n) = grid_axes(config.grid_plane);
                                        let (base_size, base_center) =
                                            base_size_and_center(config.grid_plane, start, end);
                                        let height = if let Some((origin, dir)) =
                                            ray_from_cursor(pos, scene_bounds, &camera)
                                        {
                                            height_from_cursor(
                                                origin,
                                                dir,
                                                base_center,
                                                n,
                                                camera.forward,
                                            )
                                        } else {
                                            0.0
                                        };

                                        let h = height.max(0.05);
                                        let size3 = match config.grid_plane {
                                            GridPlane::XY => Vec3::new(base_size.x, base_size.y, h),
                                            GridPlane::XZ => Vec3::new(base_size.x, h, base_size.z),
                                            GridPlane::YZ => Vec3::new(h, base_size.y, base_size.z),
                                        };
                                        let center = match config.grid_plane {
                                            GridPlane::XY => base_center + Vec3::new(0.0, 0.0, h * 0.5),
                                            GridPlane::XZ => base_center + Vec3::new(0.0, h * 0.5, 0.0),
                                            GridPlane::YZ => base_center + Vec3::new(h * 0.5, 0.0, 0.0),
                                        };

                                        let id = self.next_id;
                                        self.next_id += 1;
                                        self.entities.push(SceneEntity {
                                            id,
                                            kind: SolidKind::Box,
                                            position: center,
                                            size: size3,
                                            plane: config.grid_plane,
                                            mesh: None,
                                        });
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );

                                        self.selected = Some(id);
                                        self.box_step = None;
                                        self.box_base_start = None;
                                        self.box_base_end = None;
                                        self.box_base_size = None;
                                        self.box_height = None;
                                        self.preview_segments.clear();
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Cylinder {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            match self
                                .cylinder_step
                                .unwrap_or(CylinderCreateStep::BaseCenter)
                            {
                                CylinderCreateStep::BaseCenter => {
                                    self.cylinder_step = Some(CylinderCreateStep::BaseRadius);
                                    self.cylinder_center = Some(hit_pos);
                                    self.cylinder_radius = None;
                                    self.cylinder_height = None;

                                    let (u, v, _) = grid_axes(config.grid_plane);
                                    let point_size = config.grid_step.max(0.05) * 0.3;
                                    self.preview_segments = vec![
                                        (hit_pos - u * point_size, hit_pos + u * point_size),
                                        (hit_pos - v * point_size, hit_pos + v * point_size),
                                    ];
                                    self.preview_version =
                                        self.preview_version.wrapping_add(1);
                                    self.last_cursor = Some(pos);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                CylinderCreateStep::BaseRadius => {
                                    if let Some(center) = self.cylinder_center {
                                        let radius = (hit_pos - center).length().max(0.05);
                                        self.cylinder_radius = Some(radius);
                                        self.cylinder_height = Some(0.0);
                                        self.cylinder_step = Some(CylinderCreateStep::Height);

                                        let segments = cylinder_frame_segments(
                                            config.grid_plane,
                                            center,
                                            radius,
                                            0.0,
                                            48,
                                        );
                                        self.preview_segments = segments;
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                                CylinderCreateStep::Height => {
                                    if let (Some(center), Some(radius)) =
                                        (self.cylinder_center, self.cylinder_radius)
                                    {
                                        let (_, _, n) = grid_axes(config.grid_plane);
                                        let height = if let Some((origin, dir)) =
                                            ray_from_cursor(pos, scene_bounds, &camera)
                                        {
                                            height_from_cursor(
                                                origin,
                                                dir,
                                                center,
                                                n,
                                                camera.forward,
                                            )
                                        } else {
                                            0.0
                                        };

                                        let h = height.max(0.05);
                                        let size = Vec3::new(radius / 0.35, h, radius / 0.35);
                                        let position = center + n * (h * 0.5);

                                        let id = self.next_id;
                                        self.next_id += 1;
                                        self.entities.push(SceneEntity {
                                            id,
                                            kind: SolidKind::Cylinder,
                                            position,
                                            size,
                                            plane: config.grid_plane,
                                            mesh: None,
                                        });
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );

                                        self.selected = Some(id);
                                        self.cylinder_step = None;
                                        self.cylinder_center = None;
                                        self.cylinder_radius = None;
                                        self.cylinder_height = None;
                                        self.preview_segments.clear();
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Cone {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            match self.cone_step.unwrap_or(ConeCreateStep::BaseCenter) {
                                ConeCreateStep::BaseCenter => {
                                    self.cone_step = Some(ConeCreateStep::BaseRadius);
                                    self.cone_center = Some(hit_pos);
                                    self.cone_base_radius = None;
                                    self.cone_top_radius = None;
                                    self.cone_height = None;

                                    let (u, v, _) = grid_axes(config.grid_plane);
                                    let point_size = config.grid_step.max(0.05) * 0.3;
                                    self.preview_segments = vec![
                                        (hit_pos - u * point_size, hit_pos + u * point_size),
                                        (hit_pos - v * point_size, hit_pos + v * point_size),
                                    ];
                                    self.preview_version =
                                        self.preview_version.wrapping_add(1);
                                    self.last_cursor = Some(pos);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                ConeCreateStep::BaseRadius => {
                                    if let Some(center) = self.cone_center {
                                        let radius = (hit_pos - center).length().max(0.05);
                                        self.cone_base_radius = Some(radius);
                                        self.cone_step = Some(ConeCreateStep::TopRadius);

                                        let segments = circle_segments(
                                            config.grid_plane,
                                            center,
                                            radius,
                                            48,
                                        );
                                        self.preview_segments = segments;
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                                ConeCreateStep::TopRadius => {
                                    if let (Some(center), Some(base_radius)) =
                                        (self.cone_center, self.cone_base_radius)
                                    {
                                        let radius = (hit_pos - center).length().max(0.0);
                                        self.cone_top_radius = Some(radius);
                                        self.cone_height = Some(0.0);
                                        self.cone_step = Some(ConeCreateStep::Height);

                                        let dot_radius = config.grid_step.max(0.05) * 0.1;
                                        let mut segments = circle_segments(
                                            config.grid_plane,
                                            center,
                                            base_radius,
                                            48,
                                        );
                                        if radius > 0.0001 {
                                            segments.extend(circle_segments(
                                                config.grid_plane,
                                                center,
                                                radius,
                                                48,
                                            ));
                                        } else {
                                            segments.extend(dot_segments(
                                                config.grid_plane,
                                                center,
                                                dot_radius,
                                            ));
                                        }
                                        self.preview_segments = segments;
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                                ConeCreateStep::Height => {
                                    if let (Some(center), Some(base_radius), Some(top_radius)) = (
                                        self.cone_center,
                                        self.cone_base_radius,
                                        self.cone_top_radius,
                                    ) {
                                        let (_, _, n) = grid_axes(config.grid_plane);
                                        let height = if let Some((origin, dir)) =
                                            ray_from_cursor(pos, scene_bounds, &camera)
                                        {
                                            height_from_cursor(
                                                origin,
                                                dir,
                                                center,
                                                n,
                                                camera.forward,
                                            )
                                        } else {
                                            0.0
                                        };

                                        let h = height.max(0.05);
                                        let size = Vec3::new(
                                            base_radius / 0.4,
                                            h,
                                            top_radius / 0.4,
                                        );
                                        let position = center + n * (h * 0.5);

                                        let id = self.next_id;
                                        self.next_id += 1;
                                        self.entities.push(SceneEntity {
                                            id,
                                            kind: SolidKind::Cone,
                                            position,
                                            size,
                                            plane: config.grid_plane,
                                            mesh: None,
                                        });
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );

                                        self.selected = Some(id);
                                        self.cone_step = None;
                                        self.cone_center = None;
                                        self.cone_base_radius = None;
                                        self.cone_top_radius = None;
                                        self.cone_height = None;
                                        self.preview_segments.clear();
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Torus {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            match self.torus_step.unwrap_or(TorusCreateStep::Center) {
                                TorusCreateStep::Center => {
                                    self.torus_step = Some(TorusCreateStep::OuterRadius);
                                    self.torus_center = Some(hit_pos);
                                    self.torus_outer_radius = None;
                                    self.torus_inner_radius = None;

                                    let (u, v, _) = grid_axes(config.grid_plane);
                                    let point_size = config.grid_step.max(0.05) * 0.3;
                                    self.preview_segments = vec![
                                        (hit_pos - u * point_size, hit_pos + u * point_size),
                                        (hit_pos - v * point_size, hit_pos + v * point_size),
                                    ];
                                    self.preview_version =
                                        self.preview_version.wrapping_add(1);
                                    self.last_cursor = Some(pos);
                                    result.request_redraw = true;
                                    result.capture = true;
                                    result.handled = true;
                                    return result;
                                }
                                TorusCreateStep::OuterRadius => {
                                    if let Some(center) = self.torus_center {
                                        let outer = (hit_pos - center).length().max(0.1);
                                        self.torus_outer_radius = Some(outer);
                                        self.torus_step = Some(TorusCreateStep::InnerRadius);

                                        let segments = circle_segments(
                                            config.grid_plane,
                                            center,
                                            outer,
                                            48,
                                        );
                                        self.preview_segments = segments;
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                                TorusCreateStep::InnerRadius => {
                                    if let (Some(center), Some(outer)) =
                                        (self.torus_center, self.torus_outer_radius)
                                    {
                                        let raw_inner = (hit_pos - center).length();
                                        let inner = clamp_inner_radius(outer, raw_inner, 0.01);
                                        self.torus_inner_radius = Some(inner);

                                        let major = (outer + inner) * 0.5;
                                        let minor = (outer - inner) * 0.5;
                                        let size = Vec3::new(major, major, minor);

                                        let id = self.next_id;
                                        self.next_id += 1;
                                        self.entities.push(SceneEntity {
                                            id,
                                            kind: SolidKind::Torus,
                                            position: center,
                                            size,
                                            plane: config.grid_plane,
                                            mesh: None,
                                        });
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );

                                        self.selected = Some(id);
                                        self.torus_step = None;
                                        self.torus_center = None;
                                        self.torus_outer_radius = None;
                                        self.torus_inner_radius = None;
                                        self.preview_segments.clear();
                                        self.preview_version =
                                            self.preview_version.wrapping_add(1);
                                        self.last_cursor = Some(pos);
                                        result.finished_tool = Some(config.tool);
                                        result.request_redraw = true;
                                        result.capture = true;
                                        result.handled = true;
                                        return result;
                                    }
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Sphere {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            if let Some(center) = self.sphere_center {
                                let radius = (hit_pos - center).length().max(0.05);
                                let id = self.next_id;
                                self.next_id += 1;
                                self.entities.push(SceneEntity {
                                    id,
                                    kind: SolidKind::Sphere,
                                    position: center,
                                    size: Vec3::splat(radius * 2.0),
                                    plane: config.grid_plane,
                                    mesh: None,
                                });
                                self.entities_version = self.entities_version.wrapping_add(1);
                                result.publish_entities = Some(
                                    self.entities.iter().map(SceneEntityInfo::from).collect(),
                                );
                                self.selected = Some(id);
                                self.sphere_center = None;
                                self.preview_segments.clear();
                                self.preview_version = self.preview_version.wrapping_add(1);
                                self.last_cursor = Some(pos);
                                result.finished_tool = Some(config.tool);
                                result.request_redraw = true;
                                result.capture = true;
                                result.handled = true;
                                return result;
                            } else {
                                self.sphere_center = Some(hit_pos);
                                let (u, v, _) = grid_axes(config.grid_plane);
                                let point_size = config.grid_step.max(0.05) * 0.3;
                                self.preview_segments = vec![
                                    (hit_pos - u * point_size, hit_pos + u * point_size),
                                    (hit_pos - v * point_size, hit_pos + v * point_size),
                                ];
                                self.preview_version = self.preview_version.wrapping_add(1);
                                self.dragging = Dragging::None;
                                self.last_cursor = Some(pos);
                                result.request_redraw = true;
                                result.capture = true;
                                result.handled = true;
                                return result;
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Select {
                    let cursor = (pos.x, pos.y);
                    let mvp = build_mvp(&camera);
                    let tol_v = 10.0;
                    let tol_e = 8.0;

                    let mut selection_label: Option<String> = None;
                    let mut selection_segments: Vec<(Vec3, Vec3)> = Vec::new();
                    let mut selection_entity: Option<u64> = None;
                    let mut selection_face: Option<usize> = None;

                    let mut selected_body: Option<u64> = None;

                    let pick_face_segments = |id: u64| -> Option<Vec<(Vec3, Vec3)>> {
                        let Some(entity) = self.entities.iter().find(|e| e.id == id) else {
                            return None;
                        };
                        let (min, max) = entity_aabb(entity);
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(face_idx) = ray_box_face(origin, dir, min, max) {
                                let corners = entity_corners(min, max);
                                return Some(entity_face_edges(&corners, face_idx));
                            }
                        }
                        None
                    };

                    match config.selection_mode {
                        SelectionMode::Vertex => {
                            if let Some((fi, p)) =
                                pick_2d_vertex(cursor, mvp, scene_bounds, &self.sketch_faces, tol_v)
                            {
                                selection_label = Some(format!("Vertex2D {}", fi + 1));
                                selection_segments =
                                    point_cross_segments(config.grid_plane, p, config.grid_step);
                                selection_face = Some(fi);
                            } else if let Some((id, p)) =
                                pick_3d_vertex(cursor, mvp, scene_bounds, &self.entities, tol_v)
                            {
                                selection_label = Some(format!("Vertex {}", id));
                                selection_segments =
                                    vertex_marker_segments(p, config.grid_step.max(0.05) * 0.4);
                                selection_entity = Some(id);
                            }
                        }
                        SelectionMode::Edge => {
                            if let Some((fi, seg)) =
                                pick_2d_edge(cursor, mvp, scene_bounds, &self.sketch_faces, tol_e)
                            {
                                selection_label = Some(format!("Edge2D {}", fi + 1));
                                selection_segments = vec![seg];
                                selection_face = Some(fi);
                            } else if let Some((id, seg)) =
                                pick_3d_edge(cursor, mvp, scene_bounds, &self.entities, tol_e)
                            {
                                selection_label = Some(format!("Edge {}", id));
                                selection_segments = vec![seg];
                                selection_entity = Some(id);
                            }
                        }
                        SelectionMode::Face => {
                            if let Some(fi) =
                                pick_2d_face(cursor, mvp, scene_bounds, &self.sketch_faces)
                            {
                                selection_label = Some(format!("Face2D {}", fi + 1));
                                if let Some(face) = self.sketch_faces.get(fi) {
                                    selection_segments = outline_segments(&face.points);
                                    selection_face = Some(fi);
                                }
                            } else if let Some(id) =
                                pick_entity(pos, scene_bounds, &camera, &self.entities)
                            {
                                if let Some(segments) = pick_face_segments(id) {
                                    selection_label = Some(format!("Face {}", id));
                                    selection_segments = segments;
                                    selection_entity = Some(id);
                                }
                            }
                        }
                        SelectionMode::Body => {
                            if let Some(id) = pick_entity(pos, scene_bounds, &camera, &self.entities)
                            {
                                selected_body = Some(id);
                                selection_label = Some(format!("Body {}", id));
                            }
                        }
                        SelectionMode::Mixed => {
                            if let Some((fi, p)) =
                                pick_2d_vertex(cursor, mvp, scene_bounds, &self.sketch_faces, tol_v)
                            {
                                selection_label = Some(format!("Vertex2D {}", fi + 1));
                                selection_segments =
                                    point_cross_segments(config.grid_plane, p, config.grid_step);
                                selection_face = Some(fi);
                            } else if let Some((id, p)) =
                                pick_3d_vertex(cursor, mvp, scene_bounds, &self.entities, tol_v)
                            {
                                selection_label = Some(format!("Vertex {}", id));
                                selection_segments =
                                    vertex_marker_segments(p, config.grid_step.max(0.05) * 0.4);
                                selection_entity = Some(id);
                            } else if let Some((fi, seg)) =
                                pick_2d_edge(cursor, mvp, scene_bounds, &self.sketch_faces, tol_e)
                            {
                                selection_label = Some(format!("Edge2D {}", fi + 1));
                                selection_segments = vec![seg];
                                selection_face = Some(fi);
                            } else if let Some((id, seg)) =
                                pick_3d_edge(cursor, mvp, scene_bounds, &self.entities, tol_e)
                            {
                                selection_label = Some(format!("Edge {}", id));
                                selection_segments = vec![seg];
                                selection_entity = Some(id);
                            } else if let Some(fi) =
                                pick_2d_face(cursor, mvp, scene_bounds, &self.sketch_faces)
                            {
                                selection_label = Some(format!("Face2D {}", fi + 1));
                                if let Some(face) = self.sketch_faces.get(fi) {
                                    selection_segments = outline_segments(&face.points);
                                    selection_face = Some(fi);
                                }
                            } else if let Some(id) =
                                pick_entity(pos, scene_bounds, &camera, &self.entities)
                            {
                                if let Some(segments) = pick_face_segments(id) {
                                    selection_label = Some(format!("Face {}", id));
                                    selection_segments = segments;
                                    selection_entity = Some(id);
                                } else {
                                    selected_body = Some(id);
                                    selection_label = Some(format!("Body {}", id));
                                }
                            }
                        }
                    }

                    if let Some(id) = selection_entity.or(selected_body) {
                        if self.modifiers.shift {
                            if let Some(pos) = self.selected_entities.iter().position(|v| *v == id)
                            {
                                self.selected_entities.remove(pos);
                            } else {
                                self.selected_entities.push(id);
                            }
                        } else {
                            self.selected_entities.clear();
                            self.selected_entities.push(id);
                        }
                        self.selected = self.selected_entities.last().copied();
                        self.selected_entities_version =
                            self.selected_entities_version.wrapping_add(1);
                    } else {
                        self.selected = None;
                        self.selected_entities.clear();
                        self.selected_entities_version =
                            self.selected_entities_version.wrapping_add(1);
                    }

                    let can_drag = selected_body.is_some()
                        && !self.modifiers.shift
                        && matches!(config.selection_mode, SelectionMode::Body | SelectionMode::Mixed);
                    self.dragging = if can_drag {
                        Dragging::MoveEntity
                    } else {
                        Dragging::None
                    };

                    self.selected_face = selection_face;
                    self.face_highlight_segments = selection_segments;
                    self.face_highlight_version = self.face_highlight_version.wrapping_add(1);
                    result.selection_label = selection_label.or(Some("None".to_string()));
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                } else if let Some(create_kind) = config.tool.create_kind() {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            let id = self.next_id;
                            self.next_id += 1;
                            self.entities.push(SceneEntity {
                                id,
                                kind: create_kind,
                                position: hit_pos,
                                size: Vec3::splat(0.2),
                                plane: config.grid_plane,
                                mesh: None,
                            });
                            self.entities_version = self.entities_version.wrapping_add(1);
                            result.publish_entities = Some(
                                self.entities.iter().map(SceneEntityInfo::from).collect(),
                            );
                            self.selected = Some(id);
                            self.selected_entities.clear();
                            self.selected_entities.push(id);
                            self.selected_entities_version =
                                self.selected_entities_version.wrapping_add(1);
                            self.selected_face = None;
                            self.face_highlight_segments.clear();
                            self.face_highlight_version =
                                self.face_highlight_version.wrapping_add(1);
                            self.dragging = Dragging::CreateEntity;
                            self.drag_start = Some(hit_pos);
                        }
                    }
                }

                if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                    if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                        if let Some(id) = self.selected {
                            if let Some(entity) = self.entities.iter().find(|e| e.id == id) {
                                self.drag_offset = entity.position - hit_pos;
                            }
                        }
                    }
                }

                self.last_cursor = Some(pos);
                result.request_redraw = true;
                result.capture = true;
                result.handled = true;
            }
            SceneInput::MouseDownRight { pos } => {
                if self.modifiers.shift {
                    self.dragging = Dragging::Pan;
                } else {
                    self.dragging = Dragging::Rotate;
                }
                self.last_cursor = Some(pos);
                result.request_redraw = true;
                result.capture = true;
                result.handled = true;
            }
            SceneInput::MouseMove { pos } => {
                if config.tool == SceneTool::Select {
                    let cursor = (pos.x, pos.y);
                    let mvp = build_mvp(&camera);
                    let tol_v = 10.0;
                    let tol_e = 8.0;

                    let mut hover_segments: Vec<(Vec3, Vec3)> = Vec::new();
                    let mut found = false;

                    match config.selection_mode {
                        SelectionMode::Vertex => {
                            if let Some((_, p)) =
                                pick_2d_vertex(cursor, mvp, scene_bounds, &self.sketch_faces, tol_v)
                            {
                                hover_segments =
                                    point_cross_segments(config.grid_plane, p, config.grid_step);
                                found = true;
                            } else if let Some((_, p)) =
                                pick_3d_vertex(cursor, mvp, scene_bounds, &self.entities, tol_v)
                            {
                                hover_segments =
                                    vertex_marker_segments(p, config.grid_step.max(0.05) * 0.4);
                                found = true;
                            }
                        }
                        SelectionMode::Edge => {
                            if let Some((_, seg)) =
                                pick_2d_edge(cursor, mvp, scene_bounds, &self.sketch_faces, tol_e)
                            {
                                hover_segments = vec![seg];
                                found = true;
                            } else if let Some((_, seg)) =
                                pick_3d_edge(cursor, mvp, scene_bounds, &self.entities, tol_e)
                            {
                                hover_segments = vec![seg];
                                found = true;
                            }
                        }
                        SelectionMode::Face => {
                            if let Some(fi) = pick_2d_face(cursor, mvp, scene_bounds, &self.sketch_faces)
                            {
                                if let Some(face) = self.sketch_faces.get(fi) {
                                    hover_segments = outline_segments(&face.points);
                                    found = true;
                                }
                            }
                        }
                        SelectionMode::Body => {
                            if let Some(id) = pick_entity(pos, scene_bounds, &camera, &self.entities)
                            {
                                if let Some(entity) = self.entities.iter().find(|e| e.id == id) {
                                    let (min, max) = entity_aabb(entity);
                                    let corners = entity_corners(min, max);
                                    hover_segments = entity_edges(&corners);
                                    found = true;
                                }
                            }
                        }
                        SelectionMode::Mixed => {
                            if let Some((_, p)) =
                                pick_2d_vertex(cursor, mvp, scene_bounds, &self.sketch_faces, tol_v)
                            {
                                hover_segments =
                                    point_cross_segments(config.grid_plane, p, config.grid_step);
                                found = true;
                            } else if let Some((_, p)) =
                                pick_3d_vertex(cursor, mvp, scene_bounds, &self.entities, tol_v)
                            {
                                hover_segments =
                                    vertex_marker_segments(p, config.grid_step.max(0.05) * 0.4);
                                found = true;
                            } else if let Some((_, seg)) =
                                pick_2d_edge(cursor, mvp, scene_bounds, &self.sketch_faces, tol_e)
                            {
                                hover_segments = vec![seg];
                                found = true;
                            } else if let Some((_, seg)) =
                                pick_3d_edge(cursor, mvp, scene_bounds, &self.entities, tol_e)
                            {
                                hover_segments = vec![seg];
                                found = true;
                            } else if let Some(fi) =
                                pick_2d_face(cursor, mvp, scene_bounds, &self.sketch_faces)
                            {
                                if let Some(face) = self.sketch_faces.get(fi) {
                                    hover_segments = outline_segments(&face.points);
                                    found = true;
                                }
                            } else if let Some(id) =
                                pick_entity(pos, scene_bounds, &camera, &self.entities)
                            {
                                if let Some(entity) = self.entities.iter().find(|e| e.id == id) {
                                    let (min, max) = entity_aabb(entity);
                                    let corners = entity_corners(min, max);
                                    hover_segments = entity_edges(&corners);
                                    found = true;
                                }
                            }
                        }
                    }

                    if found {
                        self.hover_segments = hover_segments;
                    } else if !self.hover_segments.is_empty() {
                        self.hover_segments.clear();
                    }
                    self.hover_version = self.hover_version.wrapping_add(1);
                    result.request_redraw = true;
                }

                if self.sketch_active && config.tool.is_sketch() {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            self.sketch_current = Some(hit_pos);
                            match config.tool {
                                SceneTool::SketchLine => {
                                    self.preview_segments = polyline_preview_segments(
                                        &self.sketch_points,
                                        Some(hit_pos),
                                    );
                                }
                                SceneTool::SketchArc => {
                                    if self.sketch_points.len() == 1 {
                                        let center = self.sketch_points[0];
                                        let dash = config.grid_step.max(0.05) * 0.6;
                                        self.preview_segments =
                                            dashed_segments(center, hit_pos, dash, dash * 0.7);
                                    } else if self.sketch_points.len() >= 2 {
                                        let center = self.sketch_points[0];
                                        let start = self.sketch_points[1];
                                        let dash = config.grid_step.max(0.05) * 0.6;
                                        let mut segments =
                                            arc_segments(config.grid_plane, center, start, hit_pos, 36);
                                        segments.extend(dashed_segments(center, hit_pos, dash, dash * 0.7));
                                        self.preview_segments = segments;
                                    }
                                }
                                _ => {
                                    if let Some(start) = self.sketch_start {
                                        self.preview_segments = sketch_segments_for_tool(
                                            config.tool,
                                            config.grid_plane,
                                            config.grid_step,
                                            start,
                                            hit_pos,
                                        );
                                    }
                                }
                            }

                            self.preview_version = self.preview_version.wrapping_add(1);
                            result.request_redraw = true;
                            result.capture = true;
                            result.handled = true;
                            return result;
                        }
                    }
                }

                if config.tool == SceneTool::Box {
                    if let Some(step) = self.box_step {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) =
                                intersect_plane(config.grid_plane, origin, dir)
                            {
                                match step {
                                    BoxCreateStep::BaseSize => {
                                        if let Some(start) = self.box_base_start {
                                            let corners = rect_corners(config.grid_plane, start, hit_pos);
                                            let mut segments = line_segments_for_rect(corners);
                                            let (u, v, _) = grid_axes(config.grid_plane);
                                            let point_size = config.grid_step.max(0.05) * 0.3;
                                            segments.push((start - u * point_size, start + u * point_size));
                                            segments.push((start - v * point_size, start + v * point_size));
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    BoxCreateStep::Height => {
                                        if let (Some(start), Some(end)) =
                                            (self.box_base_start, self.box_base_end)
                                        {
                                            let (_, base_center) =
                                                base_size_and_center(config.grid_plane, start, end);
                                            let (_, _, n) = grid_axes(config.grid_plane);
                                            let height = height_from_cursor(
                                                origin,
                                                dir,
                                                base_center,
                                                n,
                                                camera.forward,
                                            );
                                            let segments =
                                                box_frame_segments(config.grid_plane, start, end, height);
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    BoxCreateStep::BaseStart => {}
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Cone {
                    if let Some(step) = self.cone_step {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) =
                                intersect_plane(config.grid_plane, origin, dir)
                            {
                                match step {
                                    ConeCreateStep::BaseRadius => {
                                        if let Some(center) = self.cone_center {
                                            let radius = (hit_pos - center).length().max(0.05);
                                            let segments = circle_segments(
                                                config.grid_plane,
                                                center,
                                                radius,
                                                48,
                                            );
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    ConeCreateStep::TopRadius => {
                                        if let (Some(center), Some(base_radius)) =
                                            (self.cone_center, self.cone_base_radius)
                                        {
                                            let radius = (hit_pos - center).length().max(0.0);
                                            let dot_radius = config.grid_step.max(0.05) * 0.1;
                                            let mut segments = circle_segments(
                                                config.grid_plane,
                                                center,
                                                base_radius,
                                                48,
                                            );
                                            if radius > 0.0001 {
                                                segments.extend(circle_segments(
                                                    config.grid_plane,
                                                    center,
                                                    radius,
                                                    48,
                                                ));
                                            } else {
                                                segments.extend(dot_segments(
                                                    config.grid_plane,
                                                    center,
                                                    dot_radius,
                                                ));
                                            }
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    ConeCreateStep::Height => {
                                        if let (Some(center), Some(base_radius), Some(top_radius)) = (
                                            self.cone_center,
                                            self.cone_base_radius,
                                            self.cone_top_radius,
                                        ) {
                                            let (_, _, n) = grid_axes(config.grid_plane);
                                            let height = height_from_cursor(
                                                origin,
                                                dir,
                                                center,
                                                n,
                                                camera.forward,
                                            );
                                            let dot_radius = config.grid_step.max(0.05) * 0.1;
                                            let segments = cone_frame_segments(
                                                config.grid_plane,
                                                center,
                                                base_radius,
                                                top_radius,
                                                height,
                                                48,
                                                dot_radius,
                                            );
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    ConeCreateStep::BaseCenter => {}
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Torus {
                    if let Some(step) = self.torus_step {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) =
                                intersect_plane(config.grid_plane, origin, dir)
                            {
                                match step {
                                    TorusCreateStep::OuterRadius => {
                                        if let Some(center) = self.torus_center {
                                            let outer = (hit_pos - center).length().max(0.1);
                                            let segments = circle_segments(
                                                config.grid_plane,
                                                center,
                                                outer,
                                                48,
                                            );
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    TorusCreateStep::InnerRadius => {
                                        if let (Some(center), Some(outer)) =
                                            (self.torus_center, self.torus_outer_radius)
                                        {
                                            let raw_inner = (hit_pos - center).length();
                                            let inner = clamp_inner_radius(outer, raw_inner, 0.01);
                                            let mut segments = circle_segments(
                                                config.grid_plane,
                                                center,
                                                outer,
                                                48,
                                            );
                                            segments.extend(circle_segments(
                                                config.grid_plane,
                                                center,
                                                inner.max(0.01),
                                                48,
                                            ));
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    TorusCreateStep::Center => {}
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Cylinder {
                    if let Some(step) = self.cylinder_step {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) =
                                intersect_plane(config.grid_plane, origin, dir)
                            {
                                match step {
                                    CylinderCreateStep::BaseRadius => {
                                        if let Some(center) = self.cylinder_center {
                                            let radius = (hit_pos - center).length().max(0.05);
                                            let segments = circle_segments(
                                                config.grid_plane,
                                                center,
                                                radius,
                                                48,
                                            );
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    CylinderCreateStep::Height => {
                                        if let (Some(center), Some(radius)) =
                                            (self.cylinder_center, self.cylinder_radius)
                                        {
                                            let (_, _, n) = grid_axes(config.grid_plane);
                                            let height = height_from_cursor(
                                                origin,
                                                dir,
                                                center,
                                                n,
                                                camera.forward,
                                            );
                                            let segments = cylinder_frame_segments(
                                                config.grid_plane,
                                                center,
                                                radius,
                                                height,
                                                48,
                                            );
                                            self.preview_segments = segments;
                                            self.preview_version =
                                                self.preview_version.wrapping_add(1);
                                            self.last_cursor = Some(pos);
                                            result.request_redraw = true;
                                            result.capture = true;
                                            result.handled = true;
                                            return result;
                                        }
                                    }
                                    CylinderCreateStep::BaseCenter => {}
                                }
                            }
                        }
                    }
                }

                if config.tool == SceneTool::Sphere {
                    if let Some(center) = self.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                                let radius = (hit_pos - center).length().max(0.05);
                                let (u, v, _) = grid_axes(config.grid_plane);
                                let point_size = config.grid_step.max(0.05) * 0.3;
                                let mut segments = circle_segments(
                                    config.grid_plane,
                                    center,
                                    radius,
                                    48,
                                );
                                segments.push((center - u * point_size, center + u * point_size));
                                segments.push((center - v * point_size, center + v * point_size));
                                self.preview_segments = segments;
                                self.preview_version = self.preview_version.wrapping_add(1);
                                self.last_cursor = Some(pos);
                                result.request_redraw = true;
                                result.capture = true;
                                result.handled = true;
                                return result;
                            }
                        }
                    }
                }

                match self.dragging {
                    Dragging::None => {}
                    Dragging::MoveEntity => {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                                if let Some(id) = self.selected {
                                    if let Some(entity) =
                                        self.entities.iter_mut().find(|e| e.id == id)
                                    {
                                        entity.position = hit_pos + self.drag_offset;
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );
                                    }
                                }
                            }
                        }

                        self.last_cursor = Some(pos);
                        result.request_redraw = true;
                        result.capture = true;
                        result.handled = true;
                        return result;
                    }
                    Dragging::CreateEntity => {
                        if let (Some(start), Some((origin, dir))) =
                            (self.drag_start, ray_from_cursor(pos, scene_bounds, &camera))
                        {
                            if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                                if let Some(id) = self.selected {
                                    if let Some(entity) =
                                        self.entities.iter_mut().find(|e| e.id == id)
                                    {
                                        let diff = hit_pos - start;
                                        let dist = diff.length().max(0.1);
                                        entity.size = Vec3::splat(dist);
                                        self.entities_version =
                                            self.entities_version.wrapping_add(1);
                                        result.publish_entities = Some(
                                            self.entities.iter().map(SceneEntityInfo::from).collect(),
                                        );
                                    }
                                }
                            }
                        }

                        self.last_cursor = Some(pos);
                        result.request_redraw = true;
                        result.capture = true;
                        result.handled = true;
                        return result;
                    }
                    Dragging::Pan => {
                        if let Some(last) = self.last_cursor {
                            let dx = pos.x - last.x;
                            let dy = pos.y - last.y;

                            if bounds.width > 1.0 && bounds.height > 1.0 {
                                let dx_ndc = (dx * 2.0) / bounds.width;
                                let dy_ndc = (-dy * 2.0) / bounds.height;

                                let half_h = camera.ortho_half_h;
                                let half_w = half_h * camera.aspect;

                                let pan = camera.right * (dx_ndc * half_w)
                                    + camera.up * (dy_ndc * half_h);
                                self.target += pan;
                            }

                            self.last_cursor = Some(pos);
                            result.request_redraw = true;
                            result.capture = true;
                            result.handled = true;
                            return result;
                        } else {
                            self.last_cursor = Some(pos);
                            result.request_redraw = true;
                            result.capture = true;
                            result.handled = true;
                            return result;
                        }
                    }
                    Dragging::Rotate => {
                        if let Some(last) = self.last_cursor {
                            let dx = pos.x - last.x;
                            let dy = pos.y - last.y;

                            let rot_speed = 2.5;
                            if bounds.width > 1.0 && bounds.height > 1.0 {
                                self.yaw += (dx / bounds.width) * rot_speed;
                                self.pitch += (dy / bounds.height) * rot_speed;
                            } else {
                                self.yaw += dx * 0.01;
                                self.pitch += dy * 0.01;
                            }

                            let max_pitch = 1.55;
                            self.pitch = self.pitch.clamp(-max_pitch, max_pitch);

                            self.last_cursor = Some(pos);
                            result.request_redraw = true;
                            result.capture = true;
                            result.handled = true;
                            return result;
                        } else {
                            self.last_cursor = Some(pos);
                            result.request_redraw = true;
                            result.capture = true;
                            result.handled = true;
                            return result;
                        }
                    }
                }
            }
            _ => {}
        }

        result
    }
}