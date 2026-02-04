use glam::Vec3;

use crate::scene::{
    camera_from_params, intersect_plane, pick_entity, ray_from_cursor, preset_angles, CameraMode,
    CameraPreset, GridPlane, SceneEntity, SceneEntityInfo, ScenePoint, SceneRect, SceneTool,
    SolidKind,
};

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
    next_id: u64,
    drag_start: Option<Vec3>,
    drag_offset: Vec3,
    modifiers: SceneModifiers,
    sphere_center: Option<Vec3>,
    preview_segments: Vec<(Vec3, Vec3)>,
    preview_version: u64,
    box_step: Option<BoxCreateStep>,
    box_base_start: Option<Vec3>,
    box_base_end: Option<Vec3>,
    box_base_size: Option<Vec3>,
    box_height: Option<f32>,
    cylinder_step: Option<CylinderCreateStep>,
    cylinder_center: Option<Vec3>,
    cylinder_radius: Option<f32>,
    cylinder_height: Option<f32>,
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
            next_id: 1,
            drag_start: None,
            drag_offset: Vec3::ZERO,
            modifiers: SceneModifiers::default(),
            sphere_center: None,
            preview_segments: Vec::new(),
            preview_version: 0,
            box_step: None,
            box_base_start: None,
            box_base_end: None,
            box_base_size: None,
            box_height: None,
            cylinder_step: None,
            cylinder_center: None,
            cylinder_radius: None,
            cylinder_height: None,
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
    pub selected: Option<u64>,
    pub axes_enabled: bool,
    pub axes_size: f32,
    pub axes_margin: f32,
    pub background: [f32; 4],
}

impl SceneModel {
    pub fn is_dragging(&self) -> bool {
        self.dragging != Dragging::None
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
            selected: self.selected,
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
                if self.box_step.is_some()
                    || self.sphere_center.is_some()
                    || self.cylinder_step.is_some()
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
                    self.preview_segments.clear();
                    self.preview_version = self.preview_version.wrapping_add(1);
                    result.request_redraw = true;
                    result.capture = true;
                    result.handled = true;
                }
                return result;
            }
            SceneInput::KeyDelete => {
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

                let hit = pick_entity(pos, scene_bounds, &camera, &self.entities);
                if let Some(id) = hit {
                    self.selected = Some(id);
                    self.dragging = Dragging::MoveEntity;
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
                            });
                            self.entities_version = self.entities_version.wrapping_add(1);
                            result.publish_entities = Some(
                                self.entities.iter().map(SceneEntityInfo::from).collect(),
                            );
                            self.selected = Some(id);
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