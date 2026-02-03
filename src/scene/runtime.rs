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
    preview_line: Option<(Vec3, Vec3)>,
    preview_version: u64,
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
            preview_line: None,
            preview_version: 0,
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
    pub preview_line: Option<(Vec3, Vec3)>,
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
            preview_line: self.preview_line,
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
                if config.tool == SceneTool::Sphere {
                    if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                        if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                            self.sphere_center = Some(hit_pos);
                            self.preview_line = Some((hit_pos, hit_pos));
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
                                size: 0.2,
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
                if config.tool == SceneTool::Sphere {
                    if let Some(center) = self.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                                let radius = (hit_pos - center).length().max(0.05);
                                let id = self.next_id;
                                self.next_id += 1;
                                self.entities.push(SceneEntity {
                                    id,
                                    kind: SolidKind::Sphere,
                                    position: center,
                                    size: radius * 2.0,
                                });
                                self.entities_version = self.entities_version.wrapping_add(1);
                                result.publish_entities = Some(
                                    self.entities.iter().map(SceneEntityInfo::from).collect(),
                                );
                                self.selected = Some(id);
                                self.sphere_center = None;
                                self.preview_line = None;
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
                if config.tool == SceneTool::Sphere {
                    if let Some(center) = self.sphere_center {
                        if let Some((origin, dir)) = ray_from_cursor(pos, scene_bounds, &camera) {
                            if let Some(hit_pos) = intersect_plane(config.grid_plane, origin, dir) {
                                self.preview_line = Some((center, hit_pos));
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
                                        entity.size = dist;
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