#![allow(dead_code)]

use glam::{Vec3};
use iced::{Point, Rectangle};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    Perspective,
    Orthographic,
}

impl CameraMode {
    pub const ALL: [CameraMode; 2] = [CameraMode::Orthographic, CameraMode::Perspective];

    pub fn label(self) -> &'static str {
        match self {
            CameraMode::Orthographic => "Orthographic",
            CameraMode::Perspective => "Perspective",
        }
    }
}

impl std::fmt::Display for CameraMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub eye: Vec3,
    pub forward: Vec3,
    pub right: Vec3,
    pub up: Vec3,
    pub aspect: f32,
    pub fovy: f32,
    pub mode: CameraMode,
    pub ortho_half_h: f32,
    pub near: f32,
    pub far: f32,
}

pub fn camera_from_params(
    target: Vec3,
    distance: f32,
    yaw: f32,
    pitch: f32,
    bounds: Rectangle,
    mode: CameraMode,
) -> Camera {
    let aspect = if bounds.height > 1.0 {
        bounds.width / bounds.height
    } else {
        1.0
    };

    let fovy = 45.0_f32.to_radians();
    let near = 0.01_f32;
    let far = 1000.0_f32;
    let distance = distance.clamp(0.1, 200.0);
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let offset = Vec3::new(distance * cp * sy, distance * sp, distance * cp * cy);
    let eye = target + offset;
    let forward = (target - eye).normalize_or_zero();
    let world_up = Vec3::Y;
    let right = forward.cross(world_up).normalize_or_zero();
    let up = right.cross(forward);
    let ortho_half_h = match mode {
        CameraMode::Perspective => (0.5 * fovy).tan() * distance,
        CameraMode::Orthographic => distance.max(0.1),
    };

    Camera {
        eye,
        forward,
        right,
        up,
        aspect,
        fovy,
        mode,
        ortho_half_h,
        near,
        far,
    }
}

pub fn ray_from_cursor(cursor: Point, bounds: Rectangle, camera: &Camera) -> Option<(Vec3, Vec3)> {
    if bounds.width <= 1.0 || bounds.height <= 1.0 {
        return None;
    }

    let x_ndc = (cursor.x / bounds.width) * 2.0 - 1.0;
    let y_ndc = 1.0 - (cursor.y / bounds.height) * 2.0;

    match camera.mode {
        CameraMode::Perspective => {
            let half_h = (0.5 * camera.fovy).tan();
            let half_w = half_h * camera.aspect;

            let dir = (camera.forward
                + camera.right * (x_ndc * half_w)
                + camera.up * (y_ndc * half_h))
                .normalize_or_zero();

            Some((camera.eye, dir))
        }
        CameraMode::Orthographic => {
            let half_h = camera.ortho_half_h;
            let half_w = half_h * camera.aspect;
            let origin = camera.eye
                + camera.right * (x_ndc * half_w)
                + camera.up * (y_ndc * half_h);
            Some((origin, camera.forward))
        }
    }
}
