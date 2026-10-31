//! 平面の姿勢と速度

use std::f64::consts::PI;

/// 平面の姿勢。位置 [m] と向き [rad]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose2D {
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// このロボット座標系の点 `local` を世界座標系に直す
    pub fn compose(&self, local: &Pose2D) -> Pose2D {
        let (s, c) = self.theta.sin_cos();
        Pose2D {
            x: self.x + c * local.x - s * local.y,
            y: self.y + s * local.x + c * local.y,
            theta: normalize_angle(self.theta + local.theta),
        }
    }
}

/// ロボット座標系の速度。並進 [m/s] と回転 [rad/s]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Twist2D {
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
}

impl Twist2D {
    pub fn new(vx: f64, vy: f64, omega: f64) -> Self {
        Self { vx, vy, omega }
    }

    pub const ZERO: Twist2D = Twist2D {
        vx: 0.0,
        vy: 0.0,
        omega: 0.0,
    };
}

/// 角度を (-π, π] に収める
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a <= -PI {
        a += 2.0 * PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalize_angle_wraps() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(0.5) - 0.5).abs() < 1e-12);
        assert!((normalize_angle(-2.5 * PI) + 0.5 * PI).abs() < 1e-12);
    }

    #[test]
    fn compose_rotates_offset() {
        let base = Pose2D::new(1.0, 2.0, PI / 2.0);
        let p = base.compose(&Pose2D::new(1.0, 0.0, 0.0));
        assert!((p.x - 1.0).abs() < 1e-12);
        assert!((p.y - 3.0).abs() < 1e-12);
        assert!((p.theta - PI / 2.0).abs() < 1e-12);
    }
}
