//! 車両モデル
//!
//! juiz の examples/python/turtle_sim_py/turtle_py.py の Turtle を移したもの。
//! 目標速度へ加速度の上限で近づけ、速度の上限で切り、中点の向きで積分する。

use crate::error::{SimError, SimResult};
use crate::geometry::{normalize_angle, Pose2D, Twist2D};

/// 駆動方式
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DriveKind {
    /// 対向 2 輪。横方向の速度 (vy) は無視する
    #[default]
    DiffDrive,
    /// 全方向移動
    Omni,
}

/// 車両の設定
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleConfig {
    pub drive: DriveKind,
    /// ロボットを表す円の半径 [m]
    pub radius: f64,
    /// 速度の上限(各成分の絶対値)[m/s, m/s, rad/s]
    pub max_velocity: Twist2D,
    /// 加速度の上限(各成分の絶対値)[m/s², m/s², rad/s²]
    pub max_acceleration: Twist2D,
    /// この時間 [s] 速度指令が来なければ目標速度を 0 にする。0 以下なら止めない
    pub command_timeout: f64,
}

impl Default for VehicleConfig {
    fn default() -> Self {
        // 速度・加速度は turtle_sim と同じ
        Self {
            drive: DriveKind::DiffDrive,
            radius: 0.2,
            max_velocity: Twist2D::new(1.0, 1.0, 0.5),
            max_acceleration: Twist2D::new(0.1, 0.1, 0.05),
            command_timeout: 0.5,
        }
    }
}

impl VehicleConfig {
    pub fn validate(&self) -> SimResult<()> {
        let all = [
            self.max_velocity.vx,
            self.max_velocity.vy,
            self.max_velocity.omega,
            self.max_acceleration.vx,
            self.max_acceleration.vy,
            self.max_acceleration.omega,
        ];
        if all.iter().any(|v| !(v.is_finite() && *v >= 0.0)) {
            return Err(SimError::InvalidConfig(
                "max_velocity and max_acceleration must be finite and non-negative".to_owned(),
            ));
        }
        if !(self.radius.is_finite() && self.radius >= 0.0) {
            return Err(SimError::InvalidConfig(format!(
                "radius must be non-negative, got {}",
                self.radius
            )));
        }
        Ok(())
    }
}

/// 1 台の車両の状態
#[derive(Clone, Debug)]
pub struct Vehicle {
    pub config: VehicleConfig,
    pub pose: Pose2D,
    pub velocity: Twist2D,
    target: Twist2D,
    command_age: f64,
}

impl Vehicle {
    pub fn new(config: VehicleConfig, pose: Pose2D) -> Self {
        Self {
            config,
            pose,
            velocity: Twist2D::ZERO,
            target: Twist2D::ZERO,
            command_age: 0.0,
        }
    }

    /// 目標速度。上限で切り、対向 2 輪なら vy を 0 にする
    pub fn set_target_velocity(&mut self, cmd: Twist2D) {
        let m = &self.config.max_velocity;
        let vy = match self.config.drive {
            DriveKind::DiffDrive => 0.0,
            DriveKind::Omni => clamp_abs(cmd.vy, m.vy),
        };
        self.target = Twist2D::new(clamp_abs(cmd.vx, m.vx), vy, clamp_abs(cmd.omega, m.omega));
        self.command_age = 0.0;
    }

    pub fn target_velocity(&self) -> Twist2D {
        self.target
    }

    /// 姿勢を置き直し、止める
    pub fn reset(&mut self, pose: Pose2D) {
        self.pose = Pose2D {
            theta: normalize_angle(pose.theta),
            ..pose
        };
        self.velocity = Twist2D::ZERO;
        self.target = Twist2D::ZERO;
        self.command_age = 0.0;
    }

    /// 止める(衝突したとき)。目標速度はそのまま
    pub fn stop(&mut self) {
        self.velocity = Twist2D::ZERO;
    }

    /// `dt` [s] 進めた速度を求めて持ち、その速度で動いた後の姿勢を返す(姿勢はまだ変えない)
    pub fn advance(&mut self, dt: f64) -> Pose2D {
        if dt <= 0.0 {
            return self.pose;
        }
        // 指令を受けた後の最初の step は必ずその指令で動かす(dt が timeout より長くても)
        if self.config.command_timeout > 0.0 && self.command_age >= self.config.command_timeout {
            self.target = Twist2D::ZERO;
        }
        self.command_age += dt;
        let a = &self.config.max_acceleration;
        self.velocity = Twist2D::new(
            approach(self.velocity.vx, self.target.vx, a.vx * dt),
            approach(self.velocity.vy, self.target.vy, a.vy * dt),
            approach(self.velocity.omega, self.target.omega, a.omega * dt),
        );
        integrate(&self.pose, &self.velocity, dt)
    }
}

/// ロボット座標系の速度 `v` で `dt` 動いた後の姿勢(中点の向きで積分する)
pub fn integrate(pose: &Pose2D, v: &Twist2D, dt: f64) -> Pose2D {
    let dth = v.omega * dt;
    let (s, c) = (pose.theta + dth / 2.0).sin_cos();
    let (dx, dy) = (v.vx * dt, v.vy * dt);
    Pose2D {
        x: pose.x + dx * c - dy * s,
        y: pose.y + dx * s + dy * c,
        theta: normalize_angle(pose.theta + dth),
    }
}

fn clamp_abs(v: f64, max: f64) -> f64 {
    v.clamp(-max, max)
}

/// `current` を `target` へ最大 `step` だけ近づける
fn approach(current: f64, target: f64, step: f64) -> f64 {
    let d = target - current;
    if d.abs() <= step {
        target
    } else {
        current + step.copysign(d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn omni() -> VehicleConfig {
        VehicleConfig {
            drive: DriveKind::Omni,
            ..Default::default()
        }
    }

    #[test]
    fn accelerates_with_limit_and_clamps_velocity() {
        let mut v = Vehicle::new(VehicleConfig::default(), Pose2D::default());
        v.set_target_velocity(Twist2D::new(5.0, 0.0, -5.0));
        assert_eq!(v.target_velocity(), Twist2D::new(1.0, 0.0, -0.5));
        v.advance(1.0);
        assert!((v.velocity.vx - 0.1).abs() < 1e-12);
        assert!((v.velocity.omega + 0.05).abs() < 1e-12);
        for _ in 0..30 {
            v.set_target_velocity(Twist2D::new(5.0, 0.0, -5.0));
            v.advance(0.4);
        }
        assert_eq!(v.velocity, Twist2D::new(1.0, 0.0, -0.5));
    }

    #[test]
    fn diff_drive_ignores_lateral_velocity() {
        let mut v = Vehicle::new(VehicleConfig::default(), Pose2D::default());
        v.set_target_velocity(Twist2D::new(0.0, 1.0, 0.0));
        assert_eq!(v.target_velocity().vy, 0.0);

        let mut o = Vehicle::new(omni(), Pose2D::default());
        o.set_target_velocity(Twist2D::new(0.0, 1.0, 0.0));
        assert_eq!(o.target_velocity().vy, 1.0);
    }

    #[test]
    fn command_times_out() {
        let mut v = Vehicle::new(VehicleConfig::default(), Pose2D::default());
        v.set_target_velocity(Twist2D::new(1.0, 0.0, 0.0));
        v.advance(0.3);
        v.advance(0.3);
        assert_eq!(v.target_velocity().vx, 1.0);
        v.advance(0.3);
        assert_eq!(v.target_velocity().vx, 0.0);

        let mut never = Vehicle::new(
            VehicleConfig {
                command_timeout: 0.0,
                ..Default::default()
            },
            Pose2D::default(),
        );
        never.set_target_velocity(Twist2D::new(1.0, 0.0, 0.0));
        never.advance(100.0);
        assert_eq!(never.target_velocity().vx, 1.0);
    }

    #[test]
    fn integrate_in_robot_frame() {
        let p = integrate(
            &Pose2D::new(0.0, 0.0, std::f64::consts::FRAC_PI_2),
            &Twist2D::new(1.0, 0.0, 0.0),
            2.0,
        );
        assert!(p.x.abs() < 1e-12 && (p.y - 2.0).abs() < 1e-12);

        let p = integrate(&Pose2D::default(), &Twist2D::new(0.0, 1.0, 0.0), 1.0);
        assert!(p.x.abs() < 1e-12 && (p.y - 1.0).abs() < 1e-12);

        // 一定の速度で 1 周すると元に戻る
        let mut pose = Pose2D::default();
        let v = Twist2D::new(1.0, 0.0, 1.0);
        let n = 10_000;
        let dt = 2.0 * std::f64::consts::PI / n as f64;
        for _ in 0..n {
            pose = integrate(&pose, &v, dt);
        }
        assert!(pose.x.abs() < 1e-6 && pose.y.abs() < 1e-6);
    }
}
