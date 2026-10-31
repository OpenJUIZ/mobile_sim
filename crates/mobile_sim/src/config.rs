//! conf のコンテナの引数(JSON)から、mobile_sim_core の設定を作る
//!
//! ```yaml
//! containers:
//!   - type_name: mobile_sim
//!     name: sim0
//!     map: assets/maps/default.yaml   # mapconf のパス
//!     seed: 0                         # LiDAR のノイズの乱数の種
//!     step: { mode: fixed, dt: 0.01 } # または { mode: real_time, max_dt: 0.1 }
//!     robots:
//!       - drive: diff_drive           # または omni
//!         radius: 0.2
//!         initial_pose: { x: 0.0, y: 0.0, theta: 0.0 }
//!         lidar: { num_beams: 200, range_max: 10.0 }
//! ```
//! 書かない項目は既定値(mobile_sim_core の Default)になる。

use juiz_sdk::serde_json;
use mobile_sim_core::{
    DriveKind, LidarConfig, Pose2D, RobotConfig, StepClock, Twist2D, VehicleConfig,
};
use serde::Deserialize;

/// `step` の項目
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum StepSetting {
    /// 毎回同じ dt [s]
    Fixed {
        #[serde(default = "default_dt")]
        dt: f64,
    },
    /// 実時間。dt の上限 [s]
    RealTime {
        #[serde(default = "default_max_dt")]
        max_dt: f64,
    },
}

fn default_dt() -> f64 {
    0.01
}

fn default_max_dt() -> f64 {
    0.1
}

impl Default for StepSetting {
    fn default() -> Self {
        StepSetting::RealTime {
            max_dt: default_max_dt(),
        }
    }
}

impl StepSetting {
    pub fn clock(&self) -> Result<StepClock, String> {
        match *self {
            StepSetting::Fixed { dt } if dt > 0.0 && dt.is_finite() => Ok(StepClock::Fixed(dt)),
            StepSetting::RealTime { max_dt } if max_dt > 0.0 && max_dt.is_finite() => {
                Ok(StepClock::real_time(max_dt))
            }
            _ => Err(format!("step dt must be positive: {self:?}")),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseSetting {
    #[serde(default)]
    pub x: f64,
    #[serde(default)]
    pub y: f64,
    #[serde(default)]
    pub theta: f64,
}

impl From<PoseSetting> for Pose2D {
    fn from(p: PoseSetting) -> Self {
        Pose2D::new(p.x, p.y, p.theta)
    }
}

/// 速度・加速度の上限 [m/s, m/s, rad/s]
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LimitSetting {
    pub vx: f64,
    pub vy: f64,
    pub omega: f64,
}

impl From<LimitSetting> for Twist2D {
    fn from(l: LimitSetting) -> Self {
        Twist2D::new(l.vx, l.vy, l.omega)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DriveSetting {
    #[default]
    DiffDrive,
    Omni,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarSetting {
    pub angle_min: Option<f64>,
    pub angle_max: Option<f64>,
    pub num_beams: Option<usize>,
    pub range_min: Option<f64>,
    pub range_max: Option<f64>,
    pub mount: Option<PoseSetting>,
    pub noise_stddev: Option<f64>,
    pub unknown_is_obstacle: Option<bool>,
}

impl From<&LidarSetting> for LidarConfig {
    fn from(s: &LidarSetting) -> Self {
        let d = LidarConfig::default();
        LidarConfig {
            angle_min: s.angle_min.unwrap_or(d.angle_min),
            angle_max: s.angle_max.unwrap_or(d.angle_max),
            num_beams: s.num_beams.unwrap_or(d.num_beams),
            range_min: s.range_min.unwrap_or(d.range_min),
            range_max: s.range_max.unwrap_or(d.range_max),
            mount: s.mount.map(Into::into).unwrap_or(d.mount),
            noise_stddev: s.noise_stddev.unwrap_or(d.noise_stddev),
            unknown_is_obstacle: s.unknown_is_obstacle.unwrap_or(d.unknown_is_obstacle),
        }
    }
}

/// `robots` の 1 要素
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotSetting {
    #[serde(default)]
    pub drive: DriveSetting,
    pub radius: Option<f64>,
    pub max_velocity: Option<LimitSetting>,
    pub max_acceleration: Option<LimitSetting>,
    pub command_timeout: Option<f64>,
    #[serde(default)]
    pub initial_pose: PoseSetting,
    #[serde(default)]
    pub lidar: LidarSetting,
}

impl From<&RobotSetting> for RobotConfig {
    fn from(s: &RobotSetting) -> Self {
        let d = VehicleConfig::default();
        RobotConfig {
            vehicle: VehicleConfig {
                drive: match s.drive {
                    DriveSetting::DiffDrive => DriveKind::DiffDrive,
                    DriveSetting::Omni => DriveKind::Omni,
                },
                radius: s.radius.unwrap_or(d.radius),
                max_velocity: s.max_velocity.map(Into::into).unwrap_or(d.max_velocity),
                max_acceleration: s
                    .max_acceleration
                    .map(Into::into)
                    .unwrap_or(d.max_acceleration),
                command_timeout: s.command_timeout.unwrap_or(d.command_timeout),
            },
            lidar: (&s.lidar).into(),
            initial_pose: s.initial_pose.into(),
        }
    }
}

/// `robots` を読む。省けば既定のロボット 1 台
pub fn parse_robots(v: Option<&serde_json::Value>) -> Result<Vec<RobotSetting>, String> {
    match v {
        None | Some(serde_json::Value::Null) => Ok(vec![RobotSetting::default()]),
        Some(v) => {
            let robots: Vec<RobotSetting> =
                serde_json::from_value(v.clone()).map_err(|e| format!("invalid robots: {e}"))?;
            if robots.is_empty() {
                return Err("robots must have at least one robot".to_owned());
            }
            Ok(robots)
        }
    }
}

/// `step` を読む。省けば実時間
pub fn parse_step(v: Option<&serde_json::Value>) -> Result<StepSetting, String> {
    match v {
        None | Some(serde_json::Value::Null) => Ok(StepSetting::default()),
        Some(v) => serde_json::from_value(v.clone()).map_err(|e| format!("invalid step: {e}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn defaults_when_omitted() {
        let robots = parse_robots(None).unwrap();
        assert_eq!(robots.len(), 1);
        let c: RobotConfig = (&robots[0]).into();
        assert_eq!(c, RobotConfig::default());
        assert_eq!(parse_step(None).unwrap(), StepSetting::default());
    }

    #[test]
    fn reads_full_robot() {
        let v = json!([{
            "drive": "omni",
            "radius": 0.3,
            "max_velocity": {"vx": 2.0, "vy": 1.0, "omega": 1.5},
            "command_timeout": 0.0,
            "initial_pose": {"x": 1.0, "theta": 0.5},
            "lidar": {"num_beams": 360, "range_max": 20.0, "mount": {"x": 0.1}}
        }, {}]);
        let robots = parse_robots(Some(&v)).unwrap();
        assert_eq!(robots.len(), 2);
        let c: RobotConfig = (&robots[0]).into();
        assert_eq!(c.vehicle.drive, DriveKind::Omni);
        assert_eq!(c.vehicle.radius, 0.3);
        assert_eq!(c.vehicle.max_velocity, Twist2D::new(2.0, 1.0, 1.5));
        assert_eq!(
            c.vehicle.max_acceleration,
            VehicleConfig::default().max_acceleration
        );
        assert_eq!(c.vehicle.command_timeout, 0.0);
        assert_eq!(c.initial_pose, Pose2D::new(1.0, 0.0, 0.5));
        assert_eq!(c.lidar.num_beams, 360);
        assert_eq!(c.lidar.range_max, 20.0);
        assert_eq!(c.lidar.mount, Pose2D::new(0.1, 0.0, 0.0));
        assert_eq!(c.lidar.angle_min, LidarConfig::default().angle_min);
    }

    #[test]
    fn rejects_typos_and_empty() {
        assert!(parse_robots(Some(&json!([{"radious": 0.3}]))).is_err());
        assert!(parse_robots(Some(&json!([]))).is_err());
        assert!(parse_robots(Some(&json!([{"drive": "tank"}]))).is_err());
        assert!(parse_step(Some(&json!({"mode": "warp"}))).is_err());
    }

    #[test]
    fn step_settings() {
        let s = parse_step(Some(&json!({"mode": "fixed", "dt": 0.02}))).unwrap();
        assert!(matches!(s.clock().unwrap(), StepClock::Fixed(dt) if dt == 0.02));
        let s = parse_step(Some(&json!({"mode": "real_time"}))).unwrap();
        assert_eq!(s, StepSetting::RealTime { max_dt: 0.1 });
        assert!(StepSetting::Fixed { dt: 0.0 }.clock().is_err());
    }
}
