//! シミュレーションの世界: マップとロボットの列

use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::collision::circle_collides;
use crate::error::{SimError, SimResult};
use crate::geometry::{Pose2D, Twist2D};
use crate::lidar::{self, LidarConfig, Scan};
use crate::map::OccupancyGrid;
use crate::vehicle::{Vehicle, VehicleConfig};

/// ロボット 1 台の設定
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RobotConfig {
    pub vehicle: VehicleConfig,
    pub lidar: LidarConfig,
    pub initial_pose: Pose2D,
}

/// ロボット 1 台
#[derive(Clone, Debug)]
pub struct Robot {
    pub vehicle: Vehicle,
    pub lidar: LidarConfig,
    /// 直近の `step` で壁に当たって止まったか
    pub bumper: bool,
    /// 走った距離の合計 [m](オドメトリ用)
    pub distance: f64,
}

/// ロボット 1 台の状態の写し(画面や juiz の出力用)
#[derive(Clone, Debug, PartialEq)]
pub struct RobotState {
    pub pose: Pose2D,
    pub velocity: Twist2D,
    pub target_velocity: Twist2D,
    pub bumper: bool,
    pub radius: f64,
}

/// シミュレーションの世界
#[derive(Debug)]
pub struct World {
    map: OccupancyGrid,
    robots: Vec<Robot>,
    /// 衝突で不明な格子を障害物として扱うか
    pub unknown_is_obstacle: bool,
    time: f64,
    rng: StdRng,
}

impl World {
    pub fn new(map: OccupancyGrid) -> Self {
        Self::with_seed(map, 0)
    }

    /// LiDAR のノイズの乱数の種を指定して作る(同じ種なら同じ結果になる)
    pub fn with_seed(map: OccupancyGrid, seed: u64) -> Self {
        Self {
            map,
            robots: Vec::new(),
            unknown_is_obstacle: true,
            time: 0.0,
            rng: StdRng::seed_from_u64(seed),
        }
    }

    pub fn map(&self) -> &OccupancyGrid {
        &self.map
    }

    /// マップを入れ替える。ロボットの位置はそのまま
    pub fn set_map(&mut self, map: OccupancyGrid) {
        self.map = map;
    }

    /// シミュレーションの経過時間 [s]
    pub fn time(&self) -> f64 {
        self.time
    }

    /// ロボットを足し、その番号を返す
    pub fn spawn(&mut self, config: RobotConfig) -> SimResult<usize> {
        config.vehicle.validate()?;
        config.lidar.validate()?;
        self.robots.push(Robot {
            vehicle: Vehicle::new(config.vehicle, config.initial_pose),
            lidar: config.lidar,
            bumper: false,
            distance: 0.0,
        });
        Ok(self.robots.len() - 1)
    }

    pub fn robot_count(&self) -> usize {
        self.robots.len()
    }

    pub fn robot(&self, index: usize) -> SimResult<&Robot> {
        self.robots.get(index).ok_or(SimError::NoSuchRobot(index))
    }

    fn robot_mut(&mut self, index: usize) -> SimResult<&mut Robot> {
        self.robots
            .get_mut(index)
            .ok_or(SimError::NoSuchRobot(index))
    }

    pub fn set_velocity(&mut self, index: usize, cmd: Twist2D) -> SimResult<()> {
        self.robot_mut(index)?.vehicle.set_target_velocity(cmd);
        Ok(())
    }

    pub fn reset_pose(&mut self, index: usize, pose: Pose2D) -> SimResult<()> {
        let r = self.robot_mut(index)?;
        r.vehicle.reset(pose);
        r.bumper = false;
        Ok(())
    }

    pub fn state(&self, index: usize) -> SimResult<RobotState> {
        let r = self.robot(index)?;
        Ok(RobotState {
            pose: r.vehicle.pose,
            velocity: r.vehicle.velocity,
            target_velocity: r.vehicle.target_velocity(),
            bumper: r.bumper,
            radius: r.vehicle.config.radius,
        })
    }

    /// 全ロボットを `dt` [s] 進める。移動後に壁に重なるロボットはその場で止める
    pub fn step(&mut self, dt: f64) {
        if !(dt > 0.0 && dt.is_finite()) {
            return;
        }
        self.time += dt;
        for r in &mut self.robots {
            let next = r.vehicle.advance(dt);
            let radius = r.vehicle.config.radius;
            if circle_collides(&self.map, next.x, next.y, radius, self.unknown_is_obstacle) {
                r.vehicle.stop();
                r.bumper = true;
            } else {
                r.distance += (next.x - r.vehicle.pose.x).hypot(next.y - r.vehicle.pose.y);
                r.vehicle.pose = next;
                r.bumper = false;
            }
        }
    }

    /// ロボット `index` の LiDAR で走査する
    pub fn scan(&mut self, index: usize) -> SimResult<Scan> {
        let r = self.robots.get(index).ok_or(SimError::NoSuchRobot(index))?;
        Ok(lidar::scan(
            &r.lidar,
            &self.map,
            &r.vehicle.pose,
            &mut self.rng,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;
    use crate::vehicle::DriveKind;

    /// 10 m x 4 m(0.1 m/px)、x = 3.0〜3.1 m に壁
    fn corridor() -> OccupancyGrid {
        let mut g = OccupancyGrid::new_free(100, 40, 0.1, (-5.0, -2.0));
        for j in 0..40 {
            g.set_cell(80, j, Cell::Occupied);
        }
        g
    }

    fn fast_robot() -> RobotConfig {
        RobotConfig {
            vehicle: VehicleConfig {
                max_velocity: Twist2D::new(1.0, 1.0, 1.0),
                max_acceleration: Twist2D::new(10.0, 10.0, 10.0),
                command_timeout: 0.0,
                radius: 0.2,
                drive: DriveKind::DiffDrive,
            },
            ..Default::default()
        }
    }

    #[test]
    fn robot_stops_at_wall() {
        let mut w = World::new(corridor());
        let id = w.spawn(fast_robot()).unwrap();
        w.set_velocity(id, Twist2D::new(1.0, 0.0, 0.0)).unwrap();
        for _ in 0..1000 {
            w.step(0.01);
        }
        let s = w.state(id).unwrap();
        assert!(s.bumper);
        assert_eq!(s.velocity, Twist2D::ZERO);
        // 壁の面は x = 3.0、半径 0.2 なので 2.8 の手前で止まる
        assert!(s.pose.x < 2.8 && s.pose.x > 2.78, "{}", s.pose.x);
        assert!((w.time() - 10.0).abs() < 1e-9);
        assert!((w.robot(id).unwrap().distance - s.pose.x).abs() < 1e-9);

        // 向きを変えて戻れる
        w.set_velocity(id, Twist2D::new(-1.0, 0.0, 0.0)).unwrap();
        w.step(0.1);
        let s = w.state(id).unwrap();
        assert!(!s.bumper && s.pose.x < 2.78);
    }

    #[test]
    fn scan_sees_wall_ahead() {
        let mut w = World::new(corridor());
        let id = w
            .spawn(RobotConfig {
                lidar: LidarConfig {
                    angle_min: -0.1,
                    angle_max: 0.1,
                    num_beams: 3,
                    ..Default::default()
                },
                initial_pose: Pose2D::new(1.0, 0.0, 0.0),
                ..fast_robot()
            })
            .unwrap();
        let s = w.scan(id).unwrap();
        assert!((s.ranges[1] - 2.0).abs() < 1e-9, "{:?}", s.ranges);
        assert!(w.scan(1).is_err());
    }

    #[test]
    fn reset_pose_and_errors() {
        let mut w = World::new(corridor());
        let id = w.spawn(fast_robot()).unwrap();
        w.set_velocity(id, Twist2D::new(1.0, 0.0, 0.0)).unwrap();
        w.step(0.5);
        w.reset_pose(id, Pose2D::new(-1.0, 1.0, 7.0)).unwrap();
        let s = w.state(id).unwrap();
        assert_eq!((s.pose.x, s.pose.y), (-1.0, 1.0));
        assert!(s.pose.theta.abs() <= std::f64::consts::PI);
        assert_eq!(s.velocity, Twist2D::ZERO);
        assert_eq!(s.target_velocity, Twist2D::ZERO);
        assert!(matches!(
            w.set_velocity(3, Twist2D::ZERO),
            Err(SimError::NoSuchRobot(3))
        ));

        let bad = RobotConfig {
            vehicle: VehicleConfig {
                radius: -1.0,
                ..Default::default()
            },
            ..Default::default()
        };
        assert!(w.spawn(bad).is_err());
    }

    #[test]
    fn non_positive_dt_does_nothing() {
        let mut w = World::new(corridor());
        let id = w.spawn(fast_robot()).unwrap();
        w.set_velocity(id, Twist2D::new(1.0, 0.0, 0.0)).unwrap();
        w.step(0.0);
        w.step(-1.0);
        w.step(f64::NAN);
        assert_eq!(w.time(), 0.0);
        assert_eq!(w.state(id).unwrap().pose, Pose2D::default());
    }
}
