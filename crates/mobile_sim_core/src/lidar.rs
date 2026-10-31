//! 2D LiDAR のシミュレーション
//!
//! 各ビームを占有格子の上で DDA(Amanatides–Woo)でたどり、最初の障害物の格子の境界までの距離を返す。

use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::error::{SimError, SimResult};
use crate::geometry::Pose2D;
use crate::map::OccupancyGrid;

/// LiDAR の設定
#[derive(Clone, Debug, PartialEq)]
pub struct LidarConfig {
    /// 最初のビームの角度 [rad](センサの座標系)
    pub angle_min: f64,
    /// 最後のビームの角度 [rad]
    pub angle_max: f64,
    /// ビームの本数(2 以上)
    pub num_beams: usize,
    /// 測れる最短の距離 [m]
    pub range_min: f64,
    /// 測れる最長の距離 [m]
    pub range_max: f64,
    /// ロボットの座標系での取り付け位置
    pub mount: Pose2D,
    /// 距離に足すガウスノイズの標準偏差 [m]。0 ならノイズなし
    pub noise_stddev: f64,
    /// 不明な格子を障害物として扱うか
    pub unknown_is_obstacle: bool,
}

impl Default for LidarConfig {
    fn default() -> Self {
        // 角度・本数・距離は turtle_sim と同じ(-1.5〜+1.5 rad、200 本、10 m)
        Self {
            angle_min: -1.5,
            angle_max: 1.5,
            num_beams: 200,
            range_min: 0.05,
            range_max: 10.0,
            mount: Pose2D::default(),
            noise_stddev: 0.0,
            unknown_is_obstacle: true,
        }
    }
}

impl LidarConfig {
    pub fn validate(&self) -> SimResult<()> {
        if self.num_beams < 2 {
            return Err(SimError::InvalidConfig(
                "lidar num_beams must be at least 2".to_owned(),
            ));
        }
        if self.angle_max.is_nan() || self.angle_min.is_nan() || self.angle_max <= self.angle_min {
            return Err(SimError::InvalidConfig(
                "lidar angle_max must be greater than angle_min".to_owned(),
            ));
        }
        if !(self.range_min >= 0.0 && self.range_max > self.range_min) {
            return Err(SimError::InvalidConfig(
                "lidar ranges must satisfy 0 <= range_min < range_max".to_owned(),
            ));
        }
        if !(self.noise_stddev >= 0.0 && self.noise_stddev.is_finite()) {
            return Err(SimError::InvalidConfig(
                "lidar noise_stddev must be finite and non-negative".to_owned(),
            ));
        }
        Ok(())
    }

    /// ビームの間の角度 [rad]
    pub fn angle_increment(&self) -> f64 {
        (self.angle_max - self.angle_min) / (self.num_beams - 1) as f64
    }
}

/// 1 回の走査の結果。値の並びは ROS / juiz_std の LaserScan と同じ
#[derive(Clone, Debug, PartialEq)]
pub struct Scan {
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub range_max: f64,
    /// 距離 [m]。何にも当たらなければ `f64::INFINITY`(range_max より大きい値)
    pub ranges: Vec<f64>,
}

/// ロボットの姿勢 `robot` から走査する
pub fn scan<R: Rng + ?Sized>(
    config: &LidarConfig,
    map: &OccupancyGrid,
    robot: &Pose2D,
    rng: &mut R,
) -> Scan {
    let sensor = robot.compose(&config.mount);
    let inc = config.angle_increment();
    let noise = (config.noise_stddev > 0.0)
        .then(|| Normal::new(0.0, config.noise_stddev).expect("validated stddev"));
    let ranges = (0..config.num_beams)
        .map(|k| {
            let angle = sensor.theta + config.angle_min + inc * k as f64;
            match cast_ray(
                map,
                sensor.x,
                sensor.y,
                angle,
                config.range_max,
                config.unknown_is_obstacle,
            ) {
                Some(d) => {
                    let d = match &noise {
                        Some(n) => d + n.sample(rng),
                        None => d,
                    };
                    d.clamp(config.range_min, config.range_max)
                }
                None => f64::INFINITY,
            }
        })
        .collect();
    Scan {
        angle_min: config.angle_min,
        angle_max: config.angle_max,
        angle_increment: inc,
        range_min: config.range_min,
        range_max: config.range_max,
        ranges,
    }
}

/// 世界座標の点 `(x, y)` から向き `angle` へ光線を飛ばし、障害物までの距離 [m] を返す。
/// `max_range` までに当たらなければ `None`。始点が障害物の中なら 0
pub fn cast_ray(
    map: &OccupancyGrid,
    x: f64,
    y: f64,
    angle: f64,
    max_range: f64,
    unknown_is_obstacle: bool,
) -> Option<f64> {
    let res = map.resolution();
    let (gx, gy) = map.world_to_grid(x, y);
    let (dx, dy) = (angle.cos(), angle.sin());
    let mut i = gx.floor() as i64;
    let mut j = gy.floor() as i64;
    if map.is_blocked(i, j, unknown_is_obstacle) {
        return Some(0.0);
    }
    let max_t = max_range / res;
    let step_i: i64 = if dx > 0.0 { 1 } else { -1 };
    let step_j: i64 = if dy > 0.0 { 1 } else { -1 };
    // 次の格子の境界までの、光線の長さ(格子の単位)
    let mut t_max_i = boundary_t(gx, dx);
    let mut t_max_j = boundary_t(gy, dy);
    let t_delta_i = if dx != 0.0 {
        1.0 / dx.abs()
    } else {
        f64::INFINITY
    };
    let t_delta_j = if dy != 0.0 {
        1.0 / dy.abs()
    } else {
        f64::INFINITY
    };
    loop {
        let t = if t_max_i < t_max_j {
            i += step_i;
            let t = t_max_i;
            t_max_i += t_delta_i;
            t
        } else {
            j += step_j;
            let t = t_max_j;
            t_max_j += t_delta_j;
            t
        };
        if t > max_t {
            return None;
        }
        if map.is_blocked(i, j, unknown_is_obstacle) {
            return Some(t * res);
        }
    }
}

/// 座標 `g` から向き `d` に進んで、最初に整数の境界に当たるまでの長さ
fn boundary_t(g: f64, d: f64) -> f64 {
    if d > 0.0 {
        (g.floor() + 1.0 - g) / d
    } else if d < 0.0 {
        (g - g.floor()) / -d
    } else {
        f64::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;
    use rand::rngs::StdRng;
    use rand::SeedableRng;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    /// 10 m 四方(0.05 m/px)、外周 1 画素が壁、原点が中心
    fn room() -> OccupancyGrid {
        let n = 200;
        let mut g = OccupancyGrid::new_free(n, n, 0.05, (-5.0, -5.0));
        for k in 0..n {
            g.set_cell(k, 0, Cell::Occupied);
            g.set_cell(k, n - 1, Cell::Occupied);
            g.set_cell(0, k, Cell::Occupied);
            g.set_cell(n - 1, k, Cell::Occupied);
        }
        g
    }

    #[test]
    fn distances_to_walls_of_room() {
        let g = room();
        // 壁の内側の面は ±4.95 m
        for (angle, expected) in [
            (0.0, 4.95),
            (FRAC_PI_2, 4.95),
            (PI, 4.95),
            (-FRAC_PI_2, 4.95),
        ] {
            let d = cast_ray(&g, 0.0, 0.0, angle, 20.0, true).unwrap();
            assert!((d - expected).abs() < 1e-9, "angle {angle}: {d}");
        }
        let d = cast_ray(&g, 0.0, 0.0, FRAC_PI_4, 20.0, true).unwrap();
        assert!((d - 4.95 * 2f64.sqrt()).abs() < 0.05 * 2f64.sqrt(), "{d}");
        let d = cast_ray(&g, 1.0, -2.0, 0.0, 20.0, true).unwrap();
        assert!((d - 3.95).abs() < 1e-9, "{d}");
    }

    #[test]
    fn out_of_range_and_inside_obstacle() {
        let g = room();
        assert_eq!(cast_ray(&g, 0.0, 0.0, 0.0, 3.0, true), None);
        assert_eq!(cast_ray(&g, -4.99, 0.0, 0.0, 3.0, true), Some(0.0));
        // マップの外は障害物
        let open = OccupancyGrid::new_free(10, 10, 1.0, (0.0, 0.0));
        let d = cast_ray(&open, 5.5, 5.5, 0.0, 100.0, true).unwrap();
        assert!((d - 4.5).abs() < 1e-9);
    }

    #[test]
    fn unknown_cells_follow_setting() {
        let mut g = OccupancyGrid::new_free(10, 1, 1.0, (0.0, 0.0));
        g.set_cell(5, 0, Cell::Unknown);
        assert!((cast_ray(&g, 0.5, 0.5, 0.0, 100.0, true).unwrap() - 4.5).abs() < 1e-9);
        assert!((cast_ray(&g, 0.5, 0.5, 0.0, 100.0, false).unwrap() - 9.5).abs() < 1e-9);
    }

    #[test]
    fn scan_uses_robot_pose_and_mount() {
        let g = room();
        let cfg = LidarConfig {
            angle_min: -FRAC_PI_2,
            angle_max: FRAC_PI_2,
            num_beams: 3,
            range_max: 6.0,
            mount: Pose2D::new(0.5, 0.0, 0.0),
            ..Default::default()
        };
        let mut rng = StdRng::seed_from_u64(1);
        // ロボットは (1, 0) で +y を向く。センサは (1, 0.5)
        let s = scan(&cfg, &g, &Pose2D::new(1.0, 0.0, FRAC_PI_2), &mut rng);
        assert!((s.angle_increment - FRAC_PI_2).abs() < 1e-12);
        assert!((s.ranges[0] - 3.95).abs() < 1e-9, "{:?}", s.ranges); // 右 = +x
        assert!((s.ranges[1] - 4.45).abs() < 1e-9, "{:?}", s.ranges); // 前 = +y
        assert!((s.ranges[2] - 5.95).abs() < 1e-9, "{:?}", s.ranges); // 左 = -x

        let short = LidarConfig {
            range_max: 5.0,
            ..cfg
        };
        let s = scan(&short, &g, &Pose2D::new(1.0, 0.0, FRAC_PI_2), &mut rng);
        assert!(s.ranges[2].is_infinite(), "{:?}", s.ranges);
    }

    #[test]
    fn noise_is_seeded_and_clamped() {
        let g = room();
        let cfg = LidarConfig {
            noise_stddev: 0.01,
            ..Default::default()
        };
        let p = Pose2D::default();
        let a = scan(&cfg, &g, &p, &mut StdRng::seed_from_u64(7));
        let b = scan(&cfg, &g, &p, &mut StdRng::seed_from_u64(7));
        let clean = scan(
            &LidarConfig {
                noise_stddev: 0.0,
                ..cfg.clone()
            },
            &g,
            &p,
            &mut StdRng::seed_from_u64(7),
        );
        assert_eq!(a, b);
        assert_ne!(a.ranges, clean.ranges);
        for (n, c) in a.ranges.iter().zip(&clean.ranges) {
            assert!((n - c).abs() < 0.1);
            assert!(*n >= cfg.range_min && *n <= cfg.range_max);
        }
    }

    #[test]
    fn validate_rejects_bad_config() {
        assert!(LidarConfig::default().validate().is_ok());
        for bad in [
            LidarConfig {
                num_beams: 1,
                ..Default::default()
            },
            LidarConfig {
                angle_max: -2.0,
                ..Default::default()
            },
            LidarConfig {
                range_min: 11.0,
                ..Default::default()
            },
            LidarConfig {
                noise_stddev: -1.0,
                ..Default::default()
            },
        ] {
            assert!(bad.validate().is_err(), "{bad:?}");
        }
    }
}
