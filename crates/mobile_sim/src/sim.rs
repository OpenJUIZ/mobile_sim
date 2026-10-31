//! コンテナ `mobile_sim` の中身。juiz のマクロから呼ばれる処理をここにまとめ、juiz を起動せずにテストできるようにする

use std::path::{Path, PathBuf};

use juiz_sdk::image::{DynamicImage, GrayImage, Luma};
use juiz_sdk::serde_json::{json, Value};
use juiz_std_types::common::{Header, Time};
use juiz_std_types::geometry::{Point2D, Pose2D, Pose2DStamped, Twist2D, Vector2};
use juiz_std_types::sensor::LaserScan;
use mobile_sim_core as core;
use mobile_sim_core::{Cell, MapConf, OccupancyGrid, RobotConfig, Scan, StepClock, World};

use crate::config::{parse_robots, parse_step};

/// conf の `map` を上書きする環境変数(Docker でマップを差し替えるとき用)
pub const MAP_ENV: &str = "MOBILE_SIM_MAP";
/// `juiz run` が展開先を入れる環境変数(juiz の docs/spec/app-package.md §7.1)
pub const APP_DIR_ENV: &str = "JUIZ_APP_DIR";

/// コンテナ `mobile_sim` の状態
pub struct MobileSim {
    world: World,
    clock: StepClock,
    map_path: PathBuf,
    map_conf: MapConf,
    map_image: DynamicImage,
}

impl MobileSim {
    /// conf のコンテナの引数から作る。`map` は mapconf のパス
    pub fn new(
        map: &str,
        robots: Option<&Value>,
        step: Option<&Value>,
        seed: u64,
    ) -> Result<Self, String> {
        let map_path = resolve_map_path(map);
        let (grid, map_conf) = load_map(&map_path)?;
        let map_image = render_map(&grid);
        let mut world = World::with_seed(grid, seed);
        for r in parse_robots(robots)? {
            world
                .spawn(RobotConfig::from(&r))
                .map_err(|e| e.to_string())?;
        }
        let clock = parse_step(step)?.clock()?;
        Ok(Self {
            world,
            clock,
            map_path,
            map_conf,
            map_image,
        })
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    /// 時計の dt だけ進める
    pub fn step(&mut self) {
        let dt = self.clock.tick();
        self.world.step(dt);
    }

    pub fn set_velocity(&mut self, index: i64, v: &Twist2D) -> Result<(), String> {
        let cmd = core::Twist2D::new(v.linear.x, v.linear.y, v.angular);
        self.world
            .set_velocity(to_index(index)?, cmd)
            .map_err(|e| e.to_string())
    }

    pub fn reset_pose(&mut self, index: i64, p: &Pose2D) -> Result<(), String> {
        let pose = core::Pose2D::new(p.position.x, p.position.y, p.theta);
        self.world
            .reset_pose(to_index(index)?, pose)
            .map_err(|e| e.to_string())
    }

    pub fn pose(&self, index: i64) -> Result<Pose2DStamped, String> {
        let s = self
            .world
            .state(to_index(index)?)
            .map_err(|e| e.to_string())?;
        Ok(Pose2DStamped {
            header: header(self.world.time(), "map"),
            pose: to_pose(&s.pose),
        })
    }

    pub fn odometry(&self, index: i64) -> Result<Value, String> {
        let i = to_index(index)?;
        let s = self.world.state(i).map_err(|e| e.to_string())?;
        let r = self.world.robot(i).map_err(|e| e.to_string())?;
        Ok(json!({
            "time": self.world.time(),
            "pose": pose_json(&s.pose),
            "twist": twist_json(&s.velocity),
            "distance": r.distance,
        }))
    }

    pub fn scan(&mut self, index: i64) -> Result<LaserScan, String> {
        let scan = self
            .world
            .scan(to_index(index)?)
            .map_err(|e| e.to_string())?;
        Ok(to_laser_scan(&scan, self.world.time()))
    }

    pub fn map_image(&self) -> DynamicImage {
        self.map_image.clone()
    }

    pub fn map_info(&self) -> Value {
        let g = self.world.map();
        json!({
            "path": self.map_path.to_string_lossy(),
            "image": self.map_conf.image.to_string_lossy(),
            "resolution": g.resolution(),
            "origin": self.map_conf.origin,
            "width": g.width(),
            "height": g.height(),
            "occupied_thresh": self.map_conf.occupied_thresh,
            "free_thresh": self.map_conf.free_thresh,
            "negate": self.map_conf.negate,
        })
    }

    /// 別の mapconf に切り替える。ロボットの位置はそのまま
    pub fn load_map(&mut self, map: &str) -> Result<(), String> {
        let path = resolve_path(Path::new(map));
        let (grid, conf) = load_map(&path)?;
        self.map_image = render_map(&grid);
        self.map_conf = conf;
        self.map_path = path;
        self.world.set_map(grid);
        Ok(())
    }

    /// 画面用に、全ロボットの状態とスキャンをまとめて返す
    pub fn state(&mut self) -> Value {
        let n = self.world.robot_count();
        let robots: Vec<Value> = (0..n)
            .map(|i| {
                let s = self.world.state(i).expect("index in range");
                let scan = self.world.scan(i).expect("index in range");
                json!({
                    "pose": pose_json(&s.pose),
                    "velocity": twist_json(&s.velocity),
                    "target_velocity": twist_json(&s.target_velocity),
                    "bumper": s.bumper,
                    "radius": s.radius,
                    "scan": {
                        "angle_min": scan.angle_min,
                        "angle_increment": scan.angle_increment,
                        "range_max": scan.range_max,
                        "mount": pose_json(&self.world.robot(i).expect("index in range").lidar.mount),
                        "ranges": scan.ranges.iter().map(|r| out_of_range(*r, scan.range_max)).collect::<Vec<_>>(),
                    },
                })
            })
            .collect();
        json!({ "time": self.world.time(), "robots": robots })
    }
}

fn to_index(index: i64) -> Result<usize, String> {
    usize::try_from(index).map_err(|_| format!("robot index must not be negative, got {index}"))
}

/// mapconf のパスを決める。環境変数 `MOBILE_SIM_MAP` があればそれを使う
fn resolve_map_path(map: &str) -> PathBuf {
    match std::env::var(MAP_ENV) {
        Ok(p) if !p.is_empty() => resolve_path(Path::new(&p)),
        _ => resolve_path(Path::new(map)),
    }
}

/// 相対パスは `JUIZ_APP_DIR`(`juiz run` の展開先)があればそこから、無ければカレントディレクトリから
fn resolve_path(p: &Path) -> PathBuf {
    if p.is_absolute() {
        return p.to_owned();
    }
    match std::env::var_os(APP_DIR_ENV) {
        Some(dir) if !dir.is_empty() => PathBuf::from(dir).join(p),
        _ => p.to_owned(),
    }
}

fn load_map(path: &Path) -> Result<(OccupancyGrid, MapConf), String> {
    OccupancyGrid::load(path).map_err(|e| e.to_string())
}

/// 占有格子を画像にする(障害物は黒、空きは白、不明は灰色。上の行が y の大きい側)
pub fn render_map(g: &OccupancyGrid) -> DynamicImage {
    let (w, h) = (g.width() as u32, g.height() as u32);
    let img = GrayImage::from_fn(w, h, |x, y| {
        let j = (h - 1 - y) as i64;
        Luma([match g.cell(x as i64, j) {
            Some(Cell::Occupied) => 0,
            Some(Cell::Free) => 254,
            _ => 205,
        }])
    });
    DynamicImage::ImageLuma8(img)
}

fn header(time: f64, frame_id: &str) -> Header {
    let sec = time.floor();
    Header {
        stamp: Time {
            sec: sec as i64,
            nanosec: (((time - sec) * 1e9).round() as i32).clamp(0, 999_999_999),
        },
        frame_id: frame_id.to_owned(),
    }
}

fn to_pose(p: &core::Pose2D) -> Pose2D {
    Pose2D {
        position: Point2D { x: p.x, y: p.y },
        theta: p.theta,
    }
}

fn pose_json(p: &core::Pose2D) -> Value {
    json!({ "x": p.x, "y": p.y, "theta": p.theta })
}

fn twist_json(t: &core::Twist2D) -> Value {
    json!({ "vx": t.vx, "vy": t.vy, "omega": t.omega })
}

/// 何にも当たらなかったビーム(無限大)は、JSON に載せられるよう range_max + 1 にする
fn out_of_range(r: f64, range_max: f64) -> f64 {
    if r.is_finite() {
        r
    } else {
        range_max + 1.0
    }
}

fn to_laser_scan(s: &Scan, time: f64) -> LaserScan {
    LaserScan {
        header: header(time, "laser"),
        angle_min: s.angle_min as f32,
        angle_max: s.angle_max as f32,
        angle_increment: s.angle_increment as f32,
        range_min: s.range_min as f32,
        range_max: s.range_max as f32,
        ranges: s
            .ranges
            .iter()
            .map(|r| out_of_range(*r, s.range_max) as f32)
            .collect(),
        time_increment: 0.0,
        scan_time: 0.0,
        intensities: Vec::new(),
    }
}

/// juiz の Twist2D を作る(テストとサンプル用)
pub fn twist(vx: f64, vy: f64, omega: f64) -> Twist2D {
    Twist2D {
        linear: Vector2 { x: vx, y: vy },
        angular: omega,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use juiz_sdk::serde_json::json;

    /// 10 m 四方、外周が壁の部屋の mapconf を一時ディレクトリに作る
    fn room(dir: &Path) -> PathBuf {
        let n = 100;
        let img = GrayImage::from_fn(n, n, |x, y| {
            if x == 0 || y == 0 || x == n - 1 || y == n - 1 {
                Luma([0])
            } else {
                Luma([255])
            }
        });
        img.save(dir.join("room.png")).unwrap();
        let yaml = dir.join("room.yaml");
        std::fs::write(
            &yaml,
            "image: room.png\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n",
        )
        .unwrap();
        yaml
    }

    fn sim(dir: &Path) -> MobileSim {
        MobileSim::new(
            room(dir).to_str().unwrap(),
            Some(&json!([{"max_acceleration": {"vx": 10.0, "vy": 10.0, "omega": 10.0}}])),
            Some(&json!({"mode": "fixed", "dt": 0.1})),
            0,
        )
        .unwrap()
    }

    #[test]
    fn velocity_moves_pose() {
        let dir = tempfile::tempdir().unwrap();
        let mut s = sim(dir.path());
        s.set_velocity(0, &twist(0.5, 0.0, 0.0)).unwrap();
        for _ in 0..4 {
            s.step();
        }
        let p = s.pose(0).unwrap();
        assert!((p.pose.position.x - 0.2).abs() < 1e-9, "{p:?}");
        assert_eq!(p.header.frame_id, "map");
        assert_eq!(p.header.stamp.sec, 0);
        assert!((p.header.stamp.nanosec - 400_000_000).abs() <= 1);

        let odo = s.odometry(0).unwrap();
        assert!((odo["distance"].as_f64().unwrap() - 0.2).abs() < 1e-9);
        assert!(s.set_velocity(1, &twist(0.0, 0.0, 0.0)).is_err());
        assert!(s.set_velocity(-1, &twist(0.0, 0.0, 0.0)).is_err());
    }

    #[test]
    fn scan_and_state() {
        let dir = tempfile::tempdir().unwrap();
        let mut s = sim(dir.path());
        let scan = s.scan(0).unwrap();
        assert_eq!(scan.ranges.len(), 200);
        // 正面の壁の内側の面は x = 4.9
        let mid = scan.ranges[99].min(scan.ranges[100]);
        assert!((mid - 4.9).abs() < 0.05, "{mid}");
        assert_eq!(scan.header.frame_id, "laser");

        let st = s.state();
        assert_eq!(st["robots"].as_array().unwrap().len(), 1);
        assert_eq!(
            st["robots"][0]["scan"]["ranges"].as_array().unwrap().len(),
            200
        );
        assert_eq!(st["robots"][0]["bumper"], json!(false));
    }

    #[test]
    fn out_of_range_beams_are_finite() {
        let dir = tempfile::tempdir().unwrap();
        let mut s = MobileSim::new(
            room(dir.path()).to_str().unwrap(),
            Some(&json!([{"lidar": {"range_max": 2.0}}])),
            None,
            0,
        )
        .unwrap();
        let scan = s.scan(0).unwrap();
        assert!(scan.ranges.iter().all(|r| *r == 3.0));
        let st = s.state();
        assert_eq!(st["robots"][0]["scan"]["ranges"][0], json!(3.0));
    }

    #[test]
    fn map_image_info_and_reload() {
        let dir = tempfile::tempdir().unwrap();
        let mut s = sim(dir.path());
        let img = s.map_image().to_luma8();
        assert_eq!(img.dimensions(), (100, 100));
        assert_eq!(img.get_pixel(0, 0).0[0], 0);
        assert_eq!(img.get_pixel(50, 50).0[0], 254);
        let info = s.map_info();
        assert_eq!(info["resolution"], json!(0.1));
        assert_eq!(info["origin"], json!([-5.0, -5.0, 0.0]));
        assert_eq!(info["width"], json!(100));

        let small = dir.path().join("small");
        std::fs::create_dir(&small).unwrap();
        GrayImage::from_pixel(10, 20, Luma([255]))
            .save(small.join("s.png"))
            .unwrap();
        std::fs::write(
            small.join("s.yaml"),
            "image: s.png\nresolution: 1.0\norigin: [0, 0, 0]\n",
        )
        .unwrap();
        s.load_map(small.join("s.yaml").to_str().unwrap()).unwrap();
        assert_eq!(s.map_info()["height"], json!(20));
        assert_eq!(s.map_image().to_luma8().dimensions(), (10, 20));
        assert!(s.load_map("does/not/exist.yaml").is_err());
    }

    #[test]
    fn reset_pose_and_bad_args() {
        let dir = tempfile::tempdir().unwrap();
        let mut s = sim(dir.path());
        let p = Pose2D {
            position: Point2D { x: 1.0, y: -2.0 },
            theta: 0.5,
        };
        s.reset_pose(0, &p).unwrap();
        assert_eq!(s.pose(0).unwrap().pose, p);

        let yaml = room(dir.path());
        let y = yaml.to_str().unwrap();
        assert!(MobileSim::new("missing.yaml", None, None, 0).is_err());
        assert!(MobileSim::new(y, Some(&json!([{"radius": -1.0}])), None, 0).is_err());
        assert!(MobileSim::new(y, None, Some(&json!({"mode": "fixed", "dt": -1})), 0).is_err());
    }

    #[test]
    fn header_splits_time() {
        let h = header(12.25, "map");
        assert_eq!((h.stamp.sec, h.stamp.nanosec), (12, 250_000_000));
    }
}
