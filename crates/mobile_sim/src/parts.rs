//! Transformations で使う部品: ロボット(`mobile_robot`)と LiDAR(`mobile_lidar`)の Container の中身。
//!
//! juiz の ContainerProcess は自分の ContainerPose しか読み書きできない(juiz docs/spec/transformations.md §12)。
//! そこでロボットと LiDAR を 1 台ずつ Container に分け、シミュレーションの世界(コンテナ `mobile_sim`)は
//! プラグインの中の表で名前から引いて共有する。ロックは「Container → 世界」の順にだけ取る。

use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, Weak};

use juiz_sdk::geometry::{Pose, Quaternion, Vector3};
use juiz_sdk::serde_json::{self, Map, Value};
use mobile_sim_core as core;

use crate::config::{LidarSetting, RobotSetting};
use crate::sim::MobileSim;

/// 世界への共有の参照
pub type SimRef = Arc<Mutex<MobileSim>>;

fn sims() -> &'static Mutex<HashMap<String, Weak<Mutex<MobileSim>>>> {
    static SIMS: OnceLock<Mutex<HashMap<String, Weak<Mutex<MobileSim>>>>> = OnceLock::new();
    SIMS.get_or_init(|| Mutex::new(HashMap::new()))
}

/// コンテナ `mobile_sim` の中身。世界を名前で表に載せ、`mobile_robot`・`mobile_lidar` から引けるようにする
pub struct SimHandle {
    sim: SimRef,
}

impl SimHandle {
    /// 世界を名前 `name` で表に載せる。同じ名前の古い世界(削除済み)は置き換える
    pub fn register(name: &str, sim: MobileSim) -> Result<Self, String> {
        let sim = Arc::new(Mutex::new(sim));
        let mut table = sims().lock().map_err(|_| "simulator table is poisoned")?;
        if table.get(name).and_then(Weak::upgrade).is_some() {
            return Err(format!("simulator '{name}' already exists"));
        }
        table.insert(name.to_owned(), Arc::downgrade(&sim));
        Ok(Self { sim })
    }

    pub fn sim(&self) -> MutexGuard<'_, MobileSim> {
        lock(&self.sim)
    }
}

/// 名前 `name` の世界を引く
pub fn find_sim(name: &str) -> Result<SimRef, String> {
    let table = sims().lock().map_err(|_| "simulator table is poisoned")?;
    table.get(name).and_then(Weak::upgrade).ok_or_else(|| {
        format!(
            "no mobile_sim container named '{name}'. Write it before the robots and LiDARs in containers:"
        )
    })
}

fn lock(sim: &SimRef) -> MutexGuard<'_, MobileSim> {
    // 中で panic しても世界の値は壊れていないので、そのまま使う
    sim.lock().unwrap_or_else(|e| e.into_inner())
}

/// 引数の値のうち null でないものだけを JSON の object にまとめる
pub fn settings_object(items: &[(&str, Option<Value>)]) -> Value {
    let mut m = Map::new();
    for (k, v) in items {
        if let Some(v) = v {
            if !v.is_null() {
                m.insert((*k).to_owned(), v.clone());
            }
        }
    }
    Value::Object(m)
}

/// Container `mobile_robot` の中身: 台車 1 台
pub struct MobileRobot {
    sim: SimRef,
    pub index: usize,
    /// 真の位置を書く基準(Anchor)の名前
    pub world_frame: String,
    /// オドメトリを書く基準(Anchor)の名前
    pub odom_frame: String,
}

impl MobileRobot {
    /// 世界 `sim` に、名前 `name` のロボットを足す。`settings` は conf の `robots:` の 1 要素と同じ形(lidar は除く)
    pub fn new(
        name: &str,
        sim: &str,
        world_frame: &str,
        odom_frame: &str,
        settings: &Value,
    ) -> Result<Self, String> {
        let setting: RobotSetting = serde_json::from_value(settings.clone())
            .map_err(|e| format!("invalid mobile_robot '{name}': {e}"))?;
        let sim = find_sim(sim)?;
        let index = lock(&sim).add_robot(name, &setting)?;
        Ok(Self {
            sim,
            index,
            world_frame: world_frame.to_owned(),
            odom_frame: odom_frame.to_owned(),
        })
    }

    pub fn sim(&self) -> MutexGuard<'_, MobileSim> {
        lock(&self.sim)
    }

    /// (真の姿勢, オドメトリの姿勢, シミュレーションの時刻)
    pub fn poses(&self) -> Result<(core::Pose2D, core::Pose2D, f64), String> {
        self.sim().poses(self.index)
    }
}

/// Container `mobile_lidar` の中身: LiDAR 1 台。取付位置は Transform(載せたロボット → 自分)だけが持つ
pub struct MobileLidar {
    sim: SimRef,
    pub index: usize,
    /// 自分の Container の名前(LaserScan の frame_id)
    pub name: String,
    /// 載せたロボットの Container の名前
    pub base: String,
    pub setting: LidarSetting,
}

impl MobileLidar {
    /// 世界 `sim` の、Container 名 `base` のロボットに載せる。`settings` は conf の `lidar:` と同じ形(mount は除く)
    pub fn new(name: &str, sim: &str, base: &str, settings: &Value) -> Result<Self, String> {
        let setting: LidarSetting = serde_json::from_value(settings.clone())
            .map_err(|e| format!("invalid mobile_lidar '{name}': {e}"))?;
        if setting.mount.is_some() {
            return Err(format!(
                "mobile_lidar '{name}': write the mount as a transform {base} -> {name} instead"
            ));
        }
        let sim = find_sim(sim)?;
        let index = {
            let mut s = lock(&sim);
            let index = s.robot_index(base)?;
            // 取付位置は最初の走査で Transform から読む。それまでは中心に置く
            s.set_lidar(index, &setting, core::Pose2D::default())?;
            index
        };
        Ok(Self {
            sim,
            index,
            name: name.to_owned(),
            base: base.to_owned(),
            setting,
        })
    }

    pub fn sim(&self) -> MutexGuard<'_, MobileSim> {
        lock(&self.sim)
    }
}

/// 平面の姿勢を 3 次元の Pose にする(z = 0、z 軸まわりに theta)
pub fn to_pose3(p: &core::Pose2D) -> Pose {
    Pose::new(
        Vector3::new(p.x, p.y, 0.0),
        Quaternion::from_rpy(0.0, 0.0, p.theta),
    )
}

/// 3 次元の Pose を平面に落とす(x, y と yaw)
pub fn to_pose2(p: &Pose) -> core::Pose2D {
    let (_, _, yaw) = p.orientation.to_rpy();
    core::Pose2D::new(p.position.x, p.position.y, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;
    use juiz_sdk::serde_json::json;

    #[test]
    fn pose_round_trip() {
        let p = core::Pose2D::new(1.5, -2.0, 2.5);
        let q = to_pose2(&to_pose3(&p));
        assert!((q.x - 1.5).abs() < 1e-12 && (q.y + 2.0).abs() < 1e-12);
        assert!((q.theta - 2.5).abs() < 1e-12);
    }

    #[test]
    fn settings_object_skips_null() {
        let v = settings_object(&[
            ("radius", Some(json!(0.3))),
            ("drive", None),
            ("odometry", Some(Value::Null)),
        ]);
        assert_eq!(v, json!({"radius": 0.3}));
    }
}
