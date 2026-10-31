//! mobile_sim の juiz コンポーネント。
//!
//! `mobile_sim_core` を包み、コンテナ `mobile_sim` と、その ContainerProcess を juiz に見せる。
//! 処理の中身は [`sim::MobileSim`] にあり、ここは juiz の型との受け渡しだけをする。
//!
//! | ContainerProcess | 引数 | 戻り値 |
//! | --- | --- | --- |
//! | `mobile_sim_step` | なし | なし |
//! | `mobile_sim_set_velocity` | `velocity: Twist2D`, `index` | なし |
//! | `mobile_sim_get_pose` | `index` | `Pose2DStamped` |
//! | `mobile_sim_get_odometry` | `index` | `{time, pose, twist, distance}` |
//! | `mobile_sim_get_scan` | `index` | `LaserScan` |
//! | `mobile_sim_get_map` | なし | 画像 |
//! | `mobile_sim_get_map_info` | なし | mapconf の内容と画像の大きさ |
//! | `mobile_sim_load_map` | `map`(mapconf のパス) | なし |
//! | `mobile_sim_reset_pose` | `pose: Pose2D`, `index` | なし |
//! | `mobile_sim_get_state` | なし | 全ロボットの姿勢・速度・スキャン |

pub mod config;
pub mod sim;

use juiz_sdk::anyhow::anyhow;
use juiz_sdk::prelude::*;
use juiz_std_types::geometry::{Pose2D, Pose2DStamped, Twist2D};
use juiz_std_types::sensor::LaserScan;

pub use mobile_sim_core::VERSION;
pub use sim::MobileSim;

fn err(e: String) -> JuizError {
    JuizError::InvalidValueError { message: e }
}

#[juiz_component_container(
    description = "Planar mobile robot simulator with map, collision and LiDAR"
    arguments = {
        default = {
            map = "assets/maps/default.yaml"
            seed = 0
        }
        description = {
            map = "Path of the mapconf (ROS map_server YAML). Relative to JUIZ_APP_DIR if set"
            seed = "Random seed of LiDAR noise"
            robots = "List of robot settings (drive, radius, max_velocity, max_acceleration, command_timeout, initial_pose, lidar)"
            step = "{mode: fixed, dt} or {mode: real_time, max_dt}"
        }
    }
)]
fn mobile_sim(
    map: String,
    seed: i64,
    robots: Option<Value>,
    step: Option<Value>,
) -> JuizResult<Box<MobileSim>> {
    let sim = MobileSim::new(&map, robots.as_ref(), step.as_ref(), seed as u64)
        .map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(sim))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Advance the simulation by one step"
)]
fn mobile_sim_step(container: &mut ContainerImpl<MobileSim>) -> JuizResult<()> {
    container.step();
    Ok(())
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Set the target velocity of a robot (robot frame)"
    arguments = {
        default = {
            velocity = { linear = { x = 0.0, y = 0.0 }, angular = 0.0 }
            index = 0
        }
    }
)]
fn mobile_sim_set_velocity(
    container: &mut ContainerImpl<MobileSim>,
    velocity: Twist2D,
    index: i64,
) -> JuizResult<()> {
    container
        .set_velocity(index, &velocity)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Pose of a robot in the map frame"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_pose(
    container: &mut ContainerImpl<MobileSim>,
    index: i64,
) -> JuizResult<Pose2DStamped> {
    container.pose(index).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Odometry of a robot: time, pose, twist and travelled distance"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_odometry(
    container: &mut ContainerImpl<MobileSim>,
    index: i64,
) -> JuizResult<Value> {
    container.odometry(index).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "LiDAR scan of a robot"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_scan(
    container: &mut ContainerImpl<MobileSim>,
    index: i64,
) -> JuizResult<LaserScan> {
    container.scan(index).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Map image (black: occupied, white: free, gray: unknown)"
)]
fn mobile_sim_get_map(container: &mut ContainerImpl<MobileSim>) -> JuizResult<Capsule> {
    Ok(container.map_image().into())
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Map metadata: resolution, origin, width and height"
)]
fn mobile_sim_get_map_info(container: &mut ContainerImpl<MobileSim>) -> JuizResult<Value> {
    Ok(container.map_info())
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Load another mapconf"
    arguments = { default = { map = "assets/maps/default.yaml" } }
)]
fn mobile_sim_load_map(container: &mut ContainerImpl<MobileSim>, map: String) -> JuizResult<()> {
    container.load_map(&map).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Put a robot at a pose and stop it"
    arguments = {
        default = {
            pose = { position = { x = 0.0, y = 0.0 }, theta = 0.0 }
            index = 0
        }
    }
)]
fn mobile_sim_reset_pose(
    container: &mut ContainerImpl<MobileSim>,
    pose: Pose2D,
    index: i64,
) -> JuizResult<()> {
    container
        .reset_pose(index, &pose)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "State of all robots with their scans (for the browser view)"
)]
fn mobile_sim_get_state(container: &mut ContainerImpl<MobileSim>) -> JuizResult<Value> {
    Ok(container.state())
}

juiz_component_manifest!(
    component_name = "mobile_sim"
    containers = {
        mobile_sim = [
            mobile_sim_step,
            mobile_sim_set_velocity,
            mobile_sim_get_pose,
            mobile_sim_get_odometry,
            mobile_sim_get_scan,
            mobile_sim_get_map,
            mobile_sim_get_map_info,
            mobile_sim_load_map,
            mobile_sim_reset_pose,
            mobile_sim_get_state
        ]
    }
);
