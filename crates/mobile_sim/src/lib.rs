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
//! | `mobile_sim_get_odometry` | `index` | `{time, frame_id, pose, twist, distance}`(pose はオドメトリの姿勢) |
//! | `mobile_sim_get_odom_pose` | `index` | `Pose2DStamped`(odom 座標系。誤差を含む) |
//! | `mobile_sim_get_scan` | `index` | `LaserScan` |
//! | `mobile_sim_get_map` | なし | 画像 |
//! | `mobile_sim_get_map_info` | なし | mapconf の内容と画像の大きさ |
//! | `mobile_sim_load_map` | `map`(mapconf のパス) | なし |
//! | `mobile_sim_reset_pose` | `pose: Pose2D`, `index` | なし |
//! | `mobile_sim_get_state` | なし | 全ロボットの姿勢・速度・スキャン |

pub mod config;
pub mod parts;
pub mod sim;

use juiz_sdk::anyhow::anyhow;
use juiz_sdk::prelude::*;
use juiz_std_types::geometry::{Pose2D, Pose2DStamped, Twist2D};
use juiz_std_types::sensor::LaserScan;

pub use mobile_sim_core::VERSION;
pub use parts::{MobileLidar, MobileRobot, SimHandle};
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
            seed = "Random seed of LiDAR and odometry noise"
            robots = "List of robot settings (drive, radius, max_velocity, max_acceleration, command_timeout, initial_pose, lidar, odometry). [] when robots are added as mobile_robot containers"
            step = "{mode: fixed, dt} or {mode: real_time, max_dt}"
        }
    }
)]
fn mobile_sim(
    name: String,
    map: String,
    seed: i64,
    robots: Option<Value>,
    step: Option<Value>,
) -> JuizResult<Box<SimHandle>> {
    let sim = MobileSim::new(&map, robots.as_ref(), step.as_ref(), seed as u64)
        .map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(
        SimHandle::register(&name, sim).map_err(|e| anyhow!(err(e)))?,
    ))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Advance the simulation by one step"
)]
fn mobile_sim_step(container: &mut ContainerImpl<SimHandle>) -> JuizResult<()> {
    container.sim().step();
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
    container: &mut ContainerImpl<SimHandle>,
    velocity: Twist2D,
    index: i64,
) -> JuizResult<()> {
    container
        .sim()
        .set_velocity(index, &velocity)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Pose of a robot in the map frame"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_pose(
    container: &mut ContainerImpl<SimHandle>,
    index: i64,
) -> JuizResult<Pose2DStamped> {
    container
        .sim()
        .robot_pose(index)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Odometry of a robot: time, pose, twist and travelled distance"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_odometry(
    container: &mut ContainerImpl<SimHandle>,
    index: i64,
) -> JuizResult<Value> {
    container.sim().odometry(index).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Odometry pose of a robot in the odom frame (with odometry errors)"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_odom_pose(
    container: &mut ContainerImpl<SimHandle>,
    index: i64,
) -> JuizResult<Pose2DStamped> {
    container
        .sim()
        .odom_pose(index)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "LiDAR scan of a robot"
    arguments = { default = { index = 0 } }
)]
fn mobile_sim_get_scan(
    container: &mut ContainerImpl<SimHandle>,
    index: i64,
) -> JuizResult<LaserScan> {
    container.sim().scan(index).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Map image (black: occupied, white: free, gray: unknown)"
)]
fn mobile_sim_get_map(container: &mut ContainerImpl<SimHandle>) -> JuizResult<Capsule> {
    Ok(container.sim().map_image().into())
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Map metadata: resolution, origin, width and height"
)]
fn mobile_sim_get_map_info(container: &mut ContainerImpl<SimHandle>) -> JuizResult<Value> {
    Ok(container.sim().map_info())
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "Load another mapconf"
    arguments = { default = { map = "assets/maps/default.yaml" } }
)]
fn mobile_sim_load_map(container: &mut ContainerImpl<SimHandle>, map: String) -> JuizResult<()> {
    container.sim().load_map(&map).map_err(|e| anyhow!(err(e)))
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
    container: &mut ContainerImpl<SimHandle>,
    pose: Pose2D,
    index: i64,
) -> JuizResult<()> {
    container
        .sim()
        .reset_pose(index, &pose)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_sim"
    description = "State of all robots with their scans (for the browser view)"
)]
fn mobile_sim_get_state(container: &mut ContainerImpl<SimHandle>) -> JuizResult<Value> {
    Ok(container.sim().state())
}

// ---- Transformations で使う部品(docs/transformations-plan.md) ----

#[juiz_component_container(
    description = "A robot (mobile base) of a mobile_sim simulator. Writes its true pose and odometry as ContainerPoses"
    arguments = {
        default = {
            sim = "sim0"
            world = "world"
            odom = "odom0"
        }
        description = {
            sim = "Name of the mobile_sim container this robot lives in (write it before this robot)"
            world = "Anchor to write the true pose against (needs a transform world -> this robot)"
            odom = "Anchor to write the odometry against (needs a transform odom -> this robot)"
            drive = "diff_drive or omni"
            radius = "Radius [m]"
            max_velocity = "{vx, vy, omega}"
            max_acceleration = "{vx, vy, omega}"
            command_timeout = "Stop when no velocity command comes for this time [s]"
            initial_pose = "{x, y, theta} in the world"
            odometry = "Odometry errors {linear_noise, angular_noise, drift_noise, linear_scale, angular_scale}"
        }
    }
)]
#[allow(clippy::too_many_arguments)]
fn mobile_robot(
    name: String,
    sim: String,
    world: String,
    odom: String,
    drive: Option<Value>,
    radius: Option<Value>,
    max_velocity: Option<Value>,
    max_acceleration: Option<Value>,
    command_timeout: Option<Value>,
    initial_pose: Option<Value>,
    odometry: Option<Value>,
) -> JuizResult<Box<MobileRobot>> {
    let settings = parts::settings_object(&[
        ("drive", drive),
        ("radius", radius),
        ("max_velocity", max_velocity),
        ("max_acceleration", max_acceleration),
        ("command_timeout", command_timeout),
        ("initial_pose", initial_pose),
        ("odometry", odometry),
    ]);
    let r = MobileRobot::new(&name, &sim, &world, &odom, &settings).map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(r))
}

#[juiz_component_container_process(
    container_type = "mobile_robot"
    description = "Write the true pose (against world) and the odometry (against odom) as ContainerPoses and return the odometry"
)]
fn mobile_robot_update(container: &mut ContainerImpl<MobileRobot>) -> JuizResult<Pose2DStamped> {
    let (truth, odom, time) = container.poses().map_err(|e| anyhow!(err(e)))?;
    let (world_frame, odom_frame) = (container.world_frame.clone(), container.odom_frame.clone());
    container.set_pose(&world_frame, parts::to_pose3(&truth))?;
    container.set_pose(&odom_frame, parts::to_pose3(&odom))?;
    Ok(Pose2DStamped {
        header: sim::header(time, &odom_frame),
        pose: sim::to_pose(&odom),
    })
}

#[juiz_component_container_process(
    container_type = "mobile_robot"
    description = "Set the target velocity of this robot (robot frame)"
    arguments = { default = { velocity = { linear = { x = 0.0, y = 0.0 }, angular = 0.0 } } }
)]
fn mobile_robot_set_velocity(
    container: &mut ContainerImpl<MobileRobot>,
    velocity: Twist2D,
) -> JuizResult<()> {
    let index = container.index as i64;
    container
        .sim()
        .set_velocity(index, &velocity)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_robot"
    description = "Put this robot at a pose of the world and stop it"
    arguments = { default = { pose = { position = { x = 0.0, y = 0.0 }, theta = 0.0 } } }
)]
fn mobile_robot_reset_pose(
    container: &mut ContainerImpl<MobileRobot>,
    pose: Pose2D,
) -> JuizResult<()> {
    let index = container.index as i64;
    container
        .sim()
        .reset_pose(index, &pose)
        .map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container(
    description = "A LiDAR on a mobile_robot. Its mount is the transform from the robot to this container"
    arguments = {
        default = {
            sim = "sim0"
            base = "robot0"
        }
        description = {
            sim = "Name of the mobile_sim container (write it before this LiDAR)"
            base = "Name of the mobile_robot container this LiDAR is on (needs a transform base -> this LiDAR)"
            angle_min = "First beam angle [rad]"
            angle_max = "Last beam angle [rad]"
            num_beams = "Number of beams"
            range_min = "Minimum range [m]"
            range_max = "Maximum range [m]"
            noise_stddev = "Range noise [m]"
            unknown_is_obstacle = "Whether unknown cells stop beams"
        }
    }
)]
#[allow(clippy::too_many_arguments)]
fn mobile_lidar(
    name: String,
    sim: String,
    base: String,
    angle_min: Option<Value>,
    angle_max: Option<Value>,
    num_beams: Option<Value>,
    range_min: Option<Value>,
    range_max: Option<Value>,
    noise_stddev: Option<Value>,
    unknown_is_obstacle: Option<Value>,
) -> JuizResult<Box<MobileLidar>> {
    let settings = parts::settings_object(&[
        ("angle_min", angle_min),
        ("angle_max", angle_max),
        ("num_beams", num_beams),
        ("range_min", range_min),
        ("range_max", range_max),
        ("noise_stddev", noise_stddev),
        ("unknown_is_obstacle", unknown_is_obstacle),
    ]);
    let l = MobileLidar::new(&name, &sim, &base, &settings).map_err(|e| anyhow!(err(e)))?;
    Ok(Box::new(l))
}

#[juiz_component_container_process(
    container_type = "mobile_lidar"
    description = "Scan from this LiDAR. The mount is read from the transform base -> this LiDAR; frame_id is this container"
)]
fn mobile_lidar_scan(container: &mut ContainerImpl<MobileLidar>) -> JuizResult<LaserScan> {
    let base = container.base.clone();
    let mount = parts::to_pose2(&container.pose(&base)?.pose);
    let (index, name, setting) = (
        container.index,
        container.name.clone(),
        container.setting.clone(),
    );
    let mut s = container.sim();
    s.set_lidar(index, &setting, mount)
        .map_err(|e| anyhow!(err(e)))?;
    s.scan_as(index, &name).map_err(|e| anyhow!(err(e)))
}

#[juiz_component_container_process(
    container_type = "mobile_lidar"
    description = "Mount of this LiDAR: its ContainerPose seen from the robot (juiz_std.geometry.PoseStamped)"
)]
fn mobile_lidar_mount(container: &mut ContainerImpl<MobileLidar>) -> JuizResult<Capsule> {
    let base = container.base.clone();
    Ok(container.pose(&base)?.into())
}

juiz_component_manifest!(
    component_name = "mobile_sim"
    containers = {
        mobile_sim = [
            mobile_sim_step,
            mobile_sim_set_velocity,
            mobile_sim_get_pose,
            mobile_sim_get_odometry,
            mobile_sim_get_odom_pose,
            mobile_sim_get_scan,
            mobile_sim_get_map,
            mobile_sim_get_map_info,
            mobile_sim_load_map,
            mobile_sim_reset_pose,
            mobile_sim_get_state
        ]
        mobile_robot = [
            mobile_robot_update,
            mobile_robot_set_velocity,
            mobile_robot_reset_pose
        ]
        mobile_lidar = [
            mobile_lidar_scan,
            mobile_lidar_mount
        ]
    }
);
