//! mobile_sim のシミュレーション本体。
//!
//! 車両モデル、マップと mapconf、衝突判定、LiDAR のレイキャストを持つ。
//! juiz には依存せず、`cargo test` だけで検証できるようにしてある。
//!
//! ```
//! use mobile_sim_core::{OccupancyGrid, RobotConfig, Twist2D, World};
//!
//! let mut world = World::new(OccupancyGrid::new_free(100, 100, 0.1, (-5.0, -5.0)));
//! let id = world.spawn(RobotConfig::default()).unwrap();
//! world.set_velocity(id, Twist2D::new(0.5, 0.0, 0.0)).unwrap();
//! world.step(0.1);
//! let scan = world.scan(id).unwrap();
//! assert_eq!(scan.ranges.len(), 200);
//! ```

pub mod clock;
pub mod collision;
pub mod error;
pub mod geometry;
pub mod lidar;
pub mod map;
pub mod vehicle;
pub mod world;

pub use clock::StepClock;
pub use error::{SimError, SimResult};
pub use geometry::{normalize_angle, Pose2D, Twist2D};
pub use lidar::{LidarConfig, Scan};
pub use map::{Cell, MapConf, OccupancyGrid};
pub use vehicle::{DriveKind, Vehicle, VehicleConfig};
pub use world::{Robot, RobotConfig, RobotState, World};

/// このクレートの版
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
