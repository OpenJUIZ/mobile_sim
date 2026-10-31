//! エラー

use std::path::PathBuf;

#[derive(Debug, thiserror::Error)]
pub enum SimError {
    #[error("cannot read {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("invalid mapconf {path}: {source}")]
    MapConf {
        path: PathBuf,
        source: serde_yaml::Error,
    },
    #[error("cannot load map image {path}: {source}")]
    Image {
        path: PathBuf,
        source: image::ImageError,
    },
    #[error("invalid map: {0}")]
    InvalidMap(String),
    #[error("invalid config: {0}")]
    InvalidConfig(String),
    #[error("no robot with index {0}")]
    NoSuchRobot(usize),
}

pub type SimResult<T> = Result<T, SimError>;
