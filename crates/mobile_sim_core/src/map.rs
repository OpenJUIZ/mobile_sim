//! マップ: mapconf(ROS の map_server と同じ YAML)と占有格子
//!
//! 座標の決まり(map_server と同じ):
//! - `origin` は画像の左下の画素の、左下の角の世界座標 [m, m, rad]
//! - 画像の上の行ほど y が大きい
//! - 格子の添字 `(i, j)` は左下の画素を `(0, 0)` とし、`i` が x 方向、`j` が y 方向

use std::path::{Path, PathBuf};

use image::GrayImage;
use serde::{Deserialize, Serialize};

use crate::error::{SimError, SimResult};

/// mapconf の中身
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MapConf {
    /// 画像のパス。相対パスは mapconf のあるディレクトリから
    pub image: PathBuf,
    /// 1 画素の大きさ [m/px]
    pub resolution: f64,
    /// 左下の画素の世界座標 [x, y, yaw]
    pub origin: [f64; 3],
    #[serde(default = "default_occupied_thresh")]
    pub occupied_thresh: f64,
    #[serde(default = "default_free_thresh")]
    pub free_thresh: f64,
    /// 0 なら黒が障害物、1 なら白が障害物
    #[serde(default)]
    pub negate: u8,
}

fn default_occupied_thresh() -> f64 {
    0.65
}

fn default_free_thresh() -> f64 {
    0.196
}

impl MapConf {
    /// YAML の文字列から読む
    pub fn from_yaml(text: &str, path: &Path) -> SimResult<Self> {
        let conf: MapConf = serde_yaml::from_str(text).map_err(|source| SimError::MapConf {
            path: path.to_owned(),
            source,
        })?;
        conf.validate()?;
        Ok(conf)
    }

    /// ファイルから読む
    pub fn load(path: &Path) -> SimResult<Self> {
        let text = std::fs::read_to_string(path).map_err(|source| SimError::Io {
            path: path.to_owned(),
            source,
        })?;
        Self::from_yaml(&text, path)
    }

    fn validate(&self) -> SimResult<()> {
        if !(self.resolution > 0.0 && self.resolution.is_finite()) {
            return Err(SimError::InvalidMap(format!(
                "resolution must be positive, got {}",
                self.resolution
            )));
        }
        if !(0.0..=1.0).contains(&self.free_thresh)
            || !(0.0..=1.0).contains(&self.occupied_thresh)
            || self.free_thresh > self.occupied_thresh
        {
            return Err(SimError::InvalidMap(format!(
                "thresholds must satisfy 0 <= free_thresh ({}) <= occupied_thresh ({}) <= 1",
                self.free_thresh, self.occupied_thresh
            )));
        }
        if self.negate > 1 {
            return Err(SimError::InvalidMap(format!(
                "negate must be 0 or 1, got {}",
                self.negate
            )));
        }
        if self.origin[2] != 0.0 {
            return Err(SimError::InvalidMap(
                "rotated origin (origin[2] != 0) is not supported yet".to_owned(),
            ));
        }
        Ok(())
    }
}

/// 格子 1 つの状態
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    Free,
    Occupied,
    Unknown,
}

/// 占有格子
#[derive(Clone, Debug)]
pub struct OccupancyGrid {
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    /// `j * width + i` に `(i, j)` の状態。`j = 0` が下の行
    cells: Vec<Cell>,
}

impl OccupancyGrid {
    /// mapconf のファイルから、画像も読んで作る
    pub fn load(mapconf_path: &Path) -> SimResult<(Self, MapConf)> {
        let conf = MapConf::load(mapconf_path)?;
        let image_path = if conf.image.is_absolute() {
            conf.image.clone()
        } else {
            mapconf_path
                .parent()
                .unwrap_or(Path::new("."))
                .join(&conf.image)
        };
        let img = image::open(&image_path)
            .map_err(|source| SimError::Image {
                path: image_path.clone(),
                source,
            })?
            .to_luma8();
        Ok((Self::from_image(&img, &conf)?, conf))
    }

    /// グレースケール画像と mapconf から作る(map_server の trinary モードと同じ判定)
    pub fn from_image(img: &GrayImage, conf: &MapConf) -> SimResult<Self> {
        let (width, height) = (img.width() as usize, img.height() as usize);
        if width == 0 || height == 0 {
            return Err(SimError::InvalidMap("map image is empty".to_owned()));
        }
        let mut cells = vec![Cell::Unknown; width * height];
        for (px, py, luma) in img.enumerate_pixels() {
            let v = luma.0[0] as f64 / 255.0;
            let occ = if conf.negate == 0 { 1.0 - v } else { v };
            let cell = if occ > conf.occupied_thresh {
                Cell::Occupied
            } else if occ < conf.free_thresh {
                Cell::Free
            } else {
                Cell::Unknown
            };
            let j = height - 1 - py as usize;
            cells[j * width + px as usize] = cell;
        }
        Ok(Self {
            width,
            height,
            resolution: conf.resolution,
            origin_x: conf.origin[0],
            origin_y: conf.origin[1],
            cells,
        })
    }

    /// 全部 `Free` の格子を作る(テストと、マップ無しで動かすとき用)
    pub fn new_free(width: usize, height: usize, resolution: f64, origin: (f64, f64)) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x: origin.0,
            origin_y: origin.1,
            cells: vec![Cell::Free; width * height],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// 左下の角の世界座標
    pub fn origin(&self) -> (f64, f64) {
        (self.origin_x, self.origin_y)
    }

    /// 世界座標を、格子の単位の連続座標(左下の角が 0)に直す
    pub fn world_to_grid(&self, x: f64, y: f64) -> (f64, f64) {
        (
            (x - self.origin_x) / self.resolution,
            (y - self.origin_y) / self.resolution,
        )
    }

    /// 世界座標の点を含む格子の添字。マップの外なら `None`
    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let (gx, gy) = self.world_to_grid(x, y);
        self.grid_index(gx.floor() as i64, gy.floor() as i64)
    }

    /// 格子の中心の世界座標
    pub fn cell_center(&self, i: usize, j: usize) -> (f64, f64) {
        (
            self.origin_x + (i as f64 + 0.5) * self.resolution,
            self.origin_y + (j as f64 + 0.5) * self.resolution,
        )
    }

    fn grid_index(&self, i: i64, j: i64) -> Option<(usize, usize)> {
        if i < 0 || j < 0 || i as usize >= self.width || j as usize >= self.height {
            None
        } else {
            Some((i as usize, j as usize))
        }
    }

    /// 格子の状態。マップの外は `None`
    pub fn cell(&self, i: i64, j: i64) -> Option<Cell> {
        self.grid_index(i, j)
            .map(|(i, j)| self.cells[j * self.width + i])
    }

    pub fn set_cell(&mut self, i: usize, j: usize, cell: Cell) {
        assert!(i < self.width && j < self.height);
        self.cells[j * self.width + i] = cell;
    }

    /// 格子 `(i, j)` を障害物として扱うか。マップの外は障害物とする
    pub fn is_blocked(&self, i: i64, j: i64, unknown_is_obstacle: bool) -> bool {
        match self.cell(i, j) {
            None | Some(Cell::Occupied) => true,
            Some(Cell::Unknown) => unknown_is_obstacle,
            Some(Cell::Free) => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::Luma;

    fn conf() -> MapConf {
        MapConf {
            image: "map.png".into(),
            resolution: 0.1,
            origin: [-1.0, -2.0, 0.0],
            occupied_thresh: 0.65,
            free_thresh: 0.196,
            negate: 0,
        }
    }

    #[test]
    fn parse_ros_mapconf() {
        let yaml = "image: room.pgm\nresolution: 0.05\norigin: [-5.0, -5.0, 0.0]\noccupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\n";
        let c = MapConf::from_yaml(yaml, Path::new("room.yaml")).unwrap();
        assert_eq!(c.image, PathBuf::from("room.pgm"));
        assert_eq!(c.resolution, 0.05);
        assert_eq!(c.origin, [-5.0, -5.0, 0.0]);
    }

    #[test]
    fn mapconf_defaults_and_validation() {
        let c = MapConf::from_yaml(
            "image: a.png\nresolution: 0.1\norigin: [0, 0, 0]\n",
            Path::new("a.yaml"),
        )
        .unwrap();
        assert_eq!(c.occupied_thresh, 0.65);
        assert_eq!(c.free_thresh, 0.196);
        assert_eq!(c.negate, 0);

        for bad in [
            "image: a.png\nresolution: 0\norigin: [0, 0, 0]\n",
            "image: a.png\nresolution: 0.1\norigin: [0, 0, 0.5]\n",
            "image: a.png\nresolution: 0.1\norigin: [0, 0, 0]\nfree_thresh: 0.9\n",
            "image: a.png\nresolution: 0.1\n",
        ] {
            assert!(
                MapConf::from_yaml(bad, Path::new("a.yaml")).is_err(),
                "{bad}"
            );
        }
    }

    #[test]
    fn image_rows_are_flipped_and_thresholded() {
        // 2x2: 左上が黒、右上が灰色、下の行は白
        let mut img = GrayImage::from_pixel(2, 2, Luma([255]));
        img.put_pixel(0, 0, Luma([0]));
        img.put_pixel(1, 0, Luma([128]));
        let g = OccupancyGrid::from_image(&img, &conf()).unwrap();
        assert_eq!(g.cell(0, 1), Some(Cell::Occupied));
        assert_eq!(g.cell(1, 1), Some(Cell::Unknown));
        assert_eq!(g.cell(0, 0), Some(Cell::Free));
        assert_eq!(g.cell(1, 0), Some(Cell::Free));
        assert_eq!(g.cell(2, 0), None);

        let mut negated = conf();
        negated.negate = 1;
        let g = OccupancyGrid::from_image(&img, &negated).unwrap();
        assert_eq!(g.cell(0, 1), Some(Cell::Free));
        assert_eq!(g.cell(0, 0), Some(Cell::Occupied));
    }

    #[test]
    fn world_and_cell_coordinates() {
        let g = OccupancyGrid::new_free(20, 40, 0.1, (-1.0, -2.0));
        assert_eq!(g.world_to_cell(-1.0, -2.0), Some((0, 0)));
        assert_eq!(g.world_to_cell(-0.95, -1.95), Some((0, 0)));
        assert_eq!(g.world_to_cell(0.0, 0.0), Some((10, 20)));
        assert_eq!(g.world_to_cell(0.999, 1.999), Some((19, 39)));
        assert_eq!(g.world_to_cell(1.0, 0.0), None);
        assert_eq!(g.world_to_cell(-1.01, 0.0), None);
        let (cx, cy) = g.cell_center(10, 20);
        assert!((cx - 0.05).abs() < 1e-12 && (cy - 0.05).abs() < 1e-12);
    }

    #[test]
    fn load_from_files() {
        let dir = tempfile::tempdir().unwrap();
        let mut img = GrayImage::from_pixel(4, 3, Luma([255]));
        img.put_pixel(3, 0, Luma([0]));
        img.save(dir.path().join("m.png")).unwrap();
        let yaml = dir.path().join("m.yaml");
        std::fs::write(
            &yaml,
            "image: m.png\nresolution: 0.5\norigin: [0.0, 0.0, 0.0]\n",
        )
        .unwrap();
        let (g, c) = OccupancyGrid::load(&yaml).unwrap();
        assert_eq!((g.width(), g.height()), (4, 3));
        assert_eq!(c.resolution, 0.5);
        // 画像の右上 (3, 0) は格子の (3, 2)
        assert_eq!(g.cell(3, 2), Some(Cell::Occupied));
        assert_eq!(g.world_to_cell(1.75, 1.25), Some((3, 2)));

        std::fs::write(
            &yaml,
            "image: missing.png\nresolution: 0.5\norigin: [0, 0, 0]\n",
        )
        .unwrap();
        assert!(matches!(
            OccupancyGrid::load(&yaml),
            Err(SimError::Image { .. })
        ));
    }
}
