//! 衝突判定: ロボットを円とし、円に重なる格子に障害物があるかを見る

use crate::map::OccupancyGrid;

/// 中心 `(x, y)`、半径 `radius` [m] の円が、障害物の格子(とマップの外)に重なるか
pub fn circle_collides(
    map: &OccupancyGrid,
    x: f64,
    y: f64,
    radius: f64,
    unknown_is_obstacle: bool,
) -> bool {
    let res = map.resolution();
    let (gx, gy) = map.world_to_grid(x, y);
    let r = radius / res;
    let (i0, i1) = ((gx - r).floor() as i64, (gx + r).floor() as i64);
    let (j0, j1) = ((gy - r).floor() as i64, (gy + r).floor() as i64);
    for j in j0..=j1 {
        for i in i0..=i1 {
            if !map.is_blocked(i, j, unknown_is_obstacle) {
                continue;
            }
            // 格子 [i, i+1) x [j, j+1) の中で円の中心に一番近い点
            let nx = gx.clamp(i as f64, (i + 1) as f64);
            let ny = gy.clamp(j as f64, (j + 1) as f64);
            let d2 = (nx - gx).powi(2) + (ny - gy).powi(2);
            if d2 < r * r || (r == 0.0 && d2 == 0.0) {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::Cell;

    #[test]
    fn touches_obstacle_by_distance() {
        // 1 m 格子、(5, 5) の格子だけ障害物
        let mut g = OccupancyGrid::new_free(10, 10, 1.0, (0.0, 0.0));
        g.set_cell(5, 5, Cell::Occupied);
        assert!(circle_collides(&g, 5.5, 5.5, 0.1, true));
        assert!(!circle_collides(&g, 4.5, 5.5, 0.4, true));
        assert!(circle_collides(&g, 4.5, 5.5, 0.6, true));
        // 角へは斜めの距離で判定する
        assert!(!circle_collides(&g, 4.5, 4.5, 0.7, true));
        assert!(circle_collides(&g, 4.5, 4.5, 0.75, true));
        // 半径 0 は点
        assert!(circle_collides(&g, 5.0, 5.0, 0.0, true));
        assert!(!circle_collides(&g, 4.99, 5.5, 0.0, true));
    }

    #[test]
    fn outside_and_unknown() {
        let mut g = OccupancyGrid::new_free(10, 10, 1.0, (0.0, 0.0));
        assert!(circle_collides(&g, 0.2, 5.0, 0.3, true));
        assert!(!circle_collides(&g, 0.4, 5.0, 0.3, true));
        g.set_cell(5, 5, Cell::Unknown);
        assert!(circle_collides(&g, 5.5, 5.5, 0.1, true));
        assert!(!circle_collides(&g, 5.5, 5.5, 0.1, false));
    }
}
