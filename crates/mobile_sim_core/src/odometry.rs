//! 車輪オドメトリの誤差のモデル
//!
//! 真の動きを 1 step ごとにロボット座標系の差分 (並進, 回転) にし、
//! 倍率の誤差(系統誤差)と正規分布の誤差(偶然誤差)を足して積算する。
//! 偶然誤差の分散は動いた量に比例させる(1 m 走ると標準偏差が `linear_noise` [m])。
//! こうすると step の刻み dt を変えても誤差の大きさが変わらない。

use rand::Rng;
use rand_distr::{Distribution, StandardNormal};

use crate::error::{SimError, SimResult};
use crate::geometry::{normalize_angle, Pose2D};

/// オドメトリの誤差の設定。既定値は誤差無し(真の姿勢と同じになる)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OdometryConfig {
    /// 並進の倍率(1.0 で誤差無し。車輪径の誤差など)
    pub linear_scale: f64,
    /// 回転の倍率(1.0 で誤差無し。トレッド幅の誤差など)
    pub angular_scale: f64,
    /// 1 m 走ったときの並進の誤差の標準偏差 [m/√m]
    pub linear_noise: f64,
    /// 1 rad 回ったときの向きの誤差の標準偏差 [rad/√rad]
    pub angular_noise: f64,
    /// 1 m 走ったときの向きの誤差の標準偏差 [rad/√m](直進中の向きのずれ)
    pub drift_noise: f64,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            linear_scale: 1.0,
            angular_scale: 1.0,
            linear_noise: 0.0,
            angular_noise: 0.0,
            drift_noise: 0.0,
        }
    }
}

impl OdometryConfig {
    pub fn validate(&self) -> SimResult<()> {
        let ok = |v: f64| v.is_finite() && v >= 0.0;
        if !(ok(self.linear_scale) && self.linear_scale > 0.0)
            || !(ok(self.angular_scale) && self.angular_scale > 0.0)
            || !ok(self.linear_noise)
            || !ok(self.angular_noise)
            || !ok(self.drift_noise)
        {
            return Err(SimError::InvalidConfig(format!(
                "odometry scales must be positive and noises must not be negative: {self:?}"
            )));
        }
        Ok(())
    }

    /// 真の動き `from` → `to` に誤差を足して、オドメトリの姿勢 `odom` を進める
    pub fn advance<R: Rng + ?Sized>(
        &self,
        odom: &Pose2D,
        from: &Pose2D,
        to: &Pose2D,
        rng: &mut R,
    ) -> Pose2D {
        // from の座標系で見た to
        let (s, c) = from.theta.sin_cos();
        let (gx, gy) = (to.x - from.x, to.y - from.y);
        let dx = c * gx + s * gy;
        let dy = -s * gx + c * gy;
        let dth = normalize_angle(to.theta - from.theta);
        let trans = dx.hypot(dy);
        let rot = dth.abs();

        let mut n = || -> f64 { StandardNormal.sample(rng) };
        let lin_k = self.linear_scale
            * (1.0
                + if trans > 0.0 {
                    self.linear_noise * n() / trans.sqrt()
                } else {
                    0.0
                });
        let rot_err = self.angular_noise * rot.sqrt() * n() + self.drift_noise * trans.sqrt() * n();
        let delta = Pose2D::new(dx * lin_k, dy * lin_k, dth * self.angular_scale + rot_err);
        odom.compose(&delta)
    }

    /// 誤差が無い設定か
    pub fn is_exact(&self) -> bool {
        *self == Self::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    /// 1 m を n 回に分けて直進したときの (x, y, theta) の誤差
    fn straight(cfg: &OdometryConfig, steps: usize, seed: u64) -> Pose2D {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut odom = Pose2D::default();
        let mut truth = Pose2D::default();
        for _ in 0..steps {
            let next = truth.compose(&Pose2D::new(1.0 / steps as f64, 0.0, 0.0));
            odom = cfg.advance(&odom, &truth, &next, &mut rng);
            truth = next;
        }
        odom
    }

    #[test]
    fn exact_by_default() {
        let cfg = OdometryConfig::default();
        assert!(cfg.is_exact());
        let mut rng = StdRng::seed_from_u64(0);
        let from = Pose2D::new(1.0, 2.0, 0.3);
        let to = Pose2D::new(1.5, 2.2, 0.9);
        let odom = Pose2D::new(-1.0, 0.5, -0.4);
        let got = cfg.advance(&odom, &from, &to, &mut rng);
        // odom の座標系で、真の動きと同じだけ動く
        let (s, c) = from.theta.sin_cos();
        let (gx, gy) = (to.x - from.x, to.y - from.y);
        let want = odom.compose(&Pose2D::new(c * gx + s * gy, -s * gx + c * gy, 0.6));
        assert!((got.x - want.x).abs() < 1e-12 && (got.y - want.y).abs() < 1e-12);
        assert!((got.theta - want.theta).abs() < 1e-12);
    }

    #[test]
    fn scale_is_systematic() {
        let cfg = OdometryConfig {
            linear_scale: 1.1,
            ..Default::default()
        };
        let p = straight(&cfg, 10, 0);
        assert!((p.x - 1.1).abs() < 1e-12 && p.y.abs() < 1e-12);
    }

    #[test]
    fn noise_does_not_depend_on_step_size() {
        let cfg = OdometryConfig {
            linear_noise: 0.05,
            drift_noise: 0.02,
            ..Default::default()
        };
        // 1 m 走ったときの x の誤差の標準偏差は 0.05、向きは 0.02(刻みに依らない)
        for steps in [10, 1000] {
            let trials = 400;
            let (mut sx, mut st) = (0.0, 0.0);
            for seed in 0..trials {
                let p = straight(&cfg, steps, seed);
                sx += (p.x - 1.0).powi(2);
                st += p.theta.powi(2);
            }
            let (sdx, sdt) = ((sx / trials as f64).sqrt(), (st / trials as f64).sqrt());
            assert!((sdx - 0.05).abs() < 0.01, "steps {steps}: x sd {sdx}");
            assert!((sdt - 0.02).abs() < 0.004, "steps {steps}: theta sd {sdt}");
        }
    }

    #[test]
    fn rejects_bad_values() {
        for bad in [
            OdometryConfig {
                linear_scale: 0.0,
                ..Default::default()
            },
            OdometryConfig {
                angular_scale: f64::NAN,
                ..Default::default()
            },
            OdometryConfig {
                linear_noise: -0.1,
                ..Default::default()
            },
        ] {
            assert!(bad.validate().is_err());
        }
        assert!(OdometryConfig::default().validate().is_ok());
    }
}
