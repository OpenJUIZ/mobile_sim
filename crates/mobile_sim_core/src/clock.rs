//! `World::step` に渡す dt の決め方

use std::time::Instant;

/// dt の決め方
#[derive(Clone, Debug)]
pub enum StepClock {
    /// 毎回同じ dt [s] で進める(再現性がある)
    Fixed(f64),
    /// 前回呼ばれてからの実時間で進める。止まっていた後に大きく飛ばないよう `max_dt` [s] で切る
    RealTime { last: Option<Instant>, max_dt: f64 },
}

impl StepClock {
    pub fn real_time(max_dt: f64) -> Self {
        StepClock::RealTime { last: None, max_dt }
    }

    /// 今回の dt を返す。実時間の初回は 0
    pub fn tick(&mut self) -> f64 {
        self.tick_at(Instant::now())
    }

    fn tick_at(&mut self, now: Instant) -> f64 {
        match self {
            StepClock::Fixed(dt) => *dt,
            StepClock::RealTime { last, max_dt } => {
                let dt = last.map_or(0.0, |l| now.duration_since(l).as_secs_f64());
                *last = Some(now);
                dt.min(*max_dt)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn fixed_and_real_time() {
        let mut f = StepClock::Fixed(0.01);
        assert_eq!(f.tick(), 0.01);
        assert_eq!(f.tick(), 0.01);

        let mut r = StepClock::real_time(0.1);
        let t0 = Instant::now();
        assert_eq!(r.tick_at(t0), 0.0);
        assert!((r.tick_at(t0 + Duration::from_millis(20)) - 0.02).abs() < 1e-9);
        assert_eq!(r.tick_at(t0 + Duration::from_secs(5)), 0.1);
    }
}
