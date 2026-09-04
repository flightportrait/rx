//! Measured gain control (`--gain auto`). Every window (10 s) the radio
//! looks at how often samples hit full scale, whether strong frames
//! clipped, and where the noise floor sits, and moves the tuner gain one
//! step toward the operating point: no clipping, noise floor above the
//! quantization floor. A step needs two consecutive windows agreeing and
//! is followed by a hold, so the loop cannot oscillate on a passing
//! aircraft. The decision is a pure function of the window and the
//! controller's memory, so it is tested without a dongle.

/// What one window of samples and frames looked like.
#[derive(Debug, Clone, Copy)]
pub struct Window {
    /// Fraction of samples with |I| or |Q| at or near full scale.
    pub clip_fraction: f64,
    /// A decoded frame reported a signal byte at or above 250.
    pub strong_clip: bool,
    /// Mean magnitude of the samples, raw units (0..181).
    pub noise: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Vote {
    Down,
    Up,
    Hold,
}

/// Clipping above this fraction of samples steps the gain down.
pub const CLIP_DOWN: f64 = 0.001;
/// Clipping below this fraction allows a step up.
pub const CLIP_OK: f64 = 0.0002;
/// Noise floor (raw magnitude units) below which the gain steps up:
/// below it, weak frames are lost to quantization rather than to noise.
pub const NOISE_LOW: f32 = 1.4;
/// Seconds between windows.
pub const WINDOW_S: f64 = 10.0;
/// Seconds to hold after a change.
pub const HOLD_S: f64 = 20.0;

/// The vote of one window on its own.
pub fn vote(w: &Window) -> Vote {
    if w.strong_clip || w.clip_fraction > CLIP_DOWN {
        Vote::Down
    } else if w.clip_fraction < CLIP_OK && w.noise < NOISE_LOW {
        Vote::Up
    } else {
        Vote::Hold
    }
}

/// Gain loop state: the tuner's gain steps (tenths of a dB, ascending),
/// the current step, the previous window's vote, and the hold deadline.
pub struct Controller {
    steps: Vec<i32>,
    idx: usize,
    last_vote: Option<Vote>,
    hold_until: f64,
}

impl Controller {
    /// Start at `start_tenths` (snapped to a step) with no hold.
    pub fn new(steps: Vec<i32>, start_tenths: i32) -> Self {
        let idx = steps
            .iter()
            .enumerate()
            .min_by_key(|(_, &g)| (g - start_tenths).abs())
            .map(|(i, _)| i)
            .unwrap_or(0);
        Controller {
            steps,
            idx,
            last_vote: None,
            hold_until: 0.0,
        }
    }

    pub fn current(&self) -> i32 {
        self.steps.get(self.idx).copied().unwrap_or(0)
    }

    /// Offer a window at time `now` (seconds). Returns the new gain in
    /// tenths of a dB when a step is taken.
    pub fn step(&mut self, w: &Window, now: f64) -> Option<i32> {
        if self.steps.len() < 2 || now < self.hold_until {
            self.last_vote = None;
            return None;
        }
        let v = vote(w);
        let agreed = self.last_vote == Some(v);
        self.last_vote = Some(v);
        if !agreed {
            return None;
        }
        let new_idx = match v {
            Vote::Down if self.idx > 0 => self.idx - 1,
            Vote::Up if self.idx + 1 < self.steps.len() => self.idx + 1,
            _ => return None,
        };
        self.idx = new_idx;
        self.last_vote = None;
        self.hold_until = now + HOLD_S;
        Some(self.steps[self.idx])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(clip: f64, strong: bool, noise: f32) -> Window {
        Window {
            clip_fraction: clip,
            strong_clip: strong,
            noise,
        }
    }

    #[test]
    fn votes() {
        assert_eq!(vote(&w(0.01, false, 2.0)), Vote::Down);
        assert_eq!(vote(&w(0.0, true, 2.0)), Vote::Down);
        assert_eq!(vote(&w(0.0, false, 1.0)), Vote::Up);
        assert_eq!(vote(&w(0.0005, false, 1.0)), Vote::Hold); // clipping not low enough to go up
        assert_eq!(vote(&w(0.0, false, 2.0)), Vote::Hold);
    }

    #[test]
    fn two_windows_must_agree_then_hold() {
        let mut c = Controller::new(vec![0, 100, 200, 300, 400, 496], 496);
        assert_eq!(c.current(), 496);
        assert_eq!(c.step(&w(0.01, false, 2.0), 0.0), None); // first vote down
        assert_eq!(c.step(&w(0.0, false, 2.0), 10.0), None); // hold breaks the streak
        assert_eq!(c.step(&w(0.01, false, 2.0), 20.0), None);
        assert_eq!(c.step(&w(0.01, false, 2.0), 30.0), Some(400)); // agreed: step down
        // held for 20 s: votes during the hold are ignored
        assert_eq!(c.step(&w(0.01, false, 2.0), 40.0), None);
        assert_eq!(c.step(&w(0.01, false, 2.0), 45.0), None);
        // after the hold, two more agreeing windows step again
        assert_eq!(c.step(&w(0.01, false, 2.0), 51.0), None);
        assert_eq!(c.step(&w(0.01, false, 2.0), 61.0), Some(300));
    }

    #[test]
    fn steps_up_when_quiet_and_stops_at_the_ends() {
        let mut c = Controller::new(vec![0, 100, 200], 100);
        assert_eq!(c.step(&w(0.0, false, 1.0), 0.0), None);
        assert_eq!(c.step(&w(0.0, false, 1.0), 10.0), Some(200));
        assert_eq!(c.step(&w(0.0, false, 1.0), 31.0), None);
        assert_eq!(c.step(&w(0.0, false, 1.0), 41.0), None); // top step: nowhere to go
        let mut c = Controller::new(vec![0, 100], 0);
        assert_eq!(c.step(&w(0.1, false, 2.0), 0.0), None);
        assert_eq!(c.step(&w(0.1, false, 2.0), 10.0), None); // bottom step
    }

    #[test]
    fn start_snaps_to_a_step() {
        let c = Controller::new(vec![0, 90, 200, 496], 100);
        assert_eq!(c.current(), 90);
    }
}
