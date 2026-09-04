//! stats.json in readsb's shape for the fields the radio has: totals since
//! start and the last minute, written every 10 s beside aircraft.json.
//! Signal and noise are dBFS with full scale at magnitude 181 (the
//! largest magnitude an 8-bit I/Q sample can have).

use std::collections::VecDeque;
use std::io::Write;

#[derive(Default, Clone, Copy)]
struct Bucket {
    samples: u64,
    dropped: u64,
    clean: u64,
    repaired: u64,
    strong: u64,
    /// Sum of frame signal levels (raw magnitude units) and their count.
    signal_sum: f64,
    signal_n: u64,
    /// Sum of per-block mean magnitudes and their count.
    noise_sum: f64,
    noise_n: u64,
}

impl Bucket {
    fn add(&mut self, o: &Bucket) {
        self.samples += o.samples;
        self.dropped += o.dropped;
        self.clean += o.clean;
        self.repaired += o.repaired;
        self.strong += o.strong;
        self.signal_sum += o.signal_sum;
        self.signal_n += o.signal_n;
        self.noise_sum += o.noise_sum;
        self.noise_n += o.noise_n;
    }
}

pub struct Stats {
    start: f64,
    total: Bucket,
    current: Bucket,
    /// One bucket per second, the last 60.
    minute: VecDeque<Bucket>,
    last_written: f64,
    pub gain_db: Option<f32>,
}

fn dbfs(level: f64) -> f64 {
    if level <= 0.0 {
        -50.0
    } else {
        (20.0 * (level / 181.0).log10()).max(-50.0)
    }
}

impl Stats {
    pub fn new(now: f64, gain_db: Option<f32>) -> Self {
        Stats {
            start: now,
            total: Bucket::default(),
            current: Bucket::default(),
            minute: VecDeque::with_capacity(61),
            last_written: 0.0,
            gain_db,
        }
    }

    pub fn samples(&mut self, n: u64) {
        self.current.samples += n;
    }

    pub fn dropped(&mut self, n: u64) {
        self.current.dropped += n;
    }

    /// A frame: `level` in raw magnitude units, `fixed` repaired bits.
    pub fn frame(&mut self, level: f32, fixed: u8) {
        if fixed == 0 {
            self.current.clean += 1;
        } else {
            self.current.repaired += 1;
        }
        if level >= 178.0 {
            self.current.strong += 1;
        }
        self.current.signal_sum += level as f64;
        self.current.signal_n += 1;
    }

    /// Mean magnitude of a block, raw units: the noise floor on a quiet sky.
    pub fn noise(&mut self, mean_level: f32) {
        self.current.noise_sum += mean_level as f64;
        self.current.noise_n += 1;
    }

    /// Call once a second; writes stats.json every 10 s into `dir`.
    pub fn tick(&mut self, now: f64, dir: Option<&std::path::Path>) {
        let cur = std::mem::take(&mut self.current);
        self.total.add(&cur);
        self.minute.push_back(cur);
        while self.minute.len() > 60 {
            self.minute.pop_front();
        }
        if now - self.last_written >= 10.0 {
            self.last_written = now;
            if let Some(d) = dir {
                if let Err(e) = self.write(d, now) {
                    eprintln!("rx: stats.json: {e}");
                }
            }
        }
    }

    fn section(&self, b: &Bucket, start: f64, end: f64) -> String {
        let signal = if b.signal_n > 0 {
            dbfs(b.signal_sum / b.signal_n as f64)
        } else {
            -50.0
        };
        let noise = if b.noise_n > 0 {
            dbfs(b.noise_sum / b.noise_n as f64)
        } else {
            -50.0
        };
        let gain = match self.gain_db {
            Some(g) => format!("{g:.1}"),
            None => "null".into(),
        };
        format!(
            "{{\"start\":{start:.1},\"end\":{end:.1},\"local\":{{\"samples_processed\":{},\"samples_dropped\":{},\"accepted\":[{},{}],\"strong_signals\":{},\"signal\":{signal:.1},\"noise\":{noise:.1},\"gain_db\":{gain}}},\"messages\":{}}}",
            b.samples,
            b.dropped,
            b.clean,
            b.repaired,
            b.strong,
            b.clean + b.repaired
        )
    }

    fn write(&self, dir: &std::path::Path, now: f64) -> std::io::Result<()> {
        let mut last = Bucket::default();
        for b in &self.minute {
            last.add(b);
        }
        let s = format!(
            "{{\"now\":{now:.1},\"total\":{},\"last1min\":{}}}",
            self.section(&self.total, self.start, now),
            self.section(&last, now - self.minute.len() as f64, now)
        );
        let tmp = dir.join("stats.json.tmp");
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(s.as_bytes())?;
        std::fs::rename(tmp, dir.join("stats.json"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn minute_window_and_totals() {
        let mut s = Stats::new(1000.0, Some(49.6));
        for t in 0..70 {
            s.samples(2_400_000);
            s.frame(10.0, 0);
            s.frame(10.0, 1);
            s.noise(1.7);
            s.tick(1000.0 + t as f64, None);
        }
        assert_eq!(s.total.clean, 70);
        assert_eq!(s.minute.len(), 60);
        let mut last = Bucket::default();
        for b in &s.minute {
            last.add(b);
        }
        assert_eq!(last.clean, 60);
        assert_eq!(last.samples, 60 * 2_400_000);
        let sec = s.section(&last, 0.0, 60.0);
        assert!(sec.contains("\"accepted\":[60,60]"), "{sec}");
        assert!(sec.contains("\"gain_db\":49.6"), "{sec}");
    }

    #[test]
    fn dbfs_scale() {
        assert!((dbfs(181.0)).abs() < 1e-6);
        assert!((dbfs(18.1) + 20.0).abs() < 1e-6);
        assert_eq!(dbfs(0.0), -50.0);
    }
}
