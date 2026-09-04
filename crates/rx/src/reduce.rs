//! The reduced stream the aggregators ingest (`beast_reduce_out`): per
//! aircraft, each class of information is forwarded at most once per
//! interval, so a busy station sends kilobytes rather than megabytes a
//! second. Position messages are always forwarded, since MLAT servers need
//! every one. The rules follow readsb's track.c: positions at 3/8 of the
//! interval, altitude, heading and speed at 7/8, squawk and emergency at
//! 4x, all-call replies (DF11, DF0) at 4x.

use std::collections::HashMap;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Class {
    Position,
    Often,
    Rare,
    AllCall,
    Other,
}

fn classify(bytes: &[u8]) -> (Option<u32>, Class, bool) {
    let df = bytes[0] >> 3;
    let addr = |b: &[u8]| (b[1] as u32) << 16 | (b[2] as u32) << 8 | b[3] as u32;
    match df {
        17 | 18 if bytes.len() == 14 => {
            let tc = bytes[4] >> 3;
            let class = match tc {
                5..=8 | 9..=18 | 20..=22 => Class::Position,
                19 => Class::Often,
                1..=4 => Class::Often,
                28 => Class::Rare, // emergency / status
                _ => Class::Other,
            };
            (Some(addr(bytes)), class, class == Class::Position)
        }
        11 => (Some(addr(bytes)), Class::AllCall, false),
        0 => (None, Class::AllCall, false),
        4 | 20 => (None, Class::Often, false), // altitude reply; address only via AP
        5 | 21 => (None, Class::Rare, false),  // identity reply
        _ => (None, Class::Other, false),
    }
}

pub struct Reducer {
    interval_ms: u64,
    /// (aircraft, class) -> next time a message of that class is due, ms.
    due: HashMap<(u32, Class), u64>,
    /// Address-parity frames have no reliable address here; rate them as
    /// one pool.
    due_anon: HashMap<Class, u64>,
    last_prune: u64,
}

impl Reducer {
    pub fn new(interval_ms: u64) -> Self {
        Reducer {
            interval_ms,
            due: HashMap::new(),
            due_anon: HashMap::new(),
            last_prune: 0,
        }
    }

    /// Whether to forward this frame at wall time `now_ms`.
    pub fn forward(&mut self, bytes: &[u8], now_ms: u64) -> bool {
        let (addr, class, always) = classify(bytes);
        if always {
            return true;
        }
        let iv = self.interval_ms;
        let step = match class {
            Class::Position => iv * 3 / 8,
            Class::Often => iv * 7 / 8,
            Class::Rare => iv * 4,
            Class::AllCall => iv * 4,
            Class::Other => iv,
        };
        let slot = match addr {
            Some(a) => self.due.entry((a, class)).or_insert(0),
            None => self.due_anon.entry(class).or_insert(0),
        };
        if now_ms >= *slot {
            *slot = now_ms + step;
            if now_ms > self.last_prune + 60_000 {
                self.last_prune = now_ms;
                self.due.retain(|_, t| *t + 120_000 > now_ms);
            }
            true
        } else {
            false
        }
    }
}
