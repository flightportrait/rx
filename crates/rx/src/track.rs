//! The aircraft table: what the radio knows about each address it hears,
//! and the aircraft.json the station reads (readsb's field names for the
//! fields it has). Positions come from CPR even/odd pairs within 10 s.
//! The table is also the consistency check for repaired position
//! messages: a repaired frame whose decoded position is impossibly far
//! from the aircraft's last one is rejected before it is published.

use std::collections::HashMap;
use std::io::Write;

pub struct Aircraft {
    pub hex: u32,
    pub last_seen: f64,
    pub messages: u64,
    /// Last signal level (raw magnitude units).
    pub signal: f32,
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    pub pos_time: f64,
    /// The current position came from MLAT results, not from the aircraft.
    pub pos_mlat: bool,
    pub alt_ft: Option<i32>,
    even: Option<(f64, u32, u32)>,
    odd: Option<(f64, u32, u32)>,
    /// CPR pair store for MLAT result frames, kept apart from ADS-B.
    mlat_even: Option<(f64, u32, u32)>,
    mlat_odd: Option<(f64, u32, u32)>,
}

impl Aircraft {
    fn new(hex: u32, now: f64, signal: f32) -> Self {
        Aircraft {
            hex,
            last_seen: now,
            messages: 0,
            signal,
            lat: None,
            lon: None,
            pos_time: 0.0,
            pos_mlat: false,
            alt_ft: None,
            even: None,
            odd: None,
            mlat_even: None,
            mlat_odd: None,
        }
    }
}

/// An airborne position message of DF17 or DF18 (TC 9..18), CRC checked.
struct Airborne {
    odd: bool,
    cpr_lat: u32,
    cpr_lon: u32,
    alt_ft: Option<i32>,
}

fn parse_airborne(f: &[u8]) -> Option<Airborne> {
    if f.len() != 14 || !matches!(f[0] >> 3, 17 | 18) || mb_modes::crc24(f) != 0 {
        return None;
    }
    let me = u64::from_be_bytes([0, f[4], f[5], f[6], f[7], f[8], f[9], f[10]]);
    let tc = ((me >> 51) & 0x1F) as u8;
    if !(9..=18).contains(&tc) {
        return None;
    }
    Some(Airborne {
        odd: (me >> 34) & 1 == 1,
        cpr_lat: ((me >> 17) & 0x1FFFF) as u32,
        cpr_lon: (me & 0x1FFFF) as u32,
        alt_ft: mb_modes::alt_ac12_decode(((me >> 36) & 0xFFF) as u16),
    })
}

pub struct Tracker {
    pub aircraft: HashMap<u32, Aircraft>,
    pub messages: u64,
    /// Repaired position messages rejected by the consistency check.
    pub rejected: u64,
    /// Receiver position for range in the status output.
    lat: f64,
    lon: f64,
}

/// Outcome of offering a frame to the table.
pub enum Verdict {
    Accept,
    /// A repaired position message contradicting the aircraft's track.
    Reject,
}

impl Tracker {
    pub fn new(lat: f64, lon: f64) -> Self {
        Tracker {
            aircraft: HashMap::new(),
            messages: 0,
            rejected: 0,
            lat,
            lon,
        }
    }

    /// Offer a decoded frame at wall time `now` (seconds). `fixed` is the
    /// repaired-bit count.
    pub fn offer(&mut self, bytes: &[u8], signal: f32, fixed: u8, now: f64) -> Verdict {
        let df = bytes[0] >> 3;
        let hex = match df {
            11 | 17 | 18 => (bytes[1] as u32) << 16 | (bytes[2] as u32) << 8 | bytes[3] as u32,
            _ => return Verdict::Accept, // address-parity frames: admitted upstream
        };
        // Position messages: decode, and check repaired ones against the track.
        let mut new_pos: Option<(f64, f64, Option<i32>)> = None;
        if df == 17 || df == 18 {
            if let Some(p) = mb_modes::decode::parse_df17_airborne(bytes) {
                let entry = self.aircraft.get(&hex);
                let (even, odd) = match entry {
                    Some(a) => (a.even, a.odd),
                    None => (None, None),
                };
                let mine = (now, p.cpr_lat, p.cpr_lon);
                let partner = if p.odd { even } else { odd };
                if let Some((t, la, lo)) = partner {
                    if now - t < 10.0 {
                        let (e, o) = if p.odd { ((la, lo), (p.cpr_lat, p.cpr_lon)) } else { ((p.cpr_lat, p.cpr_lon), (la, lo)) };
                        if let Some((lat, lon)) = mb_modes::cpr::global_decode_airborne(e, o, p.odd) {
                            new_pos = Some((lat, lon, p.alt_ft));
                        }
                    }
                }
                if let Some((lat, lon, _)) = new_pos {
                    // Consistency: against the last position, allow 400 m/s of
                    // travel plus 2 km. Applied to repaired frames only; a
                    // clean CRC is its own proof.
                    if fixed > 0 {
                        if let Some(a) = entry {
                            if let (Some(plat), Some(plon)) = (a.lat, a.lon) {
                                let dt = (now - a.pos_time).max(0.0);
                                let d = haversine_m(plat, plon, lat, lon);
                                if d > 400.0 * dt + 2000.0 {
                                    self.rejected += 1;
                                    return Verdict::Reject;
                                }
                            }
                        }
                    }
                }
                let a = self.aircraft.entry(hex).or_insert_with(|| Aircraft::new(hex, now, signal));
                if p.odd {
                    a.odd = Some(mine);
                } else {
                    a.even = Some(mine);
                }
                if let Some((lat, lon, alt)) = new_pos {
                    a.lat = Some(lat);
                    a.lon = Some(lon);
                    a.pos_time = now;
                    a.pos_mlat = false;
                    if alt.is_some() {
                        a.alt_ft = alt;
                    }
                }
            }
        }
        let a = self.aircraft.entry(hex).or_insert_with(|| Aircraft::new(hex, now, signal));
        a.last_seen = now;
        a.messages += 1;
        a.signal = signal;
        self.messages += 1;
        Verdict::Accept
    }

    /// An MLAT result frame from Beast input (DF18 pair with the magic
    /// timestamp). Decoded through its own CPR pair store; the position is
    /// taken only when the aircraft has no ADS-B position younger than
    /// 30 s. Never counted as a message the aircraft sent.
    pub fn offer_mlat(&mut self, bytes: &[u8], now: f64) {
        let Some(p) = parse_airborne(bytes) else { return };
        let hex = (bytes[1] as u32) << 16 | (bytes[2] as u32) << 8 | bytes[3] as u32;
        let a = self.aircraft.entry(hex).or_insert_with(|| Aircraft::new(hex, now, 0.0));
        let mine = (now, p.cpr_lat, p.cpr_lon);
        let partner = if p.odd { a.mlat_even } else { a.mlat_odd };
        if p.odd {
            a.mlat_odd = Some(mine);
        } else {
            a.mlat_even = Some(mine);
        }
        let Some((t, la, lo)) = partner else { return };
        if now - t >= 10.0 {
            return;
        }
        let (e, o) = if p.odd { ((la, lo), (p.cpr_lat, p.cpr_lon)) } else { ((p.cpr_lat, p.cpr_lon), (la, lo)) };
        let Some((lat, lon)) = mb_modes::cpr::global_decode_airborne(e, o, p.odd) else { return };
        let adsb_fresh = !a.pos_mlat && a.lat.is_some() && now - a.pos_time < 30.0;
        if adsb_fresh {
            return;
        }
        a.lat = Some(lat);
        a.lon = Some(lon);
        a.pos_time = now;
        a.pos_mlat = true;
        if a.alt_ft.is_none() {
            a.alt_ft = p.alt_ft;
        }
        a.last_seen = a.last_seen.max(now);
    }

    /// Drop aircraft silent for more than `ttl` seconds.
    pub fn expire(&mut self, now: f64, ttl: f64) {
        self.aircraft.retain(|_, a| now - a.last_seen <= ttl);
    }

    /// Write aircraft.json in readsb's shape for the fields we have.
    pub fn write_json(&self, dir: &std::path::Path, now: f64) -> std::io::Result<()> {
        let mut s = String::new();
        s.push_str(&format!("{{\"now\":{now:.1},\"messages\":{},\"aircraft\":[", self.messages));
        let mut first = true;
        let mut list: Vec<&Aircraft> = self.aircraft.values().collect();
        list.sort_by_key(|a| a.hex);
        for a in list {
            if !first {
                s.push(',');
            }
            first = false;
            s.push_str(&format!(
                "{{\"hex\":\"{:06x}\",\"messages\":{},\"seen\":{:.1},\"rssi\":{:.1}",
                a.hex,
                a.messages,
                now - a.last_seen,
                rssi_dbfs(a.signal)
            ));
            if let (Some(lat), Some(lon)) = (a.lat, a.lon) {
                s.push_str(&format!(",\"lat\":{lat:.6},\"lon\":{lon:.6},\"seen_pos\":{:.1},\"r_dst\":{:.1}", now - a.pos_time, haversine_m(self.lat, self.lon, lat, lon) / 1852.0));
                if a.pos_mlat {
                    s.push_str(",\"mlat\":[\"lat\",\"lon\"]");
                }
            }
            if let Some(alt) = a.alt_ft {
                s.push_str(&format!(",\"alt_baro\":{alt}"));
            }
            s.push('}');
        }
        s.push_str("]}");
        let tmp = dir.join("aircraft.json.tmp");
        let mut f = std::fs::File::create(&tmp)?;
        f.write_all(s.as_bytes())?;
        std::fs::rename(tmp, dir.join("aircraft.json"))
    }
}

/// readsb reports RSSI in dBFS; a full-scale magnitude is about 181.
fn rssi_dbfs(level: f32) -> f32 {
    if level <= 0.0 {
        -50.0
    } else {
        (20.0 * (level / 181.0).log10()).max(-50.0)
    }
}

fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (a1, o1, a2, o2) = (lat1.to_radians(), lon1.to_radians(), lat2.to_radians(), lon2.to_radians());
    let h = ((a2 - a1) / 2.0).sin().powi(2) + a1.cos() * a2.cos() * ((o2 - o1) / 2.0).sin().powi(2);
    2.0 * 6_371_000.0 * h.sqrt().asin()
}
