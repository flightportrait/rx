#![allow(unknown_lints)]
#![allow(clippy::chunks_exact_to_as_chunks)] // chunks_exact(2) reads as "one sample"; as_chunks does not
//! Unsigned 8-bit interleaved I/Q, the rtl_sdr output format (readsb's
//! `--iformat UC8`). One sample is two bytes: I then Q, each centered on
//! 127.4 (the RTL2832U's measured DC level, as readsb uses).

use std::fs::File;
use std::io::{BufReader, Read};

pub const SAMPLE_RATE: f64 = 2_400_000.0;

/// Magnitudes are 16-bit integers scaled by `MAG_SCALE` (max about 2900),
/// so the per-sample path is integer arithmetic on a 128 KB table.
pub const MAG_SCALE: f32 = 16.0;

/// Magnitude for every (I, Q) byte pair, indexed by `i << 8 | q`.
pub struct MagLut(Box<[u16; 65536]>);

impl MagLut {
    pub fn new() -> Self {
        let mut t = vec![0u16; 65536].into_boxed_slice();
        for i in 0..256usize {
            for q in 0..256usize {
                let fi = i as f32 - 127.4;
                let fq = q as f32 - 127.4;
                t[i << 8 | q] = ((fi * fi + fq * fq).sqrt() * MAG_SCALE).round() as u16;
            }
        }
        MagLut(t.try_into().expect("65536 entries"))
    }

    /// Append the magnitudes of `bytes` (an even count) to `out`.
    pub fn magnitudes(&self, bytes: &[u8], out: &mut Vec<u16>) {
        out.reserve(bytes.len() / 2);
        let t = &self.0;
        out.extend(
            bytes
                .chunks_exact(2)
                .map(|p| t[(p[0] as usize) << 8 | p[1] as usize]),
        );
    }
}

/// Magnitude without a table: 15*max(|I|,|Q|) + 6*min(|I|,|Q|), within
/// about 4 % of 16*sqrt(I^2+Q^2) and on the same scale. Pure integer,
/// vectorizes, and never misses the cache; the table does both on a
/// small core.
pub fn magnitudes_approx(bytes: &[u8], out: &mut Vec<u16>) {
    out.reserve(bytes.len() / 2);
    out.extend(bytes.chunks_exact(2).map(|p| {
        let i = (p[0] as i16 - 127).unsigned_abs();
        let q = (p[1] as i16 - 127).unsigned_abs();
        let (hi, lo) = if i > q { (i, q) } else { (q, i) };
        15 * hi + 6 * lo
    }));
}

/// Two-segment magnitude approximation: max(64*hi, 57*hi + 31*lo) / 4 on
/// the 16x scale, within about 1.5 % of the true magnitude everywhere.
/// Pure integer, vectorizes, no table.
pub fn magnitudes_approx2(bytes: &[u8], out: &mut Vec<u16>) {
    out.reserve(bytes.len() / 2);
    out.extend(bytes.chunks_exact(2).map(|p| {
        let i = (p[0] as i16 - 127).unsigned_abs() as u32;
        let q = (p[1] as i16 - 127).unsigned_abs() as u32;
        let (hi, lo) = if i > q { (i, q) } else { (q, i) };
        ((64 * hi).max(57 * hi + 31 * lo) >> 2) as u16
    }));
}

/// Exact magnitude in floating point, no table: the compiler vectorizes
/// the square root on both x86 and ARM.
pub fn magnitudes_f32(bytes: &[u8], out: &mut Vec<u16>) {
    out.reserve(bytes.len() / 2);
    out.extend(bytes.chunks_exact(2).map(|p| {
        let i = p[0] as f32 - 127.4;
        let q = p[1] as f32 - 127.4;
        ((i * i + q * q).sqrt() * MAG_SCALE + 0.5) as u16
    }));
}

/// The two-segment approximation on values carried at four times the
/// resolution, so magnitudes of one or two units keep their fractions.
pub fn magnitudes_approx3(bytes: &[u8], out: &mut Vec<u16>) {
    out.reserve(bytes.len() / 2);
    out.extend(bytes.chunks_exact(2).map(|p| {
        let i = (4 * p[0] as i32 - 510).unsigned_abs();
        let q = (4 * p[1] as i32 - 510).unsigned_abs();
        let (hi, lo) = if i > q { (i, q) } else { (q, i) };
        ((64 * hi).max(57 * hi + 31 * lo) >> 4) as u16
    }));
}

/// A 16 KB table over the folded pair (hi, lo) = (max|I|, min|I|,|Q|),
/// hi in 0..=128, indexed hi * 129 + lo with lo <= hi. Exact for an
/// integer DC center of 127; small enough to stay in a 32 KB L1 cache.
pub struct MagLutSmall(Box<[u16]>);

impl MagLutSmall {
    pub fn new() -> Self {
        let mut t = vec![0u16; 129 * 129];
        for hi in 0..=128usize {
            for lo in 0..=hi {
                let (h, l) = (hi as f32, lo as f32);
                t[hi * 129 + lo] = ((h * h + l * l).sqrt() * MAG_SCALE).round() as u16;
            }
        }
        MagLutSmall(t.into_boxed_slice())
    }

    pub fn magnitudes(&self, bytes: &[u8], out: &mut Vec<u16>) {
        out.reserve(bytes.len() / 2);
        let t = &self.0;
        out.extend(bytes.chunks_exact(2).map(|p| {
            let i = (p[0] as i16 - 127).unsigned_abs() as usize;
            let q = (p[1] as i16 - 127).unsigned_abs() as usize;
            let (hi, lo) = if i > q { (i, q) } else { (q, i) };
            t[hi * 129 + lo]
        }));
    }
}

impl Default for MagLutSmall {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for MagLut {
    fn default() -> Self {
        Self::new()
    }
}

/// Sequential reader of a UC8 file in sample blocks.
pub struct Uc8Reader {
    inner: BufReader<File>,
}

impl Uc8Reader {
    pub fn open(path: &str) -> anyhow::Result<Self> {
        Ok(Uc8Reader {
            inner: BufReader::with_capacity(1 << 20, File::open(path)?),
        })
    }

    /// Read up to `samples` samples. Returns the raw bytes (an even count);
    /// empty at end of file.
    pub fn read_samples(&mut self, samples: usize) -> anyhow::Result<Vec<u8>> {
        let mut buf = vec![0u8; samples * 2];
        let mut got = 0;
        while got < buf.len() {
            let n = self.inner.read(&mut buf[got..])?;
            if n == 0 {
                break;
            }
            got += n;
        }
        buf.truncate(got & !1);
        Ok(buf)
    }
}
