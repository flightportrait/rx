//! Mode S demodulation at 2.4 MS/s (2.4 samples per microsecond).
//!
//! Preamble: pulses at 0, 1.0, 3.5 and 4.5 us, each 0.5 us wide; data
//! from 8 us, one bit per microsecond, pulse-position modulated (energy
//! in the first half means 1). With fewer than three samples per bit the
//! demodulator works on an interpolated magnitude at fractional sample
//! positions: candidate message starts are tried at five phases per
//! sample, and each bit is decided by comparing the integrated magnitude
//! of its two halves.
//!
//! Error correction is CRC-syndrome based and soft: a corrupted frame is
//! repaired by flipping one or two of the least confident bits, and only
//! when the syndrome says those exact bits explain the residual.

pub const SPB: f64 = 2.4; // samples per bit (microsecond)

/// A demodulated frame.
#[derive(Debug, Clone)]
pub struct Frame {
    /// Preamble start, as a fractional sample index into the stream.
    pub start: f64,
    pub bytes: Vec<u8>,
    /// Mean magnitude of the four preamble pulses.
    pub signal: f32,
    /// Mean magnitude in the preamble gaps: energy there that is not
    /// noise is another signal.
    pub gap: f32,
    /// Energy in the empty half of each bit over the energy in the full
    /// half: near the noise-to-signal ratio for a lone frame, higher when
    /// another frame is underneath.
    pub off_ratio: f32,
    /// Mean magnitude in the empty half-bits, in sample units: the noise
    /// floor for a lone frame, more when another frame is underneath.
    pub off_level: f32,
    /// Bits flipped by error correction (0 = received clean). Emitted with
    /// every frame so a consumer can weigh repaired messages separately;
    /// readsb's output cannot tell them apart.
    pub fixed: u8,
}

#[derive(Debug, Clone)]
pub struct Params {
    /// Preamble pulse mean must exceed this multiple of the gap mean.
    pub preamble_ratio: f32,
    /// Integer pre-check: the mean of the pulses' strongest samples must
    /// exceed this multiple of the gap mean. Cheap; sets the cost.
    pub precheck_ratio: f32,
    /// Integer pre-check: at most this many gap samples may exceed the
    /// weakest pulse.
    pub gap_spikes: usize,
    /// Maximum bits flipped by error correction: 0 to 5.
    pub max_fix: u8,
    /// Single-bit fixes consider only the N least confident bits.
    pub soft1: usize,
    /// Two-bit fixes consider pairs among the N least confident bits.
    pub soft2: usize,
    /// Three-bit fixes consider triples among the N least confident bits.
    pub soft3: usize,
    /// Four- and five-bit fixes search among the N least confident bits.
    pub soft4: usize,
    pub soft5: usize,
    /// Address-parity frames: one-bit repair toward a confirmed address,
    /// among the N least confident bits (0 = off).
    pub ap_fix: usize,
    /// When a CRC-checked frame fails or needs repair at the preamble's
    /// timing, re-slice at these offsets (samples) and keep the best.
    pub retry_offsets: Vec<f64>,
    /// Fine preamble test: pulse mean must exceed the gap mean by this
    /// many gap standard deviations (0 = off).
    pub noise_sigmas: f32,
    /// Retries and multi-bit repair run only when the sliced address is
    /// within this many bit flips of a confirmed aircraft.
    pub near_bits: u32,
    /// Fractional phases tested around the best integer start: 1 tests
    /// +-0.2 sample, 2 tests +-0.2 and +-0.4.
    pub phase_span: usize,
    /// Fine test: the weakest pulse must be at least this fraction of the
    /// strongest (0 = off). Noise hits tend to be one spike.
    pub pulse_uniformity: f32,
    /// Collision recovery: after an accepted frame, reconstruct it from its
    /// bits and the measured pulse shape, fit carrier and amplitude on the
    /// I/Q, subtract, and rescan its span. Needs I/Q in `run_iq`.
    pub cancel: bool,
    /// Cancel only frames whose preamble gaps exceed this multiple of the
    /// sub-block's mean magnitude (the noise floor), or that needed repair.
    pub cancel_gap_ratio: f32,
    /// Cancel also frames whose empty half-bits carry at least this
    /// fraction of the energy of their full half-bits, provided that
    /// energy is also above `cancel_gap_ratio` times the noise floor.
    pub cancel_off_ratio: f32,
    /// Bit decision: `Box` compares half-bit energies; `Matched` correlates
    /// each half with the measured pulse shape.
    pub slicer: Slicer,
    /// Matched slicer: time of the first template tap relative to the
    /// nominal pulse start, in samples (experiment knob).
    pub tap0: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slicer {
    Box,
    Matched,
}

/// Measured pulse shape (2026-09-04 capture, RTL-SDR v3, 2.4 MS/s):
/// the mean magnitude around a preamble pulse, baseline removed, at 0.2
/// sample steps from -0.6 to +1.6 samples relative to the pulse start.
const PULSE_TAPS: [f32; 12] = [
    0.01, 0.16, 0.27, 0.50, 0.67, 0.75, 0.78, 0.73, 0.54, 0.34, 0.21, 0.03,
];
const PULSE_TAP0: f64 = -0.6;

/// Correlation of the stream with the pulse shape placed at `t0`.
#[inline]
fn pulse_corr(m: &[u16], t0: f64, tap0: f64) -> f32 {
    let mut acc = 0f32;
    for (k, &h) in PULSE_TAPS.iter().enumerate() {
        let t = t0 + tap0 + 0.2 * k as f64;
        if t >= 0.0 {
            acc += h * m[t as usize] as f32;
        }
    }
    acc
}

impl Default for Params {
    fn default() -> Self {
        Params {
            preamble_ratio: 1.6,
            precheck_ratio: 2.0,
            gap_spikes: 2,
            max_fix: 3,
            soft1: 24,
            soft2: 12,
            soft3: 16,
            soft4: 18,
            soft5: 20,
            ap_fix: 16,
            retry_offsets: vec![-0.2, 0.2, -0.4, 0.4, -0.6, 0.6],
            noise_sigmas: 0.0,
            near_bits: 5,
            phase_span: 2,
            pulse_uniformity: 0.0,
            cancel: true,
            // Both 0: every accepted frame is cancelled and its span rescanned.
            // On a real sky that is a few dozen fits a second, a fraction of a
            // percent of a Pi core; the triggers only ever paid on synthetic
            // sets with a thousand collisions a second.
            cancel_gap_ratio: 0.0,
            cancel_off_ratio: 0.0,
            slicer: Slicer::Box,
            tap0: PULSE_TAP0,
        }
    }
}

const PHASES: [f64; 5] = [0.0, 0.2, 0.4, 0.6, 0.8];
// Preamble pulses (start, end) in microseconds, and the gaps between them.
const PULSES: [(f64, f64); 4] = [(0.0, 0.5), (1.0, 1.5), (3.5, 4.0), (4.5, 5.0)];
const GAPS: [(f64, f64); 3] = [(0.5, 1.0), (1.5, 3.5), (5.0, 8.0)];
const GAP_LEN: f64 = 5.5; // microseconds of gap in GAPS
const DATA_START: f64 = 8.0 * SPB;
const LONG_SAMPLES: usize = (8.0 * SPB + 112.0 * SPB) as usize + 3;

/// Energy of the stream over the fractional interval [a, b), treating each
/// sample as the mean level over its own interval [i, i + 1).
#[inline]
fn energy(m: &[u16], a: f64, b: f64) -> f32 {
    let mut e = 0f32;
    let mut i = a.floor() as usize;
    while (i as f64) < b {
        let lo = (i as f64).max(a);
        let hi = ((i + 1) as f64).min(b);
        e += m[i] as f32 * (hi - lo) as f32;
        i += 1;
    }
    e
}

/// Energy over the half-bit interval starting at `fa` fifths of a sample,
/// 6 fifths long, in fifths: three samples with integer weights set by the
/// phase. Same integral as `energy`, no loop, no float.
/// Weights (w0, w1) of the two samples under a half bit (6 fifths) starting
/// at phase k fifths into sample i: [5-k, 1+k]. A third sample is never
/// reached: k + 6 <= 10.
const HALF_BIT_W: [[i32; 2]; 5] = [[5, 1], [4, 2], [3, 3], [2, 4], [1, 5]];
/// Advancing a half bit (6 fifths) from phase k: (samples to add, new phase).
const HALF_STEP: [(usize, usize); 5] = [(1, 1), (1, 2), (1, 3), (1, 4), (2, 0)];

/// Callers keep `i + 2 < m.len()`: `run` only starts frames before
/// `m.len() - LONG_SAMPLES`, which leaves three samples past the longest
/// frame at the largest retry offset.
#[inline(always)]
fn half_bit_at(m: &[u16], i: usize, k: usize) -> i32 {
    debug_assert!(i + 1 < m.len() && k < 5);
    // SAFETY: see above; the probe and template paths call with the same bound.
    unsafe {
        let w = HALF_BIT_W.get_unchecked(k);
        w[0] * *m.get_unchecked(i) as i32 + w[1] * *m.get_unchecked(i + 1) as i32
    }
}

/// The fine preamble test at an integer start, as fixed weights in
/// fifths of a sample (0.2 -> 1, 1.0 -> 5). Same integrals as
/// `Demodulator::preamble`, no loops, integer arithmetic. Returns
/// (pulse mean, gap mean, pulse min) in fifths.
#[inline(always)]
fn preamble_int(w: &[u16]) -> (u32, u32, u32, u32) {
    let m = |k: usize| w[k] as u32;
    let p0 = 5 * m(0) + m(1);
    let p1 = 3 * m(2) + 3 * m(3);
    let p2 = 3 * m(8) + 3 * m(9);
    let p3 = m(10) + 5 * m(11);
    // gaps: [1.2,2.4) [3.6,8.4) [12,19.2): total length 5.5 us = 13.2 samples = 66 fifths
    let g = 4 * m(1) + 2 * m(2) + 2 * m(3) + 5 * (m(4) + m(5) + m(6) + m(7)) + 2 * m(8)
        + 5 * (m(12) + m(13) + m(14) + m(15) + m(16) + m(17) + m(18)) + m(19);
    // pulse level per half-us = pulse/6 fifths; gap level per fifth-sample = g/66.
    // Return everything scaled to "per fifth of a sample" * 6 * 66 to stay integer:
    let pm = (p0 + p1 + p2 + p3) * 66 / 4; // sum/4 pulses, each over 6 fifths -> *66/6... see below
    let _ = pm;
    // level_pulse_k = p_k / 6 ; level_gap = g / 66. Compare in units of 1/(6*66):
    let lp = [p0 * 66, p1 * 66, p2 * 66, p3 * 66]; // p_k/6 * 396
    let lg = g * 6; // g/66 * 396
    let pmean = (lp[0] + lp[1] + lp[2] + lp[3]) / 4;
    let pmin = lp[0].min(lp[1]).min(lp[2]).min(lp[3]);
    let pmax = lp[0].max(lp[1]).max(lp[2]).max(lp[3]);
    (pmean, lg, pmin, pmax)
}

/// The three gate passes over one sub-block: pairwise maxima, prefix
/// sums, and the hit mask `psum * 11 * 256 > ratio4 * gsum`.
#[inline(never)]
fn gate_passes(span: &[u16], len: usize, ratio4: u32, pmax: &mut Vec<u16>, cum: &mut Vec<u32>, hits: &mut Vec<u8>) {
    pmax.clear();
    pmax.resize(span.len() - 1, 0);
    cum.clear();
    cum.resize(span.len() + 1, 0);
    hits.clear();
    hits.resize(len, 0);
    let mut acc = 0u32;
    for (o, &x) in cum[1..].iter_mut().zip(span) {
        acc += x as u32;
        *o = acc;
    }
    {
        let n = span.len() - 1;
        let (x, y, o) = (&span[..n], &span[1..n + 1], &mut pmax[..n]);
        for j in 0..n {
            o[j] = x[j].max(y[j]);
        }
        let (a, b, c8, d) = (&pmax[..len], &pmax[2..2 + len], &pmax[8..8 + len], &pmax[10..10 + len]);
        let (g8, g4, g19, g12) = (&cum[8..8 + len], &cum[4..4 + len], &cum[19..19 + len], &cum[12..12 + len]);
        let hits = &mut hits[..len];
        for j in 0..len {
            let psum = a[j] as u32 + b[j] as u32 + c8[j] as u32 + d[j] as u32;
            let gsum = (g8[j] - g4[j]) + (g19[j] - g12[j]);
            hits[j] = (psum * 2816 > ratio4 * gsum) as u8;
        }
    }
}

/// CRC-24 (poly 0xFFF409) with a byte table; identical to mb_modes::crc24.
struct Crc24 {
    table: [u32; 256],
}

impl Crc24 {
    fn new() -> Self {
        let mut table = [0u32; 256];
        for (b, e) in table.iter_mut().enumerate() {
            let mut crc = (b as u32) << 16;
            for _ in 0..8 {
                crc <<= 1;
                if crc & 0x1_000000 != 0 {
                    crc ^= mb_modes::CRC24_POLY;
                }
            }
            *e = crc & 0xFF_FFFF;
        }
        Crc24 { table }
    }

    #[inline]
    fn of(&self, data: &[u8]) -> u32 {
        let mut crc = 0u32;
        for &b in data {
            crc = (self.table[(((crc >> 16) ^ b as u32) & 0xFF) as usize] ^ (crc << 8)) & 0xFF_FFFF;
        }
        crc
    }
}

/// Index of the first nonzero byte of `hits` at or after `from`, scanning
/// whole words where possible.
#[inline]
fn next_hit(hits: &[u8], from: usize) -> Option<usize> {
    let mut j = from;
    let n = hits.len();
    while j < n && j % 8 != 0 {
        if hits[j] != 0 {
            return Some(j);
        }
        j += 1;
    }
    while j + 8 <= n {
        let w = u64::from_ne_bytes(hits[j..j + 8].try_into().unwrap());
        if w != 0 {
            return Some(j + (w.to_le().trailing_zeros() / 8) as usize);
        }
        j += 8;
    }
    while j < n {
        if hits[j] != 0 {
            return Some(j);
        }
        j += 1;
    }
    None
}

/// Pulse envelope at 0.2-sample steps from -0.6 to +1.6 samples relative
/// to the pulse start, measured on the 2026-09-04 capture (`bench
/// template`), peak 1.
pub const ENVELOPE: [f32; 12] = [
    0.013, 0.205, 0.346, 0.641, 0.859, 0.962, 1.0, 0.936, 0.692, 0.436, 0.269, 0.038,
];

/// The pulse envelope at `rel` samples from the pulse start, interpolated
/// from `ENVELOPE`; zero outside -0.6..1.6.
#[inline]
pub fn envelope_at(rel: f64) -> f32 {
    let x = (rel + 0.6) / 0.2;
    if x <= 0.0 || x >= 11.0 {
        return 0.0;
    }
    let k = x.floor() as usize;
    let f = (x - k as f64) as f32;
    ENVELOPE[k] * (1.0 - f) + ENVELOPE[k + 1] * f
}

/// Envelope of a whole frame (preamble and data) with its preamble at
/// fractional sample `start`, over `n` samples: each sample gets the
/// interpolated envelope of every pulse that reaches it.
pub fn frame_envelope(bytes: &[u8], start: f64, n: usize) -> Vec<f32> {
    let mut env = vec![0f32; n];
    let mut pulse = |t_us: f64| {
        let p0 = start + t_us * SPB;
        let lo = (p0 - 0.6).floor().max(0.0) as usize;
        let hi = ((p0 + 1.6).ceil() as usize).min(n.saturating_sub(1));
        for i in lo..=hi {
            env[i] += envelope_at(i as f64 - p0);
        }
    };
    for t in [0.0, 1.0, 3.5, 4.5] {
        pulse(t);
    }
    for (bi, byte) in bytes.iter().enumerate() {
        for k in 0..8 {
            let bit = (byte >> (7 - k)) & 1;
            let t = 8.0 + (bi * 8 + k) as f64;
            pulse(if bit == 1 { t } else { t + 0.5 });
        }
    }
    for e in env.iter_mut() {
        *e = e.min(1.2);
    }
    env
}

/// Fit of one frame to the I/Q: envelope from the bits at a trial start,
/// carrier from the phase advance between consecutive pulse samples, then
/// refined by a small grid, complex amplitude by least squares. Returns
/// (start, w, cr, ci, residual energy / energy before) for the best trial.
fn fit_frame(iq: &[u8], s0: usize, n: usize, start_rel: f64, bytes: &[u8]) -> (f64, f64, f32, f32, f32) {
    let z: Vec<(f32, f32)> = (0..n)
        .map(|k| (iq[2 * (s0 + k)] as f32 - 127.4, iq[2 * (s0 + k) + 1] as f32 - 127.4))
        .collect();
    let before: f32 = z.iter().map(|(a, b)| a * a + b * b).sum();
    let mut best = (start_rel, 0.0f64, 0f32, 0f32, f32::MAX);
    if before == 0.0 {
        return best;
    }
    // One trial: envelope at `d`, frequency w; returns (cr, ci, residual).
    let trial = |env: &[f32], w: f64| -> (f32, f32, f32) {
        // rotate incrementally: r_k = e^{-j w k}
        let (cw, sw) = ((-w).cos() as f32, (-w).sin() as f32);
        let (mut rr, mut ri) = (1f32, 0f32);
        let (mut nr, mut ni, mut den) = (0f32, 0f32, 0f32);
        for k in 0..n {
            if env[k] >= 0.3 {
                let (a, b) = z[k];
                // z * conj(e^{jwk}) = z * r_k
                nr += env[k] * (a * rr - b * ri);
                ni += env[k] * (b * rr + a * ri);
                den += env[k] * env[k];
            }
            let t = rr * cw - ri * sw;
            ri = rr * sw + ri * cw;
            rr = t;
        }
        if den == 0.0 {
            return (0.0, 0.0, f32::MAX);
        }
        let (cr, ci) = (nr / den, ni / den);
        let (cw, sw) = (w.cos() as f32, w.sin() as f32);
        let (mut rr, mut ri) = (1f32, 0f32);
        let mut after = 0f32;
        for k in 0..n {
            let (a, b) = z[k];
            if env[k] != 0.0 {
                let mr = env[k] * (cr * rr - ci * ri);
                let mi = env[k] * (cr * ri + ci * rr);
                after += (a - mr) * (a - mr) + (b - mi) * (b - mi);
            } else {
                after += a * a + b * b;
            }
            let t = rr * cw - ri * sw;
            ri = rr * sw + ri * cw;
            rr = t;
        }
        (cr, ci, after)
    };
    let freq0 = |env: &[f32]| -> f64 {
        let (mut re, mut im) = (0f32, 0f32);
        for k in 0..n - 1 {
            if env[k] >= 0.6 && env[k + 1] >= 0.6 {
                let (a, b) = z[k];
                let (c, dd) = z[k + 1];
                re += c * a + dd * b;
                im += dd * a - c * b;
            }
        }
        im.atan2(re) as f64
    };
    // coarse: alignment in 0.2 steps, frequency in 0.06 steps
    let mut best_d = 0.0f64;
    let mut best_w = 0.0f64;
    for di in -2..=2 {
        let d = di as f64 * 0.2;
        let env = frame_envelope(bytes, start_rel + d, n);
        let w0 = freq0(&env);
        for dwi in -3..=3 {
            let dw = dwi as f64 * 0.03;
            let (cr, ci, after) = trial(&env, w0 + dw);
            if after < best.4 {
                best = (start_rel + d, w0 + dw, cr, ci, after);
                best_d = d;
                best_w = w0 + dw;
            }
        }
    }
    // fine: 0.05 steps around the best alignment, 0.02 around the frequency
    for di in -3..=3 {
        let d = best_d + di as f64 * 0.05;
        let env = frame_envelope(bytes, start_rel + d, n);
        for dwi in -2..=2 {
            let w = best_w + dwi as f64 * 0.01;
            let (cr, ci, after) = trial(&env, w);
            if after < best.4 {
                best = (start_rel + d, w, cr, ci, after);
            }
        }
    }
    best.4 /= before;
    best
}

/// Subtract a decoded frame from the I/Q using `fit_frame`. Returns the
/// fraction of the span's energy removed.
fn cancel_frame(iq: &mut [u8], start: f64, bytes: &[u8]) -> f32 {
    let s0 = (start.floor() as usize).saturating_sub(2);
    let n = ((8.0 + bytes.len() as f64 * 8.0) * SPB) as usize + 6;
    let n = n.min(iq.len() / 2 - s0);
    if n < 40 {
        return 0.0;
    }
    let (st, w, cr, ci, resid) = fit_frame(iq, s0, n, start - s0 as f64, bytes);
    if resid == f32::MAX {
        return 0.0;
    }
    let env = frame_envelope(bytes, st, n);
    let (cw, sw) = (w.cos() as f32, w.sin() as f32);
    let (mut rr, mut ri) = (1f32, 0f32);
    for k in 0..n {
        let (c, sn) = (rr, ri);
        let t = rr * cw - ri * sw;
        ri = rr * sw + ri * cw;
        rr = t;
        if env[k] == 0.0 {
            continue;
        }
        let mr = env[k] * (cr * c - ci * sn);
        let mi = env[k] * (cr * sn + ci * c);
        let a = iq[2 * (s0 + k)] as f32 - 127.4 - mr;
        let b = iq[2 * (s0 + k) + 1] as f32 - 127.4 - mi;
        iq[2 * (s0 + k)] = (a + 127.4).round().clamp(0.0, 255.0) as u8;
        iq[2 * (s0 + k) + 1] = (b + 127.4).round().clamp(0.0, 255.0) as u8;
    }
    1.0 - resid
}

fn long_df(df: u8) -> bool {
    matches!(df, 16 | 17 | 18 | 19 | 20 | 21 | 24)
}

/// Syndrome of a single flipped bit at index `i` in an `nbits` frame.
fn syndromes(nbits: usize) -> Vec<u32> {
    (0..nbits)
        .map(|i| {
            let mut f = vec![0u8; nbits / 8];
            f[i / 8] |= 0x80 >> (i % 8);
            mb_modes::crc24(&f)
        })
        .collect()
}

/// Why the demodulator did or did not produce a known frame near a
/// given sample position.
#[derive(Debug, Clone)]
pub struct Probe {
    pub precheck: bool,
    pub preamble: bool,
    pub best_start: f64,
    pub best_score: f32,
    /// Bit errors against the expected bytes at the best start (or at
    /// the given position if no preamble was accepted).
    pub bit_errors: usize,
    /// Rank (0 = least confident) of each wrong bit in the confidence order.
    pub error_ranks: Vec<usize>,
    pub pulse_level: f32,
    pub gap_level: f32,
}

/// Work counters, for cost analysis.
#[derive(Debug, Default, Clone)]
pub struct Stats {
    pub samples: u64,
    pub precheck_hits: u64,
    pub preamble_hits: u64,
    pub decodes: u64,
    pub crc_ok: u64,
    pub frames: u64,
    /// Decodes that failed at the preamble's timing and were retried at
    /// neighbouring offsets; how many of those retries produced a frame.
    pub retries: u64,
    pub retry_wins: u64,
    /// Preamble hits dropped by the format bits at the integer start.
    pub df_rejects: u64,
    /// Frames subtracted from the I/Q, and frames found in a rescan.
    pub cancelled: u64,
    pub rescan_frames: u64,
    /// Energy removed by cancellation: counts in buckets <50 %, 50-80,
    /// 80-90, 90-95, >95 %.
    pub removed_hist: [u64; 5],
}

/// How the recovered address (CRC of the data part XOR the parity field)
/// moves when bit `i` of an `nbits` address-parity frame flips: by the CRC
/// of that bit alone for data bits, by the bit itself inside the parity.
fn address_syndromes(nbits: usize) -> Vec<u32> {
    let data_bits = nbits - 24;
    (0..nbits)
        .map(|i| {
            if i < data_bits {
                let mut f = vec![0u8; data_bits / 8];
                f[i / 8] |= 0x80 >> (i % 8);
                mb_modes::crc24(&f)
            } else {
                1 << (nbits - 1 - i)
            }
        })
        .collect()
}

/// Preamble weights for one start phase (a fifth of a sample), over the
/// 21 samples from the integer start: four pulses and the combined gap,
/// as integers in fifths of a sample. Exact, since every interval
/// boundary is a multiple of 0.2 sample.
struct PhaseWeights {
    pulses: [[i32; 21]; 4],
    gap: [i32; 21],
    /// Each pulse covers at most three consecutive samples: (first index,
    /// three weights).
    pulse_taps: [(usize, [i32; 3]); 4],
}

fn phase_weights(phase: f64) -> PhaseWeights {
    let mut pw = PhaseWeights {
        pulses: [[0; 21]; 4],
        gap: [0; 21],
        pulse_taps: [(0, [0; 3]); 4],
    };
    let fill = |row: &mut [i32; 21], a_us: f64, b_us: f64| {
        let a = phase + a_us * SPB;
        let b = phase + b_us * SPB;
        for (i, w) in row.iter_mut().enumerate() {
            let lo = (i as f64).max(a);
            let hi = ((i + 1) as f64).min(b);
            if hi > lo {
                *w += ((hi - lo) * 5.0).round() as i32;
            }
        }
    };
    for (k, &(a, b)) in PULSES.iter().enumerate() {
        fill(&mut pw.pulses[k], a, b);
    }
    for &(a, b) in &GAPS {
        fill(&mut pw.gap, a, b);
    }
    for k in 0..4 {
        let row = &pw.pulses[k];
        let first = row.iter().position(|&w| w != 0).unwrap();
        let mut taps = [0i32; 3];
        for (t, w) in taps.iter_mut().zip(&row[first..]) {
            *t = *w;
        }
        debug_assert!(row[first + 3..].iter().all(|&w| w == 0));
        pw.pulse_taps[k] = (first, taps);
    }
    pw
}

pub struct Demodulator {
    pub stats: Stats,
    crc: Crc24,
    /// Weights for phases 0.0, 0.2, 0.4, 0.6, 0.8.
    phases: Vec<PhaseWeights>,
    /// Downlink format sliced by the last `decode`, whatever its outcome.
    last_df: u8,
    /// Off-half over on-half energy of the last decode, and the off-half
    /// mean magnitude.
    last_off_ratio: f32,
    last_off_level: f32,
    /// Whether the address sliced by the last `decode` (DF11/17/18) is a
    /// confirmed aircraft; noise almost never is.
    last_known: bool,
    /// Stream position of the frame being decoded, for the known-aircraft
    /// window.
    now: u64,
    /// Mean magnitude of the current sub-block: the noise floor where the
    /// sky is mostly quiet.
    block_mean: f32,
    /// Scratch for the gate passes, kept between blocks.
    pmax: Vec<u16>,
    cum: Vec<u32>,
    hits: Vec<u8>,
    p: Params,
    syn56: Vec<u32>,
    syn112: Vec<u32>,
    /// (syndrome, bit) sorted by syndrome, for single-bit lookup.
    syn56_sorted: Vec<(u32, usize)>,
    syn112_sorted: Vec<(u32, usize)>,
    /// (address syndrome, bit) sorted by syndrome, for the address repair.
    asyn56_sorted: Vec<(u32, usize)>,
    asyn112_sorted: Vec<(u32, usize)>,
    /// Aircraft confirmed by a CRC-checked frame: address, last sample.
    known: std::collections::HashMap<u32, u64>,
}

/// Address-parity frames are accepted only for aircraft confirmed within
/// this many samples (60 s).
const KNOWN_WINDOW: u64 = 60 * 2_400_000;

impl Demodulator {
    pub fn new(p: Params) -> Self {
        Demodulator {
            stats: Stats::default(),
            crc: Crc24::new(),
            phases: (0..5).map(|k| phase_weights(0.2 * k as f64)).collect(),
            last_df: 0,
            last_off_ratio: 0.0,
            last_off_level: 0.0,
            last_known: false,
            now: 0,
            block_mean: 0.0,
            pmax: Vec::new(),
            cum: Vec::new(),
            hits: Vec::new(),
            p,
            syn56: syndromes(56),
            syn112: syndromes(112),
            syn56_sorted: {
                let mut v: Vec<(u32, usize)> = syndromes(56).into_iter().enumerate().map(|(i, s)| (s, i)).collect();
                v.sort_unstable();
                v
            },
            syn112_sorted: {
                let mut v: Vec<(u32, usize)> = syndromes(112).into_iter().enumerate().map(|(i, s)| (s, i)).collect();
                v.sort_unstable();
                v
            },
            asyn56_sorted: {
                let mut v: Vec<(u32, usize)> = address_syndromes(56).into_iter().enumerate().map(|(i, s)| (s, i)).collect();
                v.sort_unstable();
                v
            },
            asyn112_sorted: {
                let mut v: Vec<(u32, usize)> = address_syndromes(112).into_iter().enumerate().map(|(i, s)| (s, i)).collect();
                v.sort_unstable();
                v
            },
            known: std::collections::HashMap::new(),
        }
    }

    /// `run` with collision recovery: `iq` holds the raw bytes aligned with
    /// `m` (two per sample); `remag` recomputes magnitudes for a span.
    /// After each accepted frame the frame is subtracted from the I/Q and
    /// its span rescanned once for a frame that was underneath it.
    pub fn run_iq(&mut self, m: &mut Vec<u16>, iq: &mut [u8], base: u64, remag: &dyn Fn(&[u8], &mut [u16])) -> Vec<Frame> {
        let mut out = self.run(m, base);
        if !self.p.cancel {
            return out;
        }
        let mut k = 0;
        let mut emitted: std::collections::HashSet<(u64, Vec<u8>)> =
            out.iter().map(|f| ((f.start * 5.0).round() as u64, f.bytes.clone())).collect();
        while k < out.len() {
            let f = out[k].clone();
            k += 1;
            let start = f.start - base as f64;
            if start < 0.0 {
                continue;
            }
            // Only frames that look like collisions: energy in the preamble
            // gaps, or bits that needed repair.
            // The preamble gaps sit between pulses and collect their tails,
            // about 0.3 of the pulse level; compare with that, not the floor.
            let expected_gap = self.block_mean + 0.3 * f.signal;
            let gap_hot = self.p.cancel_gap_ratio > 0.0 && f.gap >= self.p.cancel_gap_ratio * expected_gap;
            // A pulse leaks about a third of its level into the neighbouring
            // empty half-bit (measured envelope), so the expected empty-half
            // level of a lone frame is the noise floor plus that leakage.
            let expected_off = self.block_mean + 0.35 * f.signal;
            let off_hot = self.p.cancel_off_ratio > 0.0
                && f.off_ratio >= self.p.cancel_off_ratio
                && f.off_level >= self.p.cancel_gap_ratio.max(1.0) * expected_off;
            let all = self.p.cancel_gap_ratio == 0.0 && self.p.cancel_off_ratio == 0.0;
            if f.fixed == 0 && !gap_hot && !off_hot && !all {
                continue;
            }
            let removed = cancel_frame(iq, start, &f.bytes);
            self.stats.cancelled += 1;
            let bucket = if removed < 0.5 { 0 } else if removed < 0.8 { 1 } else if removed < 0.9 { 2 } else if removed < 0.95 { 3 } else { 4 };
            self.stats.removed_hist[bucket] += 1;
            if removed < 0.3 {
                continue;
            }
            // Recompute magnitudes over the span and rescan a window around it.
            let s0 = (start.floor() as usize).saturating_sub(2);
            let n = (((8.0 + f.bytes.len() as f64 * 8.0) * SPB) as usize + 6).min(m.len() - s0);
            remag(&iq[2 * s0..2 * (s0 + n)], &mut m[s0..s0 + n]);
            let lo = s0.saturating_sub(LONG_SAMPLES);
            let hi = (s0 + n + LONG_SAMPLES + 24).min(m.len());
            if hi <= lo + LONG_SAMPLES + 2 {
                continue;
            }
            // The rescan is local and a collision is known: relax the gates
            // and let the CRC, repair and admission rules filter.
            let saved = self.p.clone();
            self.p.precheck_ratio = 1.3;
            self.p.gap_spikes = 6;
            self.p.preamble_ratio = 1.3;
            let found = self.run(&m[lo..hi], base + lo as u64);
            self.p = saved;
            for g in found {
                let key = ((g.start * 5.0).round() as u64, g.bytes.clone());
                if emitted.insert(key) {
                    self.stats.rescan_frames += 1;
                    out.push(g);
                }
            }
        }
        out.sort_by(|a, b| a.start.partial_cmp(&b.start).unwrap());
        out
    }

    /// Demodulate `m`, reporting frames whose preamble starts before
    /// `m.len() - LONG_SAMPLES`. `base` is the stream index of `m[0]`.
    pub fn run(&mut self, m: &[u16], base: u64) -> Vec<Frame> {
        let mut out = Vec::new();
        if m.len() < LONG_SAMPLES + 2 {
            return out;
        }
        let end = m.len() - LONG_SAMPLES;
        self.stats.samples += end as u64;
        // Integer gate in three straight passes, over sub-blocks small
        // enough to stay in L1: pairwise maxima (a pulse is the stronger of
        // its two samples), prefix sums (a gap sum is two subtractions),
        // and a branch-free ratio test into a hit mask. Slices are cut to
        // exact lengths so the loops vectorize. The spike count runs only
        // on hits.
        const SUB: usize = 4096;
        let ratio4 = (self.p.precheck_ratio * 256.0) as u32 * 4; // psum*11*256 > 4*ratio*256*gsum
        let max_spikes = self.p.gap_spikes;
        let mut pmax = std::mem::take(&mut self.pmax);
        let mut cum = std::mem::take(&mut self.cum);
        let mut hits = std::mem::take(&mut self.hits);
        let mut i = 0usize;
        let mut sub_start = usize::MAX; // sub-block currently in the scratch
        while i < end {
            if i < sub_start || i >= sub_start + SUB {
                // (Re)compute the scratch for the sub-block starting at i.
                sub_start = i;
                let len = SUB.min(end - i);
                let span = &m[i..i + len + 20];
                gate_passes(span, len, ratio4, &mut pmax, &mut cum, &mut hits);
                self.block_mean = cum[span.len()] as f32 / span.len() as f32;
            }
            // Jump to the next hit in this sub-block, eight bytes at a time.
            match next_hit(&hits, i - sub_start) {
                Some(k) => i = sub_start + k,
                None => {
                    i = sub_start + hits.len();
                    continue;
                }
            }
            let w: &[u16; 20] = m[i..i + 20].try_into().unwrap();
            let pmin = w[0].max(w[1]).min(w[2].max(w[3])).min(w[8].max(w[9])).min(w[10].max(w[11]));
            let spikes = (w[4] > pmin) as usize
                + (w[5] > pmin) as usize
                + (w[6] > pmin) as usize
                + (w[7] > pmin) as usize
                + (w[12] > pmin) as usize
                + (w[13] > pmin) as usize
                + (w[14] > pmin) as usize
                + (w[15] > pmin) as usize
                + (w[16] > pmin) as usize
                + (w[17] > pmin) as usize
                + (w[18] > pmin) as usize;
            if spikes > max_spikes {
                i += 1;
                continue;
            }
            self.stats.precheck_hits += 1;
            // Fine test at this sample (integer closed form); only a hit is
            // worth refining. Ratio test in fixed point.
            let ratio_q8f = (self.p.preamble_ratio * 256.0) as u32;
            let uni_q8 = (self.p.pulse_uniformity * 256.0) as u32;
            let fine = |w: &[u16]| -> Option<(f32, f32, f32)> {
                let (pmean, lg, pmin, pmax) = preamble_int(w);
                if pmin * 256 < uni_q8 * pmax {
                    return None;
                }
                // floor of one raw unit = 16 scaled, times the 396 fixed-point factor
                if pmin <= lg || pmean * 256 < ratio_q8f * lg.max(16 * 396) {
                    return None;
                }
                // back to the f32 units of `preamble`: levels per half-us
                let k = 1.0 / 396.0;
                Some(((pmean - lg) as f32 * k, pmean as f32 * k, lg as f32 * k))
            };
            let Some((score0, pm0, gm0)) = fine(&m[i..i + 20]) else {
                i += 1;
                continue;
            };
            self.stats.preamble_hits += 1;
            // Refine: the next two integer starts, then the phases around
            // the best of the three.
            let mut best = (i as f64, score0, pm0, gm0);
            for s in [i + 1, i + 2] {
                if s - sub_start >= hits.len() || hits[s - sub_start] == 0 {
                    continue;
                }
                if let Some((score, pm, gm)) = fine(&m[s..s + 20]) {
                    if score > best.1 {
                        best = (s as f64, score, pm, gm);
                    }
                }
            }
            let s0 = best.0;
            let phases: &[f64] = if self.p.phase_span >= 2 { &[-0.2, 0.2, -0.4, 0.4] } else { &[-0.2, 0.2] };
            for &ph in phases {
                let st = s0 + ph;
                if st < 0.0 {
                    continue;
                }
                if let Some((score, pm, gm)) = self.preamble_grid(m, st) {
                    if score > best.1 {
                        best = (st, score, pm, gm);
                    }
                }
            }
            let (start0, _, pulse_mean, gap_mean) = best;
            self.stats.decodes += 1;
            let mut start = start0;
            self.now = base + start0 as u64;
            let mut decoded = self.decode(m, start0);
            // A CRC-checked frame that failed or needed repair at the
            // preamble's timing gets re-sliced at neighbouring offsets.
            // Only what sliced as a CRC-checked format is worth retrying;
            // noise rarely reads as one, and a corrupted DF field is rare.
            let unhappy = self.last_known
                && match &decoded {
                    Some((b, fixed)) => matches!(b[0] >> 3, 11 | 17 | 18) && *fixed > 0,
                    None => matches!(self.last_df, 11 | 17 | 18),
                };
            if unhappy && !self.p.retry_offsets.is_empty() {
                self.stats.retries += 1;
                let mut best_fixed = decoded.as_ref().map_or(u8::MAX, |(_, f)| *f);
                let offsets = self.p.retry_offsets.clone();
                for d in offsets {
                    let st = start0 + d;
                    if st < 0.0 {
                        continue;
                    }
                    if let Some((b, f)) = self.decode(m, st) {
                        if matches!(b[0] >> 3, 11 | 17 | 18) && f < best_fixed {
                            best_fixed = f;
                            decoded = Some((b, f));
                            start = st;
                            self.stats.retry_wins += 1;
                            if f == 0 {
                                break;
                            }
                        }
                    }
                }
            }
            if decoded.is_some() {
                self.stats.crc_ok += 1;
            }
            let now = base + start as u64;
            match decoded.and_then(|f| self.admit(f, now)) {
                Some((bytes, fixed)) => {
                    let len_samples = (DATA_START + bytes.len() as f64 * 8.0 * SPB) as usize;
                    self.stats.frames += 1;
                    out.push(Frame {
                        start: base as f64 + start,
                        bytes,
                        signal: pulse_mean,
                        gap: gap_mean,
                        off_ratio: self.last_off_ratio,
                        off_level: self.last_off_level,
                        fixed,
                    });
                    i = start as usize + len_samples;
                }
                None => i += 1,
            }
        }
        self.pmax = pmax;
        self.cum = cum;
        self.hits = hits;
        out
    }

    /// Evaluate the candidate around `pos` (fractional sample index into
    /// `m`) for a frame known to be `expected`.
    pub fn probe(&self, m: &[u16], pos: f64, expected: &[u8]) -> Probe {
        let i0 = pos.floor() as usize;
        let mut precheck = false;
        let mut best: Option<(f64, f32, f32)> = None;
        for s in i0.saturating_sub(2)..=i0 + 2 {
            if s + LONG_SAMPLES + 2 > m.len() {
                break;
            }
            precheck |= self.precheck(m, s);
            for ph in PHASES {
                let st = s as f64 + ph;
                if let Some((score, pm)) = self.preamble(m, st) {
                    if best.map_or(true, |b| score > b.1) {
                        best = Some((st, score, pm));
                    }
                }
            }
        }
        let (start, score, pm) = best.unwrap_or((pos, 0.0, 0.0));
        let mut gap_total = 0f32;
        for &(a, b) in &GAPS {
            gap_total += energy(m, start + a * SPB, start + b * SPB);
        }
        let gm = gap_total / (GAP_LEN * SPB) as f32;
        let nbits = expected.len() * 8;
        let mut conf = vec![0f32; nbits];
        let mut wrong = Vec::new();
        for b in 0..nbits {
            let t0 = start + DATA_START + b as f64 * SPB;
            let mid = t0 + 0.5 * SPB;
            let (a, c) = (energy(m, t0, mid), energy(m, mid, t0 + SPB));
            conf[b] = (a - c).abs();
            let bit = (a > c) as u8;
            let exp = (expected[b / 8] >> (7 - b % 8)) & 1;
            if bit != exp {
                wrong.push(b);
            }
        }
        let mut order: Vec<usize> = (0..nbits).collect();
        order.sort_by(|&x, &y| conf[x].partial_cmp(&conf[y]).unwrap());
        let error_ranks = wrong
            .iter()
            .map(|w| order.iter().position(|o| o == w).unwrap())
            .collect();
        Probe {
            precheck,
            preamble: best.is_some(),
            best_start: start,
            best_score: score,
            bit_errors: wrong.len(),
            error_ranks,
            pulse_level: pm,
            gap_level: gm,
        }
    }

    /// Unrepaired CRC-checked frames confirm their aircraft; repaired and
    /// address-parity frames pass only for a confirmed aircraft. Returns
    /// the frame to emit.
    fn admit(&mut self, f: (Vec<u8>, u8), now: u64) -> Option<(Vec<u8>, u8)> {
        let (bytes, fixed) = f;
        let df = bytes[0] >> 3;
        let n = bytes.len();
        let confirmed = |known: &std::collections::HashMap<u32, u64>, a: u32| {
            matches!(known.get(&a), Some(&t) if now.saturating_sub(t) < KNOWN_WINDOW)
        };
        match df {
            11 | 17 | 18 => {
                let a = (bytes[1] as u32) << 16 | (bytes[2] as u32) << 8 | bytes[3] as u32;
                // 255 marks a DF11 with a nonzero interrogator code: not
                // repaired, but not trusted to confirm an aircraft either.
                if fixed == 255 {
                    return confirmed(&self.known, a).then_some((bytes, 0));
                }
                if fixed > 0 {
                    return confirmed(&self.known, a).then_some((bytes, fixed));
                }
                self.known.insert(a, now);
                if self.known.len() > 4096 {
                    self.known.retain(|_, t| now.saturating_sub(*t) < KNOWN_WINDOW);
                }
                Some((bytes, fixed))
            }
            _ => {
                let ap = (bytes[n - 3] as u32) << 16 | (bytes[n - 2] as u32) << 8 | bytes[n - 1] as u32;
                let a = self.crc.of(&bytes[..n - 3]) ^ ap;
                confirmed(&self.known, a).then_some((bytes, fixed))
            }
        }
    }

    /// The fine preamble test at a start on the fifth-of-a-sample grid,
    /// from the weight tables. Same result as `preamble` for such starts.
    #[inline]
    fn preamble_grid(&self, m: &[u16], st: f64) -> Option<(f32, f32, f32)> {
        let i = st.floor() as usize;
        let k = ((st - i as f64) * 5.0).round() as usize % 5;
        if i + 21 > m.len() {
            return None;
        }
        let w: &[u16; 21] = m[i..i + 21].try_into().unwrap();
        let pw = &self.phases[k];
        let pulse = |(first, t): &(usize, [i32; 3])| -> u32 {
            (t[0] * w[*first] as i32 + t[1] * w[first + 1] as i32 + t[2] * w[first + 2] as i32) as u32
        };
        let p = [pulse(&pw.pulse_taps[0]), pulse(&pw.pulse_taps[1]), pulse(&pw.pulse_taps[2]), pulse(&pw.pulse_taps[3])];
        let mut g = 0i32;
        for j in 0..21 {
            g += pw.gap[j] * w[j] as i32;
        }
        let g = g as u32;
        // levels: pulse over 6 fifths, gap over 66 fifths; scale both by 396
        let lp = [p[0] * 66, p[1] * 66, p[2] * 66, p[3] * 66];
        let lg = g * 6;
        let pmean = (lp[0] + lp[1] + lp[2] + lp[3]) / 4;
        let pmin = lp[0].min(lp[1]).min(lp[2]).min(lp[3]);
        let ratio_q8 = (self.p.preamble_ratio * 256.0) as u32;
        if pmin <= lg || pmean * 256 < ratio_q8 * lg.max(16 * 396) {
            return None;
        }
        let k = 1.0 / 396.0;
        Some(((pmean - lg) as f32 * k, pmean as f32 * k, lg as f32 * k))
    }

    /// Cheap integer gate: each pulse's strongest sample must exceed the
    /// mean of the samples inside the two long gaps.
    #[inline]
    fn precheck(&self, m: &[u16], i: usize) -> bool {
        let p = |a: usize, b: usize| m[i + a].max(m[i + b]) as f32;
        let (p0, p1, p2, p3) = (p(0, 1), p(2, 3), p(8, 9), p(10, 11));
        let pmin = p0.min(p1).min(p2).min(p3);
        // Gap samples: at most `gap_spikes` may exceed the weakest pulse,
        // and their mean sets the pulse-to-gap ratio.
        let (mut spikes, mut gsum) = (0usize, 0f32);
        for &g in m[i + 4..i + 8].iter().chain(&m[i + 12..i + 19]) {
            let g = g as f32;
            gsum += g;
            spikes += (g > pmin) as usize;
        }
        spikes <= self.p.gap_spikes
            && p0 + p1 + p2 + p3 > 4.0 * self.p.precheck_ratio * (gsum / 11.0)
    }

    /// Preamble test at fractional start `st`: Some((score, pulse level)).
    /// Levels are energies per half microsecond.
    fn preamble(&self, m: &[u16], st: f64) -> Option<(f32, f32)> {
        let unit = (0.5 * SPB) as f32;
        let mut pm = 0f32;
        let mut pmin = f32::MAX;
        for &(a, b) in &PULSES {
            let e = energy(m, st + a * SPB, st + b * SPB) / unit;
            pm += e;
            pmin = pmin.min(e);
        }
        pm /= 4.0;
        let mut gap_total = 0f32;
        for &(a, b) in &GAPS {
            gap_total += energy(m, st + a * SPB, st + b * SPB);
        }
        let gm = gap_total / (GAP_LEN * SPB) as f32;
        // Magnitudes carry iq::MAG_SCALE (16): the floor of one raw unit is 16.
        if pmin <= gm || pm < self.p.preamble_ratio * gm.max(16.0) {
            return None;
        }
        if self.p.noise_sigmas > 0.0 {
            // Spread of the raw gap samples around their mean.
            let i0 = st as usize;
            let mut var = 0f32;
            let mut n = 0f32;
            for &g in m[i0 + 4..i0 + 8].iter().chain(&m[i0 + 12..i0 + 19]) {
                let g = g as f32;
                var += (g - gm) * (g - gm);
                n += 1.0;
            }
            let sd = (var / n).sqrt();
            if pm - gm < self.p.noise_sigmas * sd {
                return None;
            }
        }
        Some((pm - gm, pm))
    }

    /// Slice bits from `st`, check and repair the CRC.
    fn decode(&mut self, m: &[u16], st: f64) -> Option<(Vec<u8>, u8)> {
        let mut hi: u64 = 0; // bits 0..56, MSB first
        let mut lo: u64 = 0; // bits 56..112
        let mut conf = [0i32; 112];
        // Positions in fifths of a sample: the data starts 96 fifths after
        // the preamble, a bit is 12 fifths, a half bit 6. Each half-bit
        // integral covers three samples with weights fixed by its phase.
        let f0 = (st * 5.0).round() as i64 + 96;
        let matched = self.p.slicer == Slicer::Matched;
        let tap0 = self.p.tap0;
        // Half-bit positions advance by 6 fifths each: the sample index and
        // the phase are tracked, never divided.
        let mut pos = (f0 / 5) as usize;
        let mut ph = (f0 % 5) as usize;
        let mut slice = |b: usize| -> (i32, i32) {
            if matched {
                let t0 = st + DATA_START + b as f64 * SPB;
                let a = pulse_corr(m, t0, tap0);
                let c = pulse_corr(m, t0 + 0.5 * SPB, tap0);
                return ((a * 5.0) as i32, (c * 5.0) as i32);
            }
            let a = half_bit_at(m, pos, ph);
            // SAFETY: ph is always 0..5 (it comes from HALF_STEP or f0 % 5).
            let (dp, np) = unsafe { *HALF_STEP.get_unchecked(ph) };
            pos += dp;
            ph = np;
            let c = half_bit_at(m, pos, ph);
            let (dp, np) = unsafe { *HALF_STEP.get_unchecked(ph) };
            pos += dp;
            ph = np;
            (a, c)
        };
        for b in 0..5 {
            let (a, c) = slice(b);
            hi = hi << 1 | (a > c) as u64;
            conf[b] = (a - c).abs();
        }
        let df = hi as u8;
        self.last_df = df;
        // Only formats this decoder handles are worth slicing further;
        // most noise reads as some other format and stops here.
        if !matches!(df, 0 | 4 | 5 | 11 | 16 | 17 | 18 | 20 | 21) {
            return None;
        }
        let nbits = if long_df(df) { 112 } else { 56 };
        let (mut on, mut off) = (0i64, 0i64);
        for b in 5..56 {
            let (a, c) = slice(b);
            hi = hi << 1 | (a > c) as u64;
            on += a.max(c) as i64;
            off += a.min(c) as i64;
            // SAFETY: b < 112.
            unsafe { *conf.get_unchecked_mut(b) = (a - c).abs() };
        }
        for b in 56..nbits {
            let (a, c) = slice(b);
            lo = lo << 1 | (a > c) as u64;
            on += a.max(c) as i64;
            off += a.min(c) as i64;
            unsafe { *conf.get_unchecked_mut(b) = (a - c).abs() };
        }
        self.last_off_ratio = if on > 0 { off as f32 / on as f32 } else { 1.0 };
        // off is a sum of half-bit energies in fifths (6 fifths each) over nbits - 5 bits
        self.last_off_level = off as f32 / (6.0 * (nbits - 5) as f32);
        let mut buf = [0u8; 14];
        buf[..7].copy_from_slice(&(hi << 8).to_be_bytes()[..7]);
        if nbits == 112 {
            buf[7..14].copy_from_slice(&(lo << 8).to_be_bytes()[..7]);
        }
        let bytes = &mut buf[..nbits / 8];
        if matches!(df, 11 | 17 | 18) {
            // Near a confirmed aircraft: the sliced address within
            // `near_bits` flips of one seen in the window. Weak frames often
            // carry errors in the address field too; noise is almost never
            // this close to any of a few dozen addresses.
            let a = (bytes[1] as u32) << 16 | (bytes[2] as u32) << 8 | bytes[3] as u32;
            let now = self.now;
            let near = self.p.near_bits;
            self.last_known = self
                .known
                .iter()
                .any(|(&k, &t)| now.saturating_sub(t) < KNOWN_WINDOW && (k ^ a).count_ones() <= near);
        } else {
            self.last_known = false;
        }
        match df {
            17 | 18 => {
                let deep = self.last_known;
                self.repair(bytes, &conf[..nbits], 0, deep)
            }
            11 => {
                // Residual is the interrogator code; 0..79 are assigned.
                // A nonzero code is a weak check (80 in 2^24 random frames
                // pass), so `admit` treats it like a repaired frame.
                let r = self.crc.of(bytes);
                if r == 0 {
                    Some((bytes.to_vec(), 0))
                } else if r < 80 {
                    Some((bytes.to_vec(), 255))
                } else {
                    None
                }
            }
            // Address-parity frames cannot be checked here; `admit`
            // validates the recovered address against known aircraft, and
            // may repair one low-confidence bit toward a known address.
            0 | 4 | 5 | 16 | 20 | 21 => {
                // Address-parity: accept a confirmed address as is. Otherwise
                // a single wrong bit moves the recovered address by that
                // bit's syndrome, so for each confirmed aircraft the
                // difference is looked up among the syndromes; a hit whose
                // bit ranks among the `ap_fix` least confident is repaired.
                let n = bytes.len();
                let ap = (bytes[n - 3] as u32) << 16 | (bytes[n - 2] as u32) << 8 | bytes[n - 1] as u32;
                let a = self.crc.of(&bytes[..n - 3]) ^ ap;
                let now = self.now;
                if matches!(self.known.get(&a), Some(&t) if now.saturating_sub(t) < KNOWN_WINDOW) {
                    return Some((bytes.to_vec(), 0));
                }
                if self.p.ap_fix == 0 {
                    return None;
                }
                let sorted = if n == 14 { &self.asyn112_sorted } else { &self.asyn56_sorted };
                for (&k, &t) in &self.known {
                    if now.saturating_sub(t) >= KNOWN_WINDOW {
                        continue;
                    }
                    if let Ok(pos) = sorted.binary_search_by_key(&(k ^ a), |&(s, _)| s) {
                        let i = sorted[pos].1;
                        let rank = conf[..nbits].iter().filter(|&&c| c < conf[i]).count();
                        if rank < self.p.ap_fix {
                            bytes[i / 8] ^= 0x80 >> (i % 8);
                            return Some((bytes.to_vec(), 1));
                        }
                    }
                }
                None
            }
            _ => None,
        }
    }

    /// Accept a frame whose CRC residual is `target`, or repair up to
    /// `max_fix` low-confidence bits whose syndromes explain the residual.
    /// `deep` allows repairs of two or more bits; without it only a
    /// single flip is tried. Multi-bit repairs are only admitted for
    /// confirmed aircraft, so searching for them elsewhere is wasted.
    fn repair(&self, bytes: &mut [u8], conf: &[i32], target: u32, deep: bool) -> Option<(Vec<u8>, u8)> {
        let r = self.crc.of(bytes) ^ target;
        if r == 0 {
            return Some((bytes.to_vec(), 0));
        }
        if self.p.max_fix == 0 {
            return None;
        }
        let max_fix = if deep { self.p.max_fix } else { self.p.max_fix.min(1) };
        let (syn, sorted) = if bytes.len() == 14 {
            (&self.syn112, &self.syn112_sorted)
        } else {
            (&self.syn56, &self.syn56_sorted)
        };
        // Single bit: look the residual up among the syndromes, then check
        // that the bit ranks among the `soft1` least confident. No sort.
        if let Ok(pos) = sorted.binary_search_by_key(&r, |&(s, _)| s) {
            let i = sorted[pos].1;
            let rank = conf.iter().filter(|&&c| c < conf[i]).count();
            if rank < self.p.soft1 {
                bytes[i / 8] ^= 0x80 >> (i % 8);
                return Some((bytes.to_vec(), 1));
            }
        }
        if max_fix < 2 {
            return None;
        }
        // Deeper repairs need the confidence order of the least confident bits.
        let keep = self.p.soft1.max(self.p.soft2).max(self.p.soft3).max(self.p.soft4).max(self.p.soft5).min(conf.len());
        let mut order: Vec<usize> = (0..conf.len()).collect();
        if keep < order.len() {
            order.select_nth_unstable_by_key(keep, |&i| conf[i]);
            order.truncate(keep);
        }
        order.sort_by_key(|&i| conf[i]);
        let cand = &order[..self.p.soft2.min(order.len())];
        for (x, &i) in cand.iter().enumerate() {
            for &j in &cand[x + 1..] {
                if syn[i] ^ syn[j] == r {
                    bytes[i / 8] ^= 0x80 >> (i % 8);
                    bytes[j / 8] ^= 0x80 >> (j % 8);
                    return Some((bytes.to_vec(), 2));
                }
            }
        }
        if max_fix < 3 {
            return None;
        }
        let cand = &order[..self.p.soft3.min(order.len())];
        for (x, &i) in cand.iter().enumerate() {
            for (y, &j) in cand.iter().enumerate().skip(x + 1) {
                let need = r ^ syn[i] ^ syn[j];
                for &k in &cand[y + 1..] {
                    if syn[k] == need {
                        bytes[i / 8] ^= 0x80 >> (i % 8);
                        bytes[j / 8] ^= 0x80 >> (j % 8);
                        bytes[k / 8] ^= 0x80 >> (k % 8);
                        return Some((bytes.to_vec(), 3));
                    }
                }
            }
        }
        for (depth, window) in [(4usize, self.p.soft4), (5, self.p.soft5)] {
            if (max_fix as usize) < depth {
                return None;
            }
            let cand = &order[..window.min(order.len())];
            let mut picked = Vec::with_capacity(depth);
            if find_flips(syn, cand, r, depth, 0, &mut picked) {
                for &i in &picked {
                    bytes[i / 8] ^= 0x80 >> (i % 8);
                }
                return Some((bytes.to_vec(), depth as u8));
            }
        }
        None
    }
}

/// Depth-first search for `depth` bits of `cand` (from index `from`) whose
/// syndromes XOR to `target`. The last bit is looked up, not enumerated.
fn find_flips(
    syn: &[u32],
    cand: &[usize],
    target: u32,
    depth: usize,
    from: usize,
    picked: &mut Vec<usize>,
) -> bool {
    if depth == 1 {
        for &k in &cand[from..] {
            if syn[k] == target {
                picked.push(k);
                return true;
            }
        }
        return false;
    }
    for x in from..cand.len().saturating_sub(depth - 1) {
        picked.push(cand[x]);
        if find_flips(syn, cand, target ^ syn[cand[x]], depth - 1, x + 1, picked) {
            return true;
        }
        picked.pop();
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthesize the magnitude stream of one frame at a fractional start.
    fn synth(bytes: &[u8], start: f64, amp: f32, noise: f32) -> Vec<u16> {
        let n = (start + DATA_START + bytes.len() as f64 * 8.0 * SPB) as usize + 40;
        let mut m = vec![noise; n];
        let mut pulse = |t_us: f64, w_us: f64| {
            let a = start + t_us * SPB;
            let b = start + (t_us + w_us) * SPB;
            for i in (a.floor() as usize)..=(b.ceil() as usize).min(n - 1) {
                let lo = (i as f64).max(a);
                let hi = ((i + 1) as f64).min(b);
                if hi > lo {
                    m[i] += amp * (hi - lo) as f32;
                }
            }
        };
        for t in [0.0, 1.0, 3.5, 4.5] {
            pulse(t, 0.5);
        }
        for (b, byte) in bytes.iter().enumerate() {
            for k in 0..8 {
                let bit = (byte >> (7 - k)) & 1;
                let t = 8.0 + (b * 8 + k) as f64;
                pulse(if bit == 1 { t } else { t + 0.5 }, 0.5);
            }
        }
        m.iter().map(|&x| (x * 16.0).round() as u16).collect()
    }

    const DF17: [u8; 14] = [
        0x8D, 0x40, 0x62, 0x1D, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC, 0x28, 0x63, 0xA7,
    ];

    #[test]
    fn table_crc_matches_reference() {
        let c = Crc24::new();
        assert_eq!(c.of(&DF17), mb_modes::crc24(&DF17));
        let junk = [0x5Du8, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        assert_eq!(c.of(&junk), mb_modes::crc24(&junk));
        assert_eq!(c.of(&junk[..4]), mb_modes::crc24(&junk[..4]));
    }

    #[test]
    fn clean_frame_at_every_phase() {
        assert_eq!(mb_modes::crc24(&DF17), 0);
        let mut d = Demodulator::new(Params::default());
        for ph in [0.0, 0.13, 0.5, 0.77] {
            let mut m = synth(&DF17, 50.0 + ph, 100.0, 5.0);
            m.extend(vec![80u16; LONG_SAMPLES + 4]);
            let f = d.run(&m, 0);
            assert_eq!(f.len(), 1, "phase {ph}");
            assert_eq!(f[0].bytes, DF17);
            assert_eq!(f[0].fixed, 0);
            assert!((f[0].start - (50.0 + ph)).abs() < 0.5, "start {}", f[0].start);
        }
    }

    #[test]
    fn one_and_two_bit_errors_are_repaired() {
        let mut d = Demodulator::new(Params::default());
        for flips in [vec![37usize], vec![37, 90]] {
            let mut bad = DF17;
            for &i in &flips {
                bad[i / 8] ^= 0x80 >> (i % 8);
            }
            // A clean copy first, so the aircraft is confirmed before the
            // damaged frame arrives.
            let mut m = synth(&DF17, 20.0, 100.0, 5.0);
            m.extend(vec![80u16; 40]);
            let off = m.len() as f64;
            m.extend(synth(&bad, 20.0, 100.0, 5.0));
            // Attenuate the flipped bits so they rank as least confident.
            for &i in &flips {
                let t0 = off + 20.0 + DATA_START + i as f64 * SPB;
                for s in (t0 as usize)..((t0 + SPB) as usize + 1) {
                    m[s] = 80 + ((m[s] as f32 - 80.0) * 0.2) as u16;
                }
            }
            m.extend(vec![80u16; LONG_SAMPLES + 4]);
            let f = d.run(&m, 0);
            assert_eq!(f.len(), 2);
            assert_eq!(f[1].bytes, DF17);
            // Timing retries may find an offset that needs fewer flips.
            assert!(f[1].fixed as usize <= flips.len() && f[1].fixed > 0);
        }
    }
}
