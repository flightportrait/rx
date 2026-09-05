//! rx: the Station radio. Milestone R0: drive the dongle, keep an honest
//! sample clock, run the decoder live, report rates.
//!
//! One thread reads the dongle into fixed blocks; the main thread converts
//! and decodes. The sample counter is the clock every frame timestamp
//! derives from (12 MHz Beast ticks = 5 per sample), so lost samples must
//! advance it: the reader compares samples delivered with wall time and
//! accounts a gap when the dongle falls behind by more than a block.

mod beast;
mod beast_in;
mod gain;
mod reduce;
mod sdr;
mod stats;
mod track;

use anyhow::Result;
use clap::Parser;
use std::io::Write;
use std::sync::mpsc;
use std::time::{Duration, Instant};

const SAMPLE_RATE: u32 = 2_400_000;
const FREQ: u32 = 1_090_000_000;
/// USB transfers kept queued in librtlsdr and their size (readsb's
/// values): 15 x 256 KiB, each 55 ms of samples.
const BUF_COUNT: u32 = 15;
const BUF_BYTES: usize = 262_144;

#[derive(Parser)]
#[command(about = "The Station radio: RTL-SDR in, Mode S frames out")]
struct Args {
    /// Device index.
    #[arg(long, default_value_t = 0)]
    device: u32,
    /// Device serial (overrides --device).
    #[arg(long)]
    serial: Option<String>,
    /// Tuner gain in dB, "auto" for the measured gain loop (starts at the
    /// top step, moves one step per two agreeing 10 s windows), or "agc"
    /// for the hardware AGC.
    #[arg(long, default_value = "49.6")]
    gain: String,
    /// Power the antenna port (bias tee). Only for a powered LNA.
    #[arg(long, default_value_t = false)]
    bias_tee: bool,
    /// Write raw I/Q to this file (UC8) as well.
    #[arg(long)]
    record: Option<String>,
    /// Shadow recordings: every `clip_every` seconds, write `clip_seconds`
    /// of raw I/Q into this directory (UC8, named by UTC time), keeping the
    /// newest `clip_keep` files. A nightly replay through readsb scores the
    /// live radio against it.
    #[arg(long)]
    clip_dir: Option<String>,
    #[arg(long, default_value_t = 600)]
    clip_every: u64,
    #[arg(long, default_value_t = 10)]
    clip_seconds: u64,
    #[arg(long, default_value_t = 144)]
    clip_keep: usize,
    /// Stop after this many seconds (0 = run until killed).
    #[arg(long, default_value_t = 0)]
    seconds: u64,
    /// List devices and exit.
    #[arg(long, default_value_t = false)]
    list: bool,
    /// Report every second instead of every minute.
    #[arg(long, default_value_t = false)]
    verbose: bool,
    /// Log the sample clock on every settled read (lag, baseline, read time).
    #[arg(long, default_value_t = false, hide = true)]
    clock_debug: bool,
    /// Beast server address ("" = none).
    #[arg(long, default_value = "0.0.0.0:30005")]
    beast_listen: String,
    /// Outbound connector, readsb's form: host,port,protocol[,uuid=...];
    /// protocol is beast_out, beast_reduce_out, beast_reduce_plus_out or
    /// beast_in (pull a Beast stream in). Repeatable.
    #[arg(long)]
    net_connector: Vec<String>,
    /// Beast input port (readsb's flag): MLAT results from mlatc arrive here.
    #[arg(long)]
    net_bi_port: Option<u16>,
    /// Reduced-stream interval, milliseconds (readsb's default 250; the
    /// aggregators are happy with 500).
    #[arg(long, default_value_t = 500)]
    reduce_interval: u64,
    /// Write aircraft.json into this directory every second.
    #[arg(long)]
    write_json: Option<String>,
    /// Receiver position, for range figures.
    #[arg(long, default_value_t = 0.0)]
    lat: f64,
    #[arg(long, default_value_t = 0.0)]
    lon: f64,

    // readsb's flags, accepted so a station configured for readsb runs rx
    // by changing the binary path alone.
    /// Accepted for readsb compatibility; only rtlsdr is supported.
    #[arg(long, default_value = "rtlsdr", hide = true)]
    device_type: String,
    /// Accepted for readsb compatibility (no effect).
    #[arg(long, default_value_t = false, hide = true)]
    quiet: bool,
    /// Accepted for readsb compatibility (no effect).
    #[arg(long, default_value_t = false, hide = true)]
    net: bool,
    /// readsb's Beast output port: serves on 0.0.0.0:PORT.
    #[arg(long, hide = true)]
    net_bo_port: Option<u16>,
    /// Accepted for readsb compatibility; aircraft.json is written every second.
    #[arg(long, default_value_t = 1, hide = true)]
    write_json_every: u32,
}

/// A block of samples with the stream index of its first sample and the
/// count of samples at or near full scale.
struct Block {
    first_sample: u64,
    bytes: Vec<u8>,
    clipped: u32,
}

/// How the tuner is driven.
#[derive(Clone, Copy)]
enum GainMode {
    /// Hardware AGC in tuner and demodulator.
    Agc,
    /// Manual, tenths of a dB.
    Fixed(i32),
}

struct RadioCfg {
    index: u32,
    serial: Option<String>,
    gain: GainMode,
    bias_tee: bool,
}

/// Open the dongle and apply the configuration. Retries until a device
/// answers, so a station that boots before its dongle enumerates, or
/// loses it mid-run, comes back on its own.
fn open_radio(cfg: &RadioCfg) -> sdr::Device {
    let mut wait = 2u64;
    loop {
        match sdr::Device::open(cfg.index, cfg.serial.as_deref()).and_then(|mut dev| {
            configure(&mut dev, cfg)?;
            Ok(dev)
        }) {
            Ok(dev) => return dev,
            Err(e) => {
                eprintln!("rx: no dongle ({e}); retry in {wait} s");
                std::thread::sleep(Duration::from_secs(wait));
                wait = (wait + 2).min(10);
            }
        }
    }
}

fn configure(dev: &mut sdr::Device, cfg: &RadioCfg) -> Result<()> {
    dev.set_sample_rate(SAMPLE_RATE)?;
    dev.set_center_freq(FREQ)?;
    match cfg.gain {
        GainMode::Agc => {
            dev.set_agc()?;
            eprintln!(
                "rx: {} at {} MS/s, hardware AGC",
                dev.name,
                SAMPLE_RATE as f64 / 1e6
            );
        }
        GainMode::Fixed(t) => {
            let got = dev.set_gain(t)?;
            eprintln!(
                "rx: {} at {} MS/s, gain {:.1} dB",
                dev.name,
                SAMPLE_RATE as f64 / 1e6,
                got as f32 / 10.0
            );
        }
    }
    dev.set_bias_tee(cfg.bias_tee)?;
    dev.reset_buffer()?;
    Ok(())
}

/// Samples with I or Q at or near full scale (0..5 or 250..255).
fn count_clipped(bytes: &[u8]) -> u32 {
    bytes.iter().filter(|&&b| b >= 250 || b <= 5).count() as u32
}

/// UTC name for a clip: YYYYMMDDTHHMMSSZ from a unix time, no libraries.
fn unix_utc_name(t: f64) -> String {
    let secs = t as i64;
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    // civil from days (Howard Hinnant's algorithm)
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!(
        "{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z",
        rem / 3600,
        (rem % 3600) / 60,
        rem % 60
    )
}

/// Keep the newest `keep` clips in `dir`.
fn prune_clips(dir: &std::path::Path, keep: usize) {
    let mut names: Vec<String> = std::fs::read_dir(dir)
        .map(|it| {
            it.filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|n| n.ends_with(".cu8"))
                .collect()
        })
        .unwrap_or_default();
    names.sort();
    while names.len() > keep {
        let old = names.remove(0);
        let _ = std::fs::remove_file(dir.join(old));
    }
}

fn main() -> Result<()> {
    let a = Args::parse();
    if a.list {
        for d in sdr::list() {
            println!("{}: {} serial {}", d.index, d.name, d.serial);
        }
        return Ok(());
    }
    anyhow::ensure!(
        a.device_type == "rtlsdr",
        "only --device-type rtlsdr is supported (got {})",
        a.device_type
    );
    let software_gain = a.gain == "auto";
    let gain_mode = if a.gain == "agc" {
        GainMode::Agc
    } else if software_gain {
        GainMode::Fixed(496) // the loop starts at the top and steps down on clipping
    } else {
        GainMode::Fixed((a.gain.parse::<f32>()? * 10.0).round() as i32)
    };
    let mut cfg = RadioCfg {
        index: a.device,
        serial: a.serial.clone(),
        gain: gain_mode,
        bias_tee: a.bias_tee,
    };
    let mut dev = open_radio(&cfg);
    let gain_steps = dev.gains.clone();

    // Reader thread: blocks to the decoder, sample clock kept honest, gain
    // changes taken from the controller, the dongle reopened when lost.
    let clock_debug = a.clock_debug;
    let (tx, rx) = mpsc::sync_channel::<Block>(16);
    let (gain_tx, gain_rx) = mpsc::channel::<i32>();
    let reader = std::thread::spawn(move || -> Result<()> {
        let t0 = Instant::now();
        let mut delivered: u64 = 0; // samples delivered by the dongle
        let mut clock: u64 = 0; // stream index including accounted gaps
        let mut gaps: u64 = 0;
        // The clock is the count of delivered samples, as in readsb, not
        // corrected from wall time. (The half-percent "drift" seen before
        // 2026-09-05 was sample loss from synchronous reads; see sdr::run.)
        // A genuine USB loss shows as a discontinuity that MLAT servers
        // detect as a clock reset; a dongle outage (below) is such a reset
        // and is logged.
        let mut short_reads: u64 = 0;
        let mut last_short_report = Instant::now();
        loop {
            let mut applied: Option<i32> = None;
            let mut stop = false;
            let run = dev.run(BUF_COUNT, BUF_BYTES as u32, |bytes, d| {
                while let Ok(t) = gain_rx.try_recv() {
                    match d.set_gain(t) {
                        Ok(got) => applied = Some(got),
                        Err(e) => eprintln!("rx: set gain: {e}"),
                    }
                }
                let buf = bytes[..bytes.len() & !1].to_vec();
                let samples = (buf.len() / 2) as u64;
                if clock_debug {
                    let expected = (t0.elapsed().as_secs_f64() * SAMPLE_RATE as f64) as i64;
                    let lag = expected - (delivered + samples) as i64;
                    eprintln!(
                        "rx: clock lag {:.1} ms samples {}",
                        lag as f64 / SAMPLE_RATE as f64 * 1e3,
                        samples
                    );
                }
                if (samples as usize) < BUF_BYTES / 4 {
                    short_reads += 1;
                }
                if short_reads > 0 && last_short_report.elapsed() >= Duration::from_secs(60) {
                    eprintln!("rx: {short_reads} short reads in the last minute");
                    short_reads = 0;
                    last_short_report = Instant::now();
                }
                let first = clock;
                clock += samples;
                delivered += samples;
                let clipped = count_clipped(&buf);
                if tx
                    .send(Block {
                        first_sample: first,
                        bytes: buf,
                        clipped,
                    })
                    .is_err()
                {
                    stop = true;
                    d.cancel();
                }
            });
            if let Some(got) = applied {
                cfg.gain = GainMode::Fixed(got);
            }
            if stop {
                return Ok(());
            }
            // The stream ended without being asked to: the dongle is gone.
            // The outage is a gap: the clock advances by wall time so the
            // frames after the replug keep true timestamps.
            let why = match run {
                Ok(()) => "USB stream ended".to_string(),
                Err(e) => e.to_string(),
            };
            eprintln!("rx: dongle lost ({why}), reopening");
            let lost_at = Instant::now();
            drop(dev);
            dev = open_radio(&cfg);
            let gap = (lost_at.elapsed().as_secs_f64() * SAMPLE_RATE as f64) as u64;
            clock += gap;
            delivered += gap;
            gaps += 1;
            eprintln!(
                "rx: dongle back after {:.1} s (gap {})",
                lost_at.elapsed().as_secs_f64(),
                gaps
            );
        }
    });

    let hub = beast::Hub::new();
    let listen = match a.net_bo_port {
        Some(p) => format!("0.0.0.0:{p}"),
        None => a.beast_listen.clone(),
    };
    if !listen.is_empty() {
        hub.serve(&listen)?;
        eprintln!("rx: Beast server on {listen}");
    }
    let (in_tx, in_rx) = mpsc::channel::<beast_in::InFrame>();
    if let Some(p) = a.net_bi_port {
        beast_in::listen(&format!("0.0.0.0:{p}"), in_tx.clone())?;
        eprintln!("rx: Beast input on 0.0.0.0:{p}");
    }
    for c in &a.net_connector {
        let parts: Vec<&str> = c.split(',').map(|p| p.trim()).collect();
        anyhow::ensure!(
            parts.len() >= 3,
            "--net-connector wants host,port,protocol[,uuid=...]: {c}"
        );
        if parts[2] == "beast_in" {
            beast_in::connect(format!("{}:{}", parts[0], parts[1]), in_tx.clone());
            continue;
        }
        let (stream, plus) = match parts[2] {
            "beast_out" => (beast::Stream::Full, false),
            "beast_reduce_out" => (beast::Stream::Reduced, false),
            "beast_reduce_plus_out" => (beast::Stream::Reduced, true),
            other => anyhow::bail!("unknown connector protocol {other} in {c}"),
        };
        let uuid = parts
            .iter()
            .skip(3)
            .find_map(|p| p.strip_prefix("uuid="))
            .map(str::to_string);
        if plus && uuid.is_none() {
            eprintln!("rx: {c}: beast_reduce_plus_out without uuid=; the aggregator will assign a random identity");
        }
        hub.connect(format!("{}:{}", parts[0], parts[1]), stream, uuid);
    }
    let mut reducer = reduce::Reducer::new(a.reduce_interval);
    let mut rec = match &a.record {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p)?)),
        None => None,
    };
    let clip_dir = a.clip_dir.as_ref().map(std::path::PathBuf::from);
    if let Some(d) = &clip_dir {
        std::fs::create_dir_all(d)?;
    }
    let mut clip: Option<(std::io::BufWriter<std::fs::File>, u64)> = None; // writer, samples left
    let mut next_clip = Instant::now();
    let lut = iq::MagLut::new();
    let mut d = demod::Demodulator::new(demod::Params::default());
    const TAIL: usize = 400;
    let mut mag: Vec<u16> = Vec::new();
    let mut raw: Vec<u8> = Vec::new();
    let mut base: u64 = 0; // stream index of mag[0]
    let remag = |b: &[u8], o: &mut [u16]| {
        let mut v = Vec::with_capacity(o.len());
        if cfg!(target_arch = "aarch64") {
            iq::magnitudes_f32(b, &mut v);
        } else {
            lut.magnitudes(b, &mut v);
        }
        o.copy_from_slice(&v);
    };
    let started = Instant::now();
    let mut last_report = Instant::now();
    let mut report_ticks: u64 = 0;
    let mut tracker = track::Tracker::new(a.lat, a.lon);
    let json_dir = a.write_json.as_ref().map(std::path::PathBuf::from);
    if let Some(d) = &json_dir {
        std::fs::create_dir_all(d)?;
    }
    let wall = || {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64()
    };
    let gain_db = a.gain.parse::<f32>().ok();
    let mut st = stats::Stats::new(wall(), gain_db);
    let (mut frames_1s, mut frames_total, mut aircraft): (
        u64,
        u64,
        std::collections::HashSet<u32>,
    ) = (0, 0, Default::default());
    let mut expect_first: u64 = 0;
    // Software gain: one window every 10 s of clipping, strong-frame
    // clipping and the noise floor (mean magnitude of a slice of every
    // block), decided by the controller, applied by the reader.
    let mut gainer = software_gain.then(|| gain::Controller::new(gain_steps.clone(), 496));
    let (mut win_samples, mut win_clipped, mut win_strong, mut win_noise, mut win_noise_n): (
        u64,
        u64,
        bool,
        f64,
        u64,
    ) = (0, 0, false, 0.0, 0);
    let mut win_start = Instant::now();
    for blk in rx.iter() {
        if gainer.is_some() {
            win_samples += (blk.bytes.len() / 2) as u64;
            win_clipped += blk.clipped as u64;
        }
        if let Some(w) = rec.as_mut() {
            w.write_all(&blk.bytes)?;
        }
        if let Some(dir) = &clip_dir {
            if clip.is_none() && Instant::now() >= next_clip {
                let name = format!("{}.cu8", unix_utc_name(wall()));
                match std::fs::File::create(dir.join(&name)) {
                    Ok(f) => {
                        clip = Some((
                            std::io::BufWriter::new(f),
                            a.clip_seconds * SAMPLE_RATE as u64,
                        ))
                    }
                    Err(e) => eprintln!("rx: clip: {e}"),
                }
                next_clip = Instant::now() + Duration::from_secs(a.clip_every);
                prune_clips(dir, a.clip_keep);
            }
            if let Some((w, left)) = clip.as_mut() {
                let take = (*left as usize * 2).min(blk.bytes.len());
                w.write_all(&blk.bytes[..take])?;
                *left -= (take / 2) as u64;
                if *left == 0 {
                    clip = None;
                }
            }
        }
        // A gap in the stream: the carried tail no longer touches the new
        // block, so start fresh at the new position.
        if blk.first_sample != expect_first {
            if expect_first != 0 && blk.first_sample > expect_first {
                st.dropped(blk.first_sample - expect_first);
            }
            mag.clear();
            raw.clear();
            base = blk.first_sample;
        }
        st.samples((blk.bytes.len() / 2) as u64);
        expect_first = blk.first_sample + (blk.bytes.len() / 2) as u64;
        raw.extend_from_slice(&blk.bytes);
        if cfg!(target_arch = "aarch64") {
            iq::magnitudes_f32(&blk.bytes, &mut mag);
        } else {
            lut.magnitudes(&blk.bytes, &mut mag);
        }
        if gainer.is_some() {
            // noise floor: mean magnitude of the newest 4096 samples
            let n = mag.len();
            let slice = &mag[n.saturating_sub(4096)..];
            if !slice.is_empty() {
                win_noise += slice.iter().map(|&m| m as f64).sum::<f64>()
                    / slice.len() as f64
                    / iq::MAG_SCALE as f64;
                win_noise_n += 1;
            }
        }
        if !mag.is_empty() {
            let n = (blk.bytes.len() / 2).min(mag.len());
            let sum: u64 = mag[mag.len() - n..].iter().map(|&x| x as u64).sum();
            st.noise(sum as f32 / n as f32 / iq::MAG_SCALE);
        }
        let mut chunk = Vec::new();
        let mut reduced = Vec::new();
        let now = wall();
        let now_ms = (now * 1000.0) as u64;
        // Frames from Beast input: MLAT results go to the table as MLAT
        // positions; everything received is forwarded on the full stream
        // (as readsb does), never on the reduced one.
        for f in in_rx.try_iter() {
            if f.is_mlat() {
                tracker.offer_mlat(&f.bytes, now);
            }
            beast::encode(&f.bytes, f.ts, f.signal, &mut chunk);
        }
        for f in d.run_iq(&mut mag, &mut raw, base, &remag) {
            let level = f.signal / iq::MAG_SCALE;
            if let track::Verdict::Reject = tracker.offer(&f.bytes, level, f.fixed, now) {
                continue;
            }
            frames_1s += 1;
            frames_total += 1;
            st.frame(level, f.fixed);
            if f.bytes.len() == 14 && matches!(f.bytes[0] >> 3, 17 | 18) {
                aircraft.insert(
                    (f.bytes[1] as u32) << 16 | (f.bytes[2] as u32) << 8 | f.bytes[3] as u32,
                );
            }
            // 12 MHz Beast ticks: five per sample. Signal byte: pulse level
            // in raw magnitude units (0..181) stretched to 0..255.
            let ticks = (f.start * 5.0).round() as u64;
            let signal = ((f.signal / iq::MAG_SCALE) * 1.4).clamp(0.0, 255.0) as u8;
            if signal >= 250 {
                win_strong = true;
            }
            beast::encode(&f.bytes, ticks, signal, &mut chunk);
            if reducer.forward(&f.bytes, now_ms) {
                beast::encode(&f.bytes, ticks, signal, &mut reduced);
            }
        }
        if !chunk.is_empty() {
            hub.publish(beast::Stream::Full, chunk);
        }
        if !reduced.is_empty() {
            hub.publish(beast::Stream::Reduced, reduced);
        }
        let keep = mag.len().saturating_sub(TAIL);
        base += keep as u64;
        mag.drain(..keep);
        raw.drain(..2 * keep);
        if last_report.elapsed() >= Duration::from_secs(1) {
            let now = wall();
            tracker.expire(now, 60.0);
            if let Some(dir) = &json_dir {
                if let Err(e) = tracker.write_json(dir, now) {
                    eprintln!("rx: aircraft.json: {e}");
                }
            }
            st.tick(now, json_dir.as_deref());
            report_ticks += 1;
            if a.verbose || report_ticks.is_multiple_of(60) {
                eprintln!(
                "rx: {frames_1s} frames/s, {frames_total} total, {} aircraft ({} tracked), {} cancellations, {} rescued, {} rejected repairs, {} consumers",
                aircraft.len(),
                tracker.aircraft.len(),
                d.stats.cancelled,
                d.stats.rescan_frames,
                tracker.rejected,
                hub.consumers()
            );
            }
            frames_1s = 0;
            last_report = Instant::now();
        }
        if let Some(g) = gainer.as_mut() {
            if win_start.elapsed().as_secs_f64() >= gain::WINDOW_S && win_samples > 0 {
                let w = gain::Window {
                    clip_fraction: win_clipped as f64 / (2 * win_samples) as f64,
                    strong_clip: win_strong,
                    noise: if win_noise_n > 0 {
                        (win_noise / win_noise_n as f64) as f32
                    } else {
                        0.0
                    },
                };
                let before = g.current();
                if let Some(t) = g.step(&w, wall()) {
                    eprintln!(
                        "rx: gain {:.1} -> {:.1} dB (clipping {:.2} %, noise {:.2})",
                        before as f32 / 10.0,
                        t as f32 / 10.0,
                        100.0 * w.clip_fraction,
                        w.noise
                    );
                    let _ = gain_tx.send(t);
                }
                win_samples = 0;
                win_clipped = 0;
                win_strong = false;
                win_noise = 0.0;
                win_noise_n = 0;
                win_start = Instant::now();
            }
        }
        if a.seconds > 0 && started.elapsed() >= Duration::from_secs(a.seconds) {
            break;
        }
    }
    drop(rx);
    let _ = reader.join();
    Ok(())
}
