//! rx: the Station radio. Milestone R0: drive the dongle, keep an honest
//! sample clock, run the decoder live, report rates.
//!
//! One thread reads the dongle into fixed blocks; the main thread converts
//! and decodes. The sample counter is the clock every frame timestamp
//! derives from (12 MHz Beast ticks = 5 per sample), so lost samples must
//! advance it: the reader compares samples delivered with wall time and
//! accounts a gap when the dongle falls behind by more than a block.

mod beast;
mod sdr;

use anyhow::Result;
use clap::Parser;
use std::io::Write;
use std::sync::mpsc;
use std::time::{Duration, Instant};

const SAMPLE_RATE: u32 = 2_400_000;
const FREQ: u32 = 1_090_000_000;
/// Read block: 1/8 s of samples, a multiple of 16 KB as librtlsdr wants.
const BLOCK_BYTES: usize = 655_360;

#[derive(Parser)]
#[command(about = "The Station radio: RTL-SDR in, Mode S frames out")]
struct Args {
    /// Device index.
    #[arg(long, default_value_t = 0)]
    device: u32,
    /// Device serial (overrides --device).
    #[arg(long)]
    serial: Option<String>,
    /// Tuner gain in dB, or "agc" for the hardware AGC.
    #[arg(long, default_value = "49.6")]
    gain: String,
    /// Power the antenna port (bias tee). Only for a powered LNA.
    #[arg(long, default_value_t = false)]
    bias_tee: bool,
    /// Write raw I/Q to this file (UC8) as well.
    #[arg(long)]
    record: Option<String>,
    /// Stop after this many seconds (0 = run until killed).
    #[arg(long, default_value_t = 0)]
    seconds: u64,
    /// List devices and exit.
    #[arg(long, default_value_t = false)]
    list: bool,
    /// Beast server address ("" = none).
    #[arg(long, default_value = "0.0.0.0:30005")]
    beast_listen: String,
    /// Push the Beast stream to host:port (repeatable).
    #[arg(long)]
    beast_connect: Vec<String>,
}

/// A block of samples with the stream index of its first sample.
struct Block {
    first_sample: u64,
    bytes: Vec<u8>,
}

fn main() -> Result<()> {
    let a = Args::parse();
    if a.list {
        for d in sdr::list() {
            println!("{}: {} serial {}", d.index, d.name, d.serial);
        }
        return Ok(());
    }
    let mut dev = sdr::Device::open(a.device, a.serial.as_deref())?;
    dev.set_sample_rate(SAMPLE_RATE)?;
    dev.set_center_freq(FREQ)?;
    if a.gain == "agc" {
        dev.set_agc()?;
        eprintln!("rx: {} at {} MS/s, hardware AGC", dev.name, SAMPLE_RATE as f64 / 1e6);
    } else {
        let want = (a.gain.parse::<f32>()? * 10.0).round() as i32;
        let got = dev.set_gain(want)?;
        eprintln!("rx: {} at {} MS/s, gain {:.1} dB", dev.name, SAMPLE_RATE as f64 / 1e6, got as f32 / 10.0);
    }
    dev.set_bias_tee(a.bias_tee)?;
    dev.reset_buffer()?;

    // Reader thread: blocks to the decoder, sample clock kept honest.
    let (tx, rx) = mpsc::sync_channel::<Block>(16);
    let reader = std::thread::spawn(move || -> Result<()> {
        let t0 = Instant::now();
        let mut delivered: u64 = 0; // samples delivered by the dongle
        let mut clock: u64 = 0; // stream index including accounted gaps
        let mut gaps: u64 = 0;
        loop {
            let mut buf = vec![0u8; BLOCK_BYTES];
            let n = dev.read(&mut buf)?;
            buf.truncate(n & !1);
            let samples = (buf.len() / 2) as u64;
            // Expected samples by wall time; a shortfall beyond one block is
            // a gap the dongle dropped (USB stall). Advance the clock so the
            // frames after it keep true timestamps.
            let expected = (t0.elapsed().as_secs_f64() * SAMPLE_RATE as f64) as u64;
            let block = (BLOCK_BYTES / 2) as u64;
            if expected > delivered + samples + 2 * block {
                let gap = expected - delivered - samples;
                clock += gap;
                delivered += gap;
                gaps += 1;
                eprintln!("rx: sample gap of {:.1} ms accounted (gap {})", gap as f64 / SAMPLE_RATE as f64 * 1e3, gaps);
            }
            let first = clock;
            clock += samples;
            delivered += samples;
            if tx.send(Block { first_sample: first, bytes: buf }).is_err() {
                return Ok(());
            }
        }
    });

    let hub = beast::Hub::new();
    if !a.beast_listen.is_empty() {
        hub.serve(&a.beast_listen)?;
        eprintln!("rx: Beast server on {}", a.beast_listen);
    }
    for c in &a.beast_connect {
        hub.connect(c.clone());
    }
    let mut rec = match &a.record {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p)?)),
        None => None,
    };
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
    let (mut frames_1s, mut frames_total, mut aircraft): (u64, u64, std::collections::HashSet<u32>) = (0, 0, Default::default());
    let mut expect_first: u64 = 0;
    for blk in rx.iter() {
        if let Some(w) = rec.as_mut() {
            w.write_all(&blk.bytes)?;
        }
        // A gap in the stream: the carried tail no longer touches the new
        // block, so start fresh at the new position.
        if blk.first_sample != expect_first {
            mag.clear();
            raw.clear();
            base = blk.first_sample;
        }
        expect_first = blk.first_sample + (blk.bytes.len() / 2) as u64;
        raw.extend_from_slice(&blk.bytes);
        if cfg!(target_arch = "aarch64") {
            iq::magnitudes_f32(&blk.bytes, &mut mag);
        } else {
            lut.magnitudes(&blk.bytes, &mut mag);
        }
        let mut chunk = Vec::new();
        for f in d.run_iq(&mut mag, &mut raw, base, &remag) {
            frames_1s += 1;
            frames_total += 1;
            if f.bytes.len() == 14 && matches!(f.bytes[0] >> 3, 17 | 18) {
                aircraft.insert((f.bytes[1] as u32) << 16 | (f.bytes[2] as u32) << 8 | f.bytes[3] as u32);
            }
            // 12 MHz Beast ticks: five per sample. Signal byte: pulse level
            // in raw magnitude units (0..181) stretched to 0..255.
            let ticks = (f.start * 5.0).round() as u64;
            let signal = ((f.signal / iq::MAG_SCALE) * 1.4).clamp(0.0, 255.0) as u8;
            beast::encode(&f.bytes, ticks, signal, &mut chunk);
        }
        if !chunk.is_empty() {
            hub.publish(chunk);
        }
        let keep = mag.len().saturating_sub(TAIL);
        base += keep as u64;
        mag.drain(..keep);
        raw.drain(..2 * keep);
        if last_report.elapsed() >= Duration::from_secs(1) {
            eprintln!(
                "rx: {frames_1s} frames/s, {frames_total} total, {} aircraft, {} cancellations, {} rescued, {} consumers",
                aircraft.len(),
                d.stats.cancelled,
                d.stats.rescan_frames,
                hub.consumers()
            );
            frames_1s = 0;
            last_report = Instant::now();
        }
        if a.seconds > 0 && started.elapsed() >= Duration::from_secs(a.seconds) {
            break;
        }
    }
    drop(rx);
    let _ = reader.join();
    Ok(())
}
