//! Beast binary output: the format every consumer of a Mode S receiver
//! speaks (mlat-client, readsb, the aggregators' ingest).
//!
//! Message: 0x1a, type (0x32 short Mode S, 0x33 long), 6-byte big-endian
//! timestamp in 12 MHz ticks, one signal byte, the frame. Every 0x1a in
//! the body is doubled. A server hands the stream to any client that
//! connects; connectors push it to a remote host and reconnect with
//! backoff when the link drops. Slow consumers are dropped rather than
//! allowed to stall the radio.

use std::io::Write;
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Encode one frame. `ticks` is the 12 MHz timestamp, `signal` 0..255.
pub fn encode(frame: &[u8], ticks: u64, signal: u8, out: &mut Vec<u8>) {
    out.push(0x1a);
    out.push(if frame.len() == 7 { 0x32 } else { 0x33 });
    let mut body = [0u8; 21];
    body[..6].copy_from_slice(&ticks.to_be_bytes()[2..8]);
    body[6] = signal;
    body[7..7 + frame.len()].copy_from_slice(frame);
    for &b in &body[..7 + frame.len()] {
        out.push(b);
        if b == 0x1a {
            out.push(0x1a);
        }
    }
}

/// Which stream a consumer takes.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Stream {
    Full,
    Reduced,
}

/// The bytes readsb sends when a `beast_reduce_plus_out` link opens: the
/// station UUID (0x1a 0xE4, 36 characters padded with 'f'), a 0x1a "WO"
/// marker, then five heartbeats.
pub fn hello(uuid: Option<&str>) -> Vec<u8> {
    let mut v = Vec::new();
    if let Some(u) = uuid {
        let mut padded = [b'f'; 36];
        for (o, b) in padded.iter_mut().zip(u.trim().bytes()) {
            *o = b;
        }
        v.extend_from_slice(&[0x1a, 0xE4]);
        v.extend_from_slice(&padded);
        v.extend_from_slice(&[0x1a, b'W', b'O']);
    }
    for _ in 0..5 {
        v.extend_from_slice(HEARTBEAT);
    }
    v
}

/// readsb's Beast heartbeat: a Mode A/C message of zeros.
pub const HEARTBEAT: &[u8] = &[0x1a, 0x31, 0, 0, 0, 0, 0, 0, 0, 0, 0];

struct Sink {
    stream: Stream,
    tx: SyncSender<Arc<Vec<u8>>>,
}

/// Fan-out of encoded bytes to every live sink, full or reduced stream.
#[derive(Clone)]
pub struct Hub {
    sinks: Arc<Mutex<Vec<Sink>>>,
}

impl Hub {
    pub fn new() -> Self {
        Hub {
            sinks: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Deliver a chunk to every sink of `stream`; a sink whose queue is
    /// full is dropped.
    pub fn publish(&self, stream: Stream, chunk: Vec<u8>) {
        let chunk = Arc::new(chunk);
        let mut sinks = self.sinks.lock().unwrap();
        sinks.retain(|s| {
            if s.stream != stream {
                return true;
            }
            match s.tx.try_send(chunk.clone()) {
                Ok(()) => true,
                Err(TrySendError::Full(_)) => {
                    eprintln!("beast: dropping a consumer that stopped reading");
                    false
                }
                Err(TrySendError::Disconnected(_)) => false,
            }
        });
    }

    fn subscribe(&self, stream: Stream) -> Receiver<Arc<Vec<u8>>> {
        let (tx, rx) = sync_channel(256);
        self.sinks.lock().unwrap().push(Sink { stream, tx });
        rx
    }

    pub fn consumers(&self) -> usize {
        self.sinks.lock().unwrap().len()
    }

    /// Accept clients on `listen` and stream to each until it goes away.
    pub fn serve(&self, listen: &str) -> std::io::Result<()> {
        let listener = TcpListener::bind(listen)?;
        let hub = self.clone();
        std::thread::spawn(move || {
            for conn in listener.incoming() {
                let Ok(mut stream) = conn else { continue };
                let peer = stream.peer_addr().map(|p| p.to_string()).unwrap_or_default();
                let rx = hub.subscribe(Stream::Full);
                std::thread::spawn(move || {
                    let _ = stream.set_nodelay(true);
                    if stream.write_all(&hello(None)).is_err() {
                        return;
                    }
                    if pump(&mut stream, &rx).is_err() {}
                    eprintln!("beast: client {peer} gone");
                });
            }
        });
        Ok(())
    }

    /// Push a stream to `addr`, reconnecting with backoff up to 60 s. The
    /// UUID, when given, identifies the station to the aggregator.
    pub fn connect(&self, addr: String, stream: Stream, uuid: Option<String>) {
        let hub = self.clone();
        std::thread::spawn(move || {
            let mut backoff = 1u64;
            loop {
                match addr.to_socket_addrs().ok().and_then(|mut a| a.next()).and_then(|a| TcpStream::connect_timeout(&a, Duration::from_secs(10)).ok()) {
                    Some(mut s) => {
                        eprintln!("beast: connected to {addr}{}", if uuid.is_some() { " (sent UUID)" } else { "" });
                        backoff = 1;
                        let _ = s.set_nodelay(true);
                        let rx = hub.subscribe(stream);
                        if s.write_all(&hello(uuid.as_deref())).is_ok() {
                            let _ = pump(&mut s, &rx);
                        }
                        eprintln!("beast: {addr} dropped, reconnecting");
                    }
                    None => {
                        eprintln!("beast: {addr} unreachable, retry in {backoff} s");
                    }
                }
                std::thread::sleep(Duration::from_secs(backoff));
                backoff = (backoff * 2).min(60);
            }
        });
    }
}

/// Write chunks to a socket; a heartbeat every 5 s of silence keeps the
/// link alive and lets the far end notice when it dies.
fn pump(s: &mut TcpStream, rx: &Receiver<Arc<Vec<u8>>>) -> std::io::Result<()> {
    loop {
        match rx.recv_timeout(Duration::from_secs(5)) {
            Ok(chunk) => s.write_all(&chunk)?,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => s.write_all(HEARTBEAT)?,
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_and_frames() {
        let mut out = Vec::new();
        // timestamp containing 0x1a bytes, signal 0x1a, frame containing 0x1a
        encode(&[0x1a, 2, 3, 4, 5, 6, 7], 0x00001a00001a, 0x1a, &mut out);
        assert_eq!(out[0], 0x1a);
        assert_eq!(out[1], 0x32);
        // body: ts 00 00 1a 1a 00 00 1a 1a ... each 0x1a doubled
        let body = &out[2..];
        let mut i = 0;
        let mut decoded = Vec::new();
        while i < body.len() {
            decoded.push(body[i]);
            if body[i] == 0x1a {
                assert_eq!(body[i + 1], 0x1a);
                i += 1;
            }
            i += 1;
        }
        assert_eq!(decoded.len(), 14);
        assert_eq!(&decoded[7..], &[0x1a, 2, 3, 4, 5, 6, 7]);
        assert_eq!(decoded[6], 0x1a);
    }
}
