//! Beast input: frames arriving over the network, as readsb's
//! `--net-bi-port` and `beast_in` connectors accept them. The station's
//! mlatc sends its MLAT results here (DF18 pairs carrying the "magic"
//! MLAT timestamp 0xFF004D4C4154). A listener and a connector both feed
//! one channel; the main loop drains it once per block.
//!
//! Framing: 0x1a, type (0x31 Mode A/C, 0x32 short, 0x33 long, 0x34
//! config), 6-byte big-endian timestamp, signal byte, payload; every
//! 0x1a inside the body is doubled. Anything else after a 0x1a (the 0xE4
//! uuid message, 'W' markers, unknown types) is skipped by resyncing on
//! the next 0x1a. Mode A/C messages (which is what heartbeats are) are
//! consumed and dropped.

use std::io::Read;
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::sync::mpsc::Sender;
use std::time::Duration;

/// The timestamp mlat-client and mlatc put on result frames.
pub const MLAT_MAGIC: u64 = 0xFF00_4D4C_4154;

/// One frame received on Beast input.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InFrame {
    pub ts: u64,
    pub signal: u8,
    pub bytes: Vec<u8>,
}

impl InFrame {
    pub fn is_mlat(&self) -> bool {
        self.ts == MLAT_MAGIC
    }
}

/// Streaming parser: feed bytes, collect frames. Bytes of an incomplete
/// message are kept until the next feed.
#[derive(Default)]
pub struct Parser {
    buf: Vec<u8>,
}

impl Parser {
    pub fn new() -> Self {
        Parser::default()
    }

    pub fn feed(&mut self, data: &[u8], out: &mut Vec<InFrame>) {
        self.buf.extend_from_slice(data);
        let mut pos = 0;
        while let Some((used, frame)) = parse_one(&self.buf[pos..]) {
            pos += used;
            if let Some(f) = frame {
                out.push(f);
            }
        }
        self.buf.drain(..pos);
    }
}

/// One message from the front of `b`: (bytes consumed, frame). None when
/// the buffer holds an incomplete message.
fn parse_one(b: &[u8]) -> Option<(usize, Option<InFrame>)> {
    let start = b.iter().position(|&x| x == 0x1a)?;
    if start > 0 {
        return Some((start, None)); // garbage before the escape
    }
    let t = *b.get(1)?;
    let (payload_len, keep) = match t {
        0x31 => (2, false),
        0x32 => (7, true),
        0x33 => (14, true),
        0x34 => (7, false),
        // 0xE4 uuid, 'W' markers, anything unknown: skip the escape and
        // resync on the next one.
        _ => return Some((1, None)),
    };
    let mut body = Vec::with_capacity(7 + payload_len);
    let mut i = 2;
    while body.len() < 7 + payload_len {
        let x = *b.get(i)?;
        if x == 0x1a {
            match b.get(i + 1) {
                None => return None, // need one more byte to decide
                Some(0x1a) => i += 1,
                Some(_) => return Some((i, None)), // truncated message; resync here
            }
        }
        body.push(x);
        i += 1;
    }
    if !keep {
        return Some((i, None));
    }
    let ts = body[..6].iter().fold(0u64, |a, &x| a << 8 | x as u64);
    Some((
        i,
        Some(InFrame {
            ts,
            signal: body[6],
            bytes: body[7..].to_vec(),
        }),
    ))
}

fn pump(mut s: TcpStream, tx: &Sender<InFrame>) {
    let _ = s.set_read_timeout(Some(Duration::from_secs(60)));
    let mut parser = Parser::new();
    let mut chunk = [0u8; 65536];
    let mut frames = Vec::new();
    loop {
        let n = match s.read(&mut chunk) {
            Ok(0) => return,
            Ok(n) => n,
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue
            }
            Err(_) => return,
        };
        frames.clear();
        parser.feed(&chunk[..n], &mut frames);
        for f in frames.drain(..) {
            if tx.send(f).is_err() {
                return;
            }
        }
    }
}

/// Accept Beast senders on `listen`; every frame goes to `tx`.
pub fn listen(listen: &str, tx: Sender<InFrame>) -> std::io::Result<()> {
    let listener = TcpListener::bind(listen)?;
    std::thread::spawn(move || {
        for conn in listener.incoming() {
            let Ok(s) = conn else { continue };
            let peer = s.peer_addr().map(|p| p.to_string()).unwrap_or_default();
            let tx = tx.clone();
            std::thread::spawn(move || {
                pump(s, &tx);
                eprintln!("beast in: {peer} gone");
            });
        }
    });
    Ok(())
}

/// Pull a Beast stream from `addr`, reconnecting with backoff up to 60 s.
pub fn connect(addr: String, tx: Sender<InFrame>) {
    std::thread::spawn(move || {
        let mut backoff = 1u64;
        loop {
            match addr
                .to_socket_addrs()
                .ok()
                .and_then(|mut a| a.next())
                .and_then(|a| TcpStream::connect_timeout(&a, Duration::from_secs(10)).ok())
            {
                Some(s) => {
                    eprintln!("beast in: connected to {addr}");
                    backoff = 1;
                    pump(s, &tx);
                    eprintln!("beast in: {addr} dropped, reconnecting");
                }
                None => eprintln!("beast in: {addr} unreachable, retry in {backoff} s"),
            }
            std::thread::sleep(Duration::from_secs(backoff));
            backoff = (backoff * 2).min(60);
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::beast;

    #[test]
    fn round_trip_with_escapes_and_heartbeats() {
        let short = [0x1au8, 2, 3, 4, 5, 6, 7];
        let long = [
            0x8Du8, 0x1a, 0x62, 0x1a, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC, 0x28, 0x63, 0x1a,
        ];
        let mut wire = Vec::new();
        wire.extend_from_slice(beast::HEARTBEAT);
        beast::encode(&short, 0x00001a00001a, 0x1a, &mut wire);
        // a uuid message and a marker, as readsb sends them
        wire.extend_from_slice(&[0x1a, 0xE4]);
        wire.extend_from_slice(&[b'f'; 36]);
        wire.extend_from_slice(&[0x1a, b'W', b'O']);
        beast::encode(&long, MLAT_MAGIC, 200, &mut wire);
        wire.extend_from_slice(beast::HEARTBEAT);

        // feed in awkward pieces
        let mut p = Parser::new();
        let mut out = Vec::new();
        for piece in wire.chunks(5) {
            p.feed(piece, &mut out);
        }
        assert_eq!(out.len(), 2, "{out:?}");
        assert_eq!(
            out[0],
            InFrame {
                ts: 0x00001a00001a,
                signal: 0x1a,
                bytes: short.to_vec()
            }
        );
        assert_eq!(
            out[1],
            InFrame {
                ts: MLAT_MAGIC,
                signal: 200,
                bytes: long.to_vec()
            }
        );
        assert!(out[1].is_mlat());
        assert!(!out[0].is_mlat());
    }

    #[test]
    fn resyncs_after_garbage_and_truncation() {
        let frame = [0x8Du8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13];
        let mut good = Vec::new();
        beast::encode(&frame, 12345, 7, &mut good);
        let mut wire = vec![1, 2, 3]; // garbage
        wire.extend_from_slice(&good[..10]); // a truncated message
        wire.extend_from_slice(&good); // then a whole one
        let mut p = Parser::new();
        let mut out = Vec::new();
        p.feed(&wire, &mut out);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].bytes, frame.to_vec());
        assert_eq!(out[0].ts, 12345);
    }
}
