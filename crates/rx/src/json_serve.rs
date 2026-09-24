//! aircraft.json and stats.json over a local socket, so nothing is
//! written to disk for the station to read back. Plain HTTP/1.0, one
//! document per request, no framework: `GET /aircraft.json` and
//! `GET /stats.json`, refreshed by the radio's once-a-second tick.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default)]
pub struct Docs {
    pub aircraft: Mutex<String>,
    pub stats: Mutex<String>,
}

impl Docs {
    /// The response body for a request line, or None for anything else.
    fn body(&self, request_line: &str) -> Option<String> {
        let mut parts = request_line.split_whitespace();
        if parts.next()? != "GET" {
            return None;
        }
        let path = parts.next()?;
        let path = path.split('?').next().unwrap_or(path);
        match path {
            "/aircraft.json" | "/data/aircraft.json" => Some(self.aircraft.lock().unwrap().clone()),
            "/stats.json" | "/data/stats.json" => Some(self.stats.lock().unwrap().clone()),
            _ => None,
        }
    }
}

/// The response for a request line, headers and body.
fn respond(docs: &Docs, request_line: &str) -> String {
    match docs.body(request_line) {
        Some(body) if !body.is_empty() => format!(
            "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\
             Cache-Control: no-store\r\nConnection: close\r\n\r\n{body}",
            body.len()
        ),
        Some(_) => {
            "HTTP/1.0 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                .into()
        }
        None => "HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n".into(),
    }
}

fn handle(mut s: TcpStream, docs: &Docs) {
    let _ = s.set_read_timeout(Some(Duration::from_secs(2)));
    let _ = s.set_write_timeout(Some(Duration::from_secs(2)));
    let mut buf = [0u8; 1024];
    let n = s.read(&mut buf).unwrap_or(0);
    let head = String::from_utf8_lossy(&buf[..n]);
    let line = head.lines().next().unwrap_or("");
    let _ = s.write_all(respond(docs, line).as_bytes());
}

/// Serve `docs` on `listen` from a thread; returns once bound.
pub fn serve(listen: &str, docs: Arc<Docs>) -> std::io::Result<()> {
    let listener = TcpListener::bind(listen)?;
    std::thread::spawn(move || {
        for conn in listener.incoming().flatten() {
            let docs = docs.clone();
            std::thread::spawn(move || handle(conn, &docs));
        }
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn docs() -> Docs {
        let d = Docs::default();
        *d.aircraft.lock().unwrap() = "{\"aircraft\":[]}".into();
        *d.stats.lock().unwrap() = "{\"total\":{}}".into();
        d
    }

    #[test]
    fn the_two_documents_by_either_name() {
        let d = docs();
        for p in [
            "/aircraft.json",
            "/data/aircraft.json",
            "/aircraft.json?_=1",
        ] {
            assert_eq!(
                d.body(&format!("GET {p} HTTP/1.1")).unwrap(),
                "{\"aircraft\":[]}"
            );
        }
        assert_eq!(
            d.body("GET /stats.json HTTP/1.0").unwrap(),
            "{\"total\":{}}"
        );
        assert!(d.body("GET /index.html HTTP/1.0").is_none());
        assert!(d.body("POST /aircraft.json HTTP/1.0").is_none());
        assert!(d.body("").is_none());
    }

    #[test]
    fn responses_carry_a_length_and_close() {
        let d = docs();
        let r = respond(&d, "GET /aircraft.json HTTP/1.1");
        assert!(r.starts_with("HTTP/1.0 200 OK\r\n"));
        assert!(r.contains("Content-Length: 15\r\n"));
        assert!(r.ends_with("\r\n\r\n{\"aircraft\":[]}"));
        assert!(respond(&d, "GET /nope HTTP/1.1").starts_with("HTTP/1.0 404"));
        // Before the first tick there is nothing to serve yet, and that
        // is not a 200 with an empty body.
        let empty = Docs::default();
        assert!(respond(&empty, "GET /aircraft.json HTTP/1.1").starts_with("HTTP/1.0 503"));
    }

    #[test]
    fn served_over_a_real_socket() {
        let d = Arc::new(docs());
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap();
        drop(l);
        serve(&addr.to_string(), d).unwrap();
        let mut s = TcpStream::connect(addr).unwrap();
        s.write_all(b"GET /stats.json HTTP/1.0\r\nHost: x\r\n\r\n")
            .unwrap();
        let mut out = String::new();
        s.read_to_string(&mut out).unwrap();
        assert!(out.ends_with("{\"total\":{}}"), "{out}");
    }
}
