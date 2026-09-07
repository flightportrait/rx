# rx

rx is the Station radio: it drives an RTL-SDR, demodulates Mode S,
keeps an aircraft table, and speaks Beast on the network. It does what
a station used readsb for, in one binary and one process, supervised
by [stationd](https://github.com/flightportrait/station). Its decoder
is scored against readsb on replayed captures before every change
ships; on a Pi 3B the whole radio takes about a fifth of one core.

Status: v0.1, running on our own station since 2026-09-05 with readsb
as the fallback radio. The milestones and their acceptance tests are in
[docs/PLAN.md](docs/PLAN.md).

## Build

```sh
sudo apt-get install librtlsdr-dev    # rx links librtlsdr dynamically
cargo build --release
```

Release binaries for x86_64 and aarch64 glibc are on the
[releases page](https://github.com/flightportrait/rx/releases); they
need `librtlsdr0` from the distribution.

## Crates

- `iq`: 8-bit I/Q reading and magnitude conversion.
- `demod`: the decoder. Preamble detection on a fifth-of-a-sample grid,
  integer bit slicing, soft-decision CRC repair, address-parity repair,
  timing retries, and collision recovery by subtracting a decoded frame
  from the I/Q and rescanning. Our replay bench depends on this crate,
  so the bench always scores what the radio ships.
- `rx`: the radio. `sdr` binds librtlsdr (LGPL, dynamically linked);
  `beast` encodes frames and runs the server and connectors; `reduce`
  is the rate-limited stream the aggregators ingest; `track` is the
  aircraft table and aircraft.json.

## Run

```sh
rx --gain 49.6 --net-bo-port 30005 --write-json /var/run/rx --lat 1.29849 --lon 103.85728 \
   --net-connector feed.flightportrait.com,30004,beast_reduce_plus_out,uuid=<station-uuid> \
   --net-connector in.adsb.lol,30004,beast_reduce_plus_out,uuid=<uuid>
```

rx accepts readsb's flags for these purposes (`--device-type rtlsdr`,
`--gain auto`, `--quiet`, `--net`, `--net-bo-port`, `--write-json`,
`--write-json-every`, `--lat`, `--lon`, `--net-connector`), so a station
configured for readsb runs rx by changing the binary path.

## Behaviour

- Samples are read with librtlsdr's asynchronous API, fifteen 256 KiB
  transfers queued, as readsb does. A synchronous read leaves the USB
  bus idle while the host handles each block and the RTL2832U silently
  drops samples in that gap: 1.5 % of the stream on a Pi 3B, which
  looked like a clock drift and was enough to keep MLAT servers from
  ever pairing the station (found and fixed 2026-09-05; the delivered
  rate is now within 250 ppm of nominal).
- The clock is the count of delivered samples, as in readsb, and every
  frame carries a 12 MHz Beast timestamp derived from it. It is not
  corrected from wall time. A real USB loss is a discontinuity MLAT
  servers detect as a clock reset; short reads are counted and reported
  once a minute.
- Gain: `--gain <dB>` fixed, `--gain agc` the hardware AGC, `--gain auto`
  rx's own loop: every 10 s it looks at clipped samples and the noise
  floor and moves the tuner one step, only when two windows agree, then
  holds 20 s.
- A dongle that disappears, or is absent at boot, is reopened every 2 s
  (backoff to 10 s) with its settings restored; the outage advances the
  clock by wall time, which MLAT sees as one reset.
- Beast input (`--net-bi-port`, `beast_in` connectors): MLAT results
  from mlatc reach the aircraft table as MLAT positions (never
  overriding an ADS-B position younger than 30 s) and are forwarded on
  the full stream, never on the reduced one.
- `stats.json` beside aircraft.json every 10 s, readsb's shape for the
  fields rx has.
- Every emitted frame carries its repaired-bit count internally; a
  repaired position message that contradicts the aircraft's track (more
  than 400 m/s of travel plus 2 km from its last position) is rejected
  before it is published.
- Connectors reconnect with backoff up to 60 s. A `beast_reduce_plus_out`
  connector opens with the station UUID (0x1a 0xE4 and 36 characters),
  a 0x1a "WO" marker and five heartbeats, as readsb does; a heartbeat is
  sent after 5 s of silence.
- The reduced stream forwards, per aircraft, positions always, altitude,
  heading and speed at 7/8 of the interval, squawk and emergency at 4x,
  all-call replies at 4x. Interval 500 ms by default.
- Every accepted frame is subtracted from the I/Q and its span rescanned
  for a frame underneath. On a real sky that is a few dozen fits a
  second.

## In the station

stationd runs rx in its `[programs].radio` slot with readsb as
`[programs].readsb` fallback: three radio exits within ten minutes, or
fifteen minutes of silence, hand the dongle to readsb with one sentence
on the status page; crash fallbacks retry rx after an hour. Drilled
2026-09-05 on the Pi: three kills, readsb up in seconds, rx back on the
next restart.

Not in scope: Mode A/C, 978 MHz, other SDRs, the web map, history,
graphs, the aircraft database.

## License

AGPL-3.0-or-later ([LICENSE-AGPL](LICENSE-AGPL)) for `rx` and `demod`;
`iq` is MIT ([LICENSE-MIT](LICENSE-MIT)). librtlsdr is LGPL and linked
dynamically.
