# rx

rx is the Station radio: it drives an RTL-SDR, demodulates Mode S with
the decoder benchmarked in demod-bench, keeps an aircraft table, and
speaks Beast on the network. It does what the station used readsb for.
It is one binary, one process, supervised by stationd.

Status: private, milestone R2 of the plan below. Runs on the Pi 3B.

## Crates

- `iq`: 8-bit I/Q reading and magnitude conversion.
- `demod`: the decoder. Preamble detection on a fifth-of-a-sample grid,
  integer bit slicing, soft-decision CRC repair, address-parity repair,
  timing retries, and collision recovery by subtracting a decoded frame
  from the I/Q and rescanning. demod-bench depends on this crate, so the
  bench always scores what the radio ships.
- `rx`: the radio. `sdr` binds librtlsdr (LGPL, dynamically linked);
  `beast` encodes frames and runs the server and connectors; `reduce`
  is the rate-limited stream the aggregators ingest; `track` is the
  aircraft table and aircraft.json.

## Run

```sh
rx --gain 49.6 --net-bo-port 30005 --write-json /var/run/rx --lat 1.29849 --lon 103.85728 \\
   --net-connector feed.flightportrait.com,30004,beast_reduce_plus_out,uuid=<station-uuid> \\
   --net-connector in.adsb.lol,30004,beast_reduce_plus_out,uuid=<uuid>
```

rx accepts readsb's flags for these purposes (`--device-type rtlsdr`,
`--gain auto`, `--quiet`, `--net`, `--net-bo-port`, `--write-json`,
`--write-json-every`, `--lat`, `--lon`, `--net-connector`), so a station
configured for readsb runs rx by changing the binary path.

## Behaviour

- The sample counter is the clock. Every frame carries a 12 MHz Beast
  timestamp derived from it; when the dongle falls behind wall time by
  more than two read blocks, the gap is accounted so later timestamps
  stay true. MLAT depends on this.
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

## Plan

- R0 dongle in, honest clock: done 2026-09-05 (30-minute soak on the Pi
  3B, no gaps).
- R1 live decoder with Beast timestamps: done; readsb in network-only
  mode builds a correct aircraft table from rx's stream.
- R2 Beast server, connectors, reduced stream, UUID hello: done; the
  hub and an aggregator accepting the station as a normal feeder is the
  acceptance test still to run.
- R3 aircraft table and the consistency check: done in code; the zero
  inconsistent repaired positions live is the acceptance test.
- R4 aircraft.json for stationd: done.
- R5 shadow and swap: rx on the Pi with periodic raw recordings replayed
  through readsb nightly; two weeks ahead with no false frames, then rx
  becomes the installer's default with readsb as fallback.

Not in scope: Mode A/C, 978 MHz, other SDRs, the web map, history,
graphs, the aircraft database.

## License

AGPL-3.0-or-later. librtlsdr is LGPL and linked dynamically.
