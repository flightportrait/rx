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
  timestamp derived from it. The stream runs a steady latency behind
  wall time; only a lag that grows beyond the smallest lag seen recently
  and stays grown across eight reads is accounted as a loss, by
  advancing the clock. MLAT depends on this.
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
- R1 live decoder with Beast timestamps: done. readsb in network-only
  mode builds a correct aircraft table from rx's stream, and rx live
  matches the bench replaying the same ten minutes to within one frame
  (3700 live, 3699 offline; readsb offline 1931 CRC-valid frames to
  rx's 1990, address-parity 1542 to 1709, every repaired position
  consistent with its track).
- R2 Beast server, connectors, reduced stream, UUID hello: done and
  accepted 2026-09-05. rx on the Pi fed feed.flightportrait.com over a
  `beast_reduce_plus_out` connector with a fresh UUID; the hub took the
  connection, and the public API listed the station online under the
  id derived from that UUID (fp-1a9a5a0e4e). rx at 17 % of a Pi 3B core
  and 6 MB resident with every frame cancelled.
- R3 aircraft table and the consistency check: done in code; the zero
  inconsistent repaired positions live is the acceptance test.
- R4 aircraft.json for stationd: done.
- R5 shadow and swap: started 2026-09-05. The Pi station's config
  points its radio at rx (the readsb line kept in station.toml.readsb
  for a one-line revert); stationd, mlatc and the status page run
  unchanged on it. `--clip-dir` writes ten seconds of raw I/Q every ten
  minutes; leserveur pulls the clips nightly (`~/shadow/shadow.sh`,
  cron 03:17 UTC) and appends one line per clip to `~/shadow/shadow.log`
  with CRC-valid and address-parity counts for rx and readsb, flagging
  any aircraft only rx reported. Two weeks ahead with no
  false frames, then rx becomes the installer's default with readsb as
  fallback.

Timestamp integrity, 2026-09-05: three minutes recorded while the live
Beast stream was collected; every one of the 476 live frames matched
the offline replay with a clock offset of exactly zero ticks from start
to end.

Not in scope: Mode A/C, 978 MHz, other SDRs, the web map, history,
graphs, the aircraft database.

## License

AGPL-3.0-or-later. librtlsdr is LGPL and linked dynamically.
