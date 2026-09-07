# Plan

Milestones with their acceptance tests, as recorded when each landed.

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
  minutes; a machine we operate pulls the clips nightly and appends one
  line per clip to a shadow log
  with CRC-valid and address-parity counts for rx and readsb, flagging
  any aircraft only rx reported. The first night flagged three; all
  three are real (two appear in readsb's output in neighbouring clips,
  the third is a clean all-call readsb missed), so the flag means "readsb
  missed it", and a false frame would show as an address neither the
  aggregators nor later clips know. Two weeks ahead with no
  false frames, then rx becomes the installer's default with readsb as
  fallback.

Timestamp integrity, 2026-09-05: three minutes recorded while the live
Beast stream was collected; every one of the 476 live frames matched
the offline replay with a clock offset of exactly zero ticks from start
to end.

