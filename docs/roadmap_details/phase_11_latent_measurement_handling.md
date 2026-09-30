# Phase 11 - Latency, Ordered Measurements, Buffering, and Replay

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Timing correctness precedes hardware integration, alignment modes, and raw-GNSS
complexity. This phase establishes the bounded real-time default and makes
delayed correction/replay an explicit optional capability rather than allowing
mutable current context to leak into old observations.

## Pass 11.1: timestamped measurement-event contract

- [ ] Define one timestamp-ordered measurement-event contract containing an
  immutable measurement, time of validity, covariance/noise description,
  validity/quality metadata, sensor identity, sequence information, and the
  observation/model context captured at measurement time.
- [ ] Replace per-sensor batch ordering with bounded timestamp-ordered FIFO
  processing. Multiple observations queued before a Navigator update must be
  evaluated, accepted/rejected, injected, and reset in deterministic temporal
  order, including same-epoch ordering rules.
- [ ] Define late, duplicate, out-of-order, future-dated, corrupt, and
  overflow behavior with explicit status and diagnostics; never silently use
  mutable current lever-arm/angular-rate/noise context for an old observation.

## Pass 11.2: bounded state, covariance, and transition history

- [ ] Add fixed-capacity state/covariance/STM/process-noise history with
  compile-time capacity and runtime horizon validation. Define ownership,
  memory placement, lookup/interpolation, checkpoint cadence, and wraparound.
- [ ] Make measurement-time lookup and correction fail closed outside retained
  history. Report required versus available horizon and resource usage.
- [ ] Add tests for exact/in-between timestamps, history boundaries, multiple
  sensors, same-epoch events, rollover, and deterministic overflow.

## Pass 11.3: delayed correction and forward replay

- [ ] Apply an accepted delayed correction at its measurement epoch and replay
  stored propagation and later measurements to the present without changing
  the default zero-latency result.
- [ ] Preserve deterministic event ordering, gate statistics, applied
  corrections, covariance health, and provenance through replay. Separate
  delayed real-time correction from offline smoothing.
- [ ] Add latency/dropout/reordering injection in simulation and recorded-data
  tools, then verify zero-latency equivalence and bounded delayed-update
  behavior across stationary and dynamic trajectories.

## Pass 11.4: smoothing and batch extensions

- [ ] Add RTS smoothing only after the forward history/replay interfaces are
  stable and measured resource costs are understood.
- [ ] Keep smoothing optional and outside the default real-time embedded path.
