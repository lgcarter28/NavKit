# Phase 14 - Tightly Coupled GNSS

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Tightly coupled GNSS follows a working receiver-level hardware product,
latency/replay support, and startup aiding. It remains a major product phase,
but it no longer blocks proving NavKit on real sensors and a real vehicle.

## Pass 14.1: tightly coupled GNSS algorithm document

- [ ] Create a complete standalone LaTeX algorithm document specifying raw
  observables, state definitions, clock modeling, equations, Jacobians,
  noise/error models, validity/sanitization, constellation and receiver data
  contracts, simulator requirements, and variable-DOF statistical tests.

## Pass 14.2: raw and semi-raw observable models

- [ ] Add emulator and filter processing for pseudorange, Doppler, carrier
  phase, and delta range where justified, plus receiver clock bias/drift states,
  initialization, process noise, and diagnostics.
- [ ] Add appropriate atmosphere, satellite clock/orbit, multipath, receiver,
  and measurement errors with explicit fidelity and provenance.
- [ ] Use the Phase 11 measurement-event, context-snapshot, history, and replay
  seams rather than creating a GNSS-specific latency architecture.

## Pass 14.3: constellation and receiver adapters

- [ ] Define bounded per-constellation capacities, observables, frequencies,
  ephemeris/time systems, and validity rather than hiding everything behind an
  unbounded generic GNSS container.
- [ ] Add receiver-specific adapters for devices that supply satellite transmit
  states and devices such as u-blox receivers that require host-side satellite
  state calculation from broadcast data.

## Pass 14.4: integrity and receiver-aiding investigation

- [ ] Add per-SV and whole-epoch fault detection, variable-DOF chi-square
  rejection, RAIM-style checks, quality screening, and a monitored backup
  least-squares solution.
- [ ] Compute DOF from the valid scalar residuals actually tested, cache bounded
  thresholds by configured probability/DOF, and expose DOF, threshold, NIS,
  probability, and decision in diagnostics.
- [ ] Investigate ultra-tightly coupled receiver aiding only after raw-observable
  interfaces and hardware receiver constraints are understood.

## Pass 14.5: receiver replay and HWIL qualification

- [ ] Qualify the raw-observable decoder, timing, satellite-state source,
  variable-DOF update path, and integrity decisions through deterministic raw
  receiver capture/replay before live HWIL.
- [ ] Exercise tightly coupled processing with owned or selected receiver
  hardware when it exposes the required observables. If the available u-blox or
  other legacy receiver cannot provide them, document the receiver limitation
  and select a feasible raw-observable prototype rather than weakening the data
  contract or claiming hardware qualification from simulation alone.
- [ ] Reuse the Phase 12 HWIL evidence contract to compare loose and tight
  coupling under controlled visibility changes, outages, latency, reordered
  epochs, per-SV faults, and degraded measurements. Record solution error,
  availability, exclusion decisions, clock-state behavior, data age, deadlines,
  and estimator health.
- [ ] Treat RF constellation simulation, conducted RF injection, and live-sky
  testing as distinct evidence tiers. Use the highest safe and affordable tier,
  record unavailable stimulus capabilities explicitly, and retain deferred
  acceptance for tests that require equipment not yet owned.
