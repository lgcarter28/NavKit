# Phase 20 - Simulation Platform and Qualification Infrastructure

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase turns the simulator and analysis tooling into a broader platform after the core navigation, validation, embedded, and advanced-aiding capabilities are stable.

## Pass 20.1: multi-vehicle simulation

- [ ] Add multi-vehicle simulation support when scenario management, logging, and analysis outputs can represent multiple truth and estimate streams clearly.
- [ ] Define inter-vehicle timing, relative measurements, shared environment assumptions, and output naming conventions before implementation.

## Pass 20.2: mixed-HWIL and external-flight-computer orchestration

- [ ] Extend the Pass 8.8 composition seam into a dedicated mixed-HWIL
  application whose channel graph assigns exactly one producer to every input:
  emulator, real transport, or external stimulus equipment. Keep reusable
  simulator models under `navkit::sim` and hardware transports outside the
  target-neutral mission host.
- [ ] Add a separate long-range HWIL orchestrator for production flight-computer
  hardware running its real flight binary. The orchestrator owns simulated
  truth/plant state, clocks, networking and buses, RF/analog/digital stimulus,
  faults, telemetry, and actuator feedback; the flight binary remains unaware
  of the test environment.
- [ ] Define transport timing, startup/shutdown, failure modes, per-channel
  ownership, synchronization, and retained qualification evidence before the
  first concrete hardware adapter is accepted.

## Pass 20.3: production scenario management and qualification reports

- [ ] Add production-grade scenario management for large scenario suites, parameter sweeps, and qualification campaigns.
- [ ] Generate automatic qualification reports that combine scenario manifests, config versions, binary/build metadata, validation metrics, plots, timing, and resource evidence.
- [ ] Keep qualification artifacts reproducible from committed configs, seeds, source revision, and tool versions.

## Pass 20.4: trajectory truth analysis and visualization

- [ ] Extend the offline analysis bundle schema to package trajectory truth efficiently in HDF5, retaining time plus canonical ECEF position, velocity, acceleration, body-to-ECEF attitude, and body inertial angular rate.
- [ ] Profile and optimize high-volume HDF5 trajectory/campaign packaging before adding broad concurrency: define raw-table retention modes, derived-only modes, bounded chunk layouts, compression choices, source/config fingerprints, cache reuse rules, and per-stage timing/size evidence. Keep the C++ runtime logger CSV/binary-telemetry agnostic; HDF5 remains an offline Python packaging concern.
- [ ] Derive and cache trajectory views in ECI; LLA position; local-level NED velocity, acceleration, and attitude; and body-frame velocity, acceleration, specific force, and angular-rate quantities. Record the frame, units, planet/gravity, and Earth-orientation assumptions with each product.
- [ ] Add trajectory flight-path products: NED azimuth/elevation/flight-path angle, plus aerodynamic angle-of-attack/sideslip and total-angle/wind-roll views. Start with explicitly zero wind, then add wind and atmospheric state/log products when an atmosphere model exists.
- [ ] Provide reusable static and Plotly interactive trajectory renderers, including quick named-field or explicit x/y dashboard inspection from either CSV run folders or HDF5 bundles. Reuse the existing domain-data-preparation and renderer split instead of duplicating plotting logic.
- [ ] Keep HDF5 as offline packaging/post-processing for desktop truth logs until an embedded binary telemetry contract is deliberately designed; do not make direct HDF5 logging a NavKit embedded dependency.
