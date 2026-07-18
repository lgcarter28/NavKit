# Phase 11 - Tightly Coupled GNSS

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Tightly coupled GNSS is large enough to own a dedicated phase. It should follow the loosely coupled GNSS cleanup so the receiver-level baseline remains clean and comparable.

The first tightly coupled pass should assume clean/current observable timing. Full delayed-observation replay, state-history lookup, and smoothing behavior belong to Phase 12.

## Pass 11.1: tightly coupled GNSS algorithm document

- [ ] Create a complete standalone LaTeX algorithm document for tightly coupled GNSS before implementation. It should fully specify observables, state definitions, clock modeling, measurement equations, Jacobians, covariance/noise models, validity/sanitization logic, constellation/receiver-specific data contracts, and simulation/emulator requirements with enough detail to implement from the document.

## Pass 11.2: raw/semi-raw observable models

- [ ] Add tightly coupled GNSS emulator and Kalman-filter processing support for raw or semi-raw observables such as pseudorange, Doppler, carrier phase, and delta range where appropriate. Treat this as the industry-grade path beyond receiver-level position/velocity aiding.
- [ ] Add GNSS receiver clock states and models, including clock bias and clock drift, with clear state-definition, process-noise, initialization, and logging/plotting support.
- [ ] Add reasonable-fidelity GNSS error models for tightly coupled simulation, including atmospheric, satellite clock/orbit, measurement noise, multipath, and receiver-specific effects as needed by scenario fidelity.
- [ ] Keep full latency/replay support out of the first implementation unless Phase 12 has already provided the required buffering and context-snapshot machinery.

## Pass 11.3: constellation and receiver specificity

- [ ] Decide how much constellation specialization is required for flight-critical-style design. Avoid a vague blanket "GNSS" abstraction where satellite limits, observables, frequencies, ephemeris/time systems, and per-constellation data validity need explicit sizing or handling.
- [ ] Add receiver-specific observable adapters. For example, receivers that provide SV transmit time/position/velocity, such as NavStrike-style messages, can feed different emulator/model paths than receivers such as u-blox that require satellite state calculation or estimation from broadcast ephemeris.

## Pass 11.4: integrity and ultra-tight-coupling investigation

- [ ] Add tightly coupled sanitization and integrity checks: per-SV chi-squared rejection, RAIM-style consistency checks, measurement quality gating, and backup least-squares receiver solutions for monitoring or fallback.
- [ ] Investigate ultra-tightly coupled GNSS/INS support and the receiver-aiding message requirements needed for embedded integration.
