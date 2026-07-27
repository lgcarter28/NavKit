# Phase 19 - Alternative Estimators

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase explores estimator backends beyond the default EKF after the measurement, validation, buffering, and reporting seams have enough maturity to support meaningful comparisons.

## Pass 19.1: sliding-window optimization

- [ ] Add a sliding-window optimization investigation and design document before implementation.
- [ ] Define which states, measurements, priors, marginalization behavior, and runtime constraints make sense for NavKit.
- [ ] Keep the default EKF path clean; do not introduce shared abstractions until both EKF and windowed implementations clearly benefit.

## Pass 19.2: factor-graph and incremental smoothing backend

- [ ] Add a factor-graph backend only after the measurement/environment interfaces have a concrete reason to be shared.
- [ ] Define factor ownership, graph state, marginalization, linearization points, and replay/latency interactions explicitly.
- [ ] Add comparison reports against the EKF baseline using the Monte Carlo and validation infrastructure.

## Pass 19.3: shared estimator interfaces

- [ ] Extract shared measurement/environment interfaces only where the EKF, sliding-window, and graph implementations genuinely benefit.
- [ ] Avoid turning EKF-specific policy concepts into overly broad abstractions that obscure the default embedded path.
