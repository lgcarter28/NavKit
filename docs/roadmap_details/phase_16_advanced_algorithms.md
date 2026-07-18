# Phase 16 - Advanced Navigation Algorithms

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase collects advanced algorithms that should not block the nearer-term Monte Carlo, validation, sensor cleanup, tightly coupled GNSS, latency, transfer-alignment, profiling, or embedded-hardening phases.

## Pass 16.1: advanced GNSS techniques

- [ ] Add differential GNSS support after tightly coupled GNSS observables, receiver adapters, and integrity-monitoring seams are stable.
- [ ] Add RTK support after differential GNSS and carrier-phase handling are mature enough to justify ambiguity-state and correction-stream complexity.
- [ ] Decide which correction data formats, base/rover assumptions, ambiguity-resolution strategies, and logging/validation artifacts are required before implementation.

## Pass 16.2: vision navigation

- [ ] Add vision navigation algorithm documentation before implementation, covering camera models, feature tracking, measurement equations, covariance/error models, frame conventions, timing/latency, and estimator integration.
- [ ] Add camera-based feature tracking and visual odometry support when trajectory, timing, and measurement-buffering seams can support it.
- [ ] Add terrain-relative navigation and optical navigation scenarios when maps/terrain/star/target reference data contracts are clear.
- [ ] Evaluate visual-inertial integration only after the baseline vision measurement path and latent measurement handling are stable.

## Pass 16.3: LiDAR, SLAM, and map-relative navigation

- [ ] Add LiDAR aiding documentation before implementation, covering point-cloud/range observables, scan matching, feature extraction, covariance/error models, frame conventions, timing/latency, and estimator integration.
- [ ] Add LiDAR odometry and LiDAR-inertial odometry support after trajectory, buffering, and map/reference data contracts are stable.
- [ ] Add SLAM support as a distinct advanced path, including map-state ownership, landmark/feature representation, loop closure, covariance/consistency handling, and the boundary between online navigation state and mapping state.
- [ ] Add map-relative navigation scenarios for terrain, landmarks, point clouds, or surveyed infrastructure once map/reference-data formats are selected.
- [ ] Keep SLAM/factor-graph-style backends separated from the default EKF path until shared measurement/environment interfaces have a concrete reason to exist.

## Pass 16.4: celestial, radar, and external aiding

- [ ] Add star-tracker/celestial aiding support with inertial reference catalogs, attitude measurement models, timing requirements, covariance models, and frame-transform conventions documented before implementation.
- [ ] Add radar aiding support for range, range-rate, angle, altimetry, terrain-relative, or beacon/landmark-style measurements as scenario needs mature.
- [ ] Add generic ground-based/external aiding source abstractions for beacons, surveyed landmarks, RF/radar sites, motion-capture, or range/angle/range-rate infrastructure without forcing unrelated sensors into one vague model.
- [ ] Add simulation scenarios that exercise mixed aiding sources in GPS-denied or GNSS-degraded conditions.

## Pass 16.5: advanced estimator/navigation modes

- [ ] Add GPS-denied navigation demonstrations using mature non-GNSS aiding sources.
- [ ] Add integrity monitoring and fault detection/exclusion beyond the initial GNSS-specific checks.
- [ ] Add multi-hypothesis and robust estimation when the core EKF validation and logging/reporting infrastructure can support comparisons.
