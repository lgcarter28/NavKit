# NavKit Master Roadmap

This document is the canonical current-state handoff and working roadmap. It was reconciled from earlier planning notes and verified against the repository as it exists today.

The roadmap separates near-term, dependency-ordered engineering from longer-term product ideas. Checked items are verified in the current repository, not merely claimed by an older TODO.

## How to use this roadmap

- Work in order within the near-term phases unless a task is explicitly independent.
- Keep every phase compiling and testable. Preserve the stationary GNSS baseline unless a phase intentionally changes its behavior.
- Treat ADR-001, ADR-002, and ADR-003 as proposed architectural direction. Update their status or content when the implementation decision is actually accepted.
- Before starting a phase, turn it into a small implementation plan with named files, tests, and acceptance evidence.
- Mark an item complete only after its tests and documentation are part of the configured build or workflow.
- Keep owner-only legal, employment, account, release, and backup actions separate from engineering work.

## Reconciliation decisions

These decisions record conflicts and stale assumptions resolved during roadmap consolidation:

- The task to move WGS-84 constants into `Earth.hpp` is superseded. The implemented direction is the generic `planet::Wgs84` policy under `include/navkit/planet`, consistent with ADR-002.
- Environment-policy Pass 1 is substantially implemented: planet/gravity concepts, CRTP bases, frame tags, WGS-84, Moon, Mars, spherical gravity, J2 gravity, and environment tests exist.
- The estimator policy refactor is partially complete. `StateDefPolicy`, `InjectionPolicy`, `ResetPolicy`, `MeasurementPolicy`, `NoisePolicy`, `FilterPolicy`, `SensorCollectionPolicy`, and `UpdatePolicy` exist. `KalmanFilter` is constrained on state, injection, reset, and measurement-model boundaries; `Sensor` is constrained on noise-policy compatibility; and `Navigator` is constrained on current filter, sensor-collection, and update-policy boundaries. Propagation remains future work.
- `GnssPosModel`, `GnssVelModel`, and `BaroAltModel` exist, but only GNSS position is integrated into the current simulation. The barometer model currently selects the third position component; it is not yet a general ECEF-to-local-vertical altitude model.
- The current trajectory generator supports only a simplistic stationary ECEF trajectory. It sets body rate to zero and therefore does not model Earth rotation correctly for stationary IMU truth.
- `ImuSimulator` and `BaroSimulator` are empty shells, and the IMU process model is a placeholder.
- Analysis already provides position error/covariance, innovation, NIS, p-value, mean p-value, and innovation histogram plots. More formal statistics and consistency tests remain future work.
- The documented and configured language standard is C++23.
- `tests/test_state_def_policy.cpp` is included in the configured test target, so its positive and negative concept assertions compile in local and CI builds.

## Current verified baseline

- [x] CMake and Conan build with Eigen, nlohmann-json, and doctest.
- [x] Python wrappers support build, test, stationary simulation, analysis, formatting, and copyright checks.
- [x] Fixed-capacity `RingBuffer`, fixed-size state/covariance aliases, and named state segments exist.
- [x] Generic measurement update, Joseph-form covariance update, injection/reset hooks, sensor queues, Navigator orchestration, and measurement statistics exist.
- [x] Stationary GNSS-position simulation writes truth, measurement, navigation, metadata, manifest, and update-statistics logs.
- [x] Offline plotting is separated from the embedded C++ library.
- [x] Planet, gravity, frame, and basic unit/frame infrastructure exists.
- [x] ADRs document the proposed compile-time policy architecture.
- [x] ADR-003 documents the C++ syntax distinction between unconstrained concept definitions and constrained public template parameters.
- [x] Repository setup, naming, founding, licensing, changelog, and copyright documentation exists.

---

# Phase 0 — Provenance and owner-controlled safeguards

**Goal:** Preserve clear independent provenance and recovery points. These actions are owner-managed and are not prerequisites for ordinary local refactoring unless timing or legal advice makes them so.

- [ ] Complete any required pre-existing-IP disclosure and retain the resulting records.
- [ ] Verify that the proprietary license and copyright notices express the intended ownership model.
- [ ] Create a documented pre-employment/pre-major-refactor release snapshot.
  - [ ] Choose the release/version name.
  - [ ] Create the tag and private release.
  - [ ] Archive the source snapshot and relevant documentation.
- [ ] Maintain independent backups beyond hosted Git remotes.
- [ ] Reserve desired project domains if still useful.

**Exit evidence:** dated release/tag, archived snapshot, verified backups, and owner-retained disclosure records where applicable.

---

# Phase 1 — Baseline integrity and documentation alignment

**Goal:** Make the existing baseline trustworthy before expanding the architecture.

## Build and test integrity

- [x] Add `tests/test_state_def_policy.cpp` to `navkit_tests` and verify its positive and negative assertions compile.
- [x] Audit test sources against `tests/CMakeLists.txt` so every intended test is built.
- [ ] Add a regression command/check for the stationary GNSS simulation and analysis pipeline.
- [x] Configure C++23 consistently in CMake, README, setup, and contributor guidance; verify the supported Debug toolchain.
- [ ] Establish a numerical baseline for stationary GNSS output or selected metrics so refactors can demonstrate unchanged behavior.

## Architecture records

- [ ] Write a concise current architecture overview that distinguishes implemented code from target architecture.
- [ ] Review ADR-001 through ADR-003 and either accept them, revise them, or keep them Proposed with explicit unresolved questions.
- [ ] Reconcile README, `SETUP.md`, and this roadmap after those decisions.

## Automation

- [x] Add ordered Linux and Windows CI for source checks, C++23 Debug builds, tests, simulation, and headless analysis.
- [ ] Confirm the first hosted GitHub Actions run passes on both platforms.
- [ ] Add clang-tidy selectively after the baseline build is stable.
- [ ] Add coverage reporting after the test target accurately represents the suite.

**Exit criteria:** all intended tests are configured, the baseline build/test/simulation workflow is reproducible, language-standard intent is explicit, and architecture documents no longer overstate implementation status.

---

# Phase 2 — Estimator policy boundaries

**Goal:** Complete the next estimator policy refactor pass without changing GNSS-only runtime behavior.

## Injection and reset

- [x] Define candidate-first `InjectionPolicy<Candidate, StateDef>`.
- [x] Define candidate-first `ResetPolicy<Candidate, StateDef>`.
- [x] Constrain `KalmanFilter` on `StateDefPolicy`, injection, and reset policies.
- [x] Preserve the current INS additive-injection sign convention and zero-error reset behavior.
- [x] Keep covariance reset explicitly a no-op until attitude-aware reset is designed.
- [x] Add valid and invalid compile-time policy tests.

## Measurement models

- [x] Define `MeasurementPolicy<Candidate, StateDef>` around dimension, fixed-size matrix types, noise context, observation, Jacobian, covariance, and Kalman-gain operations.
- [x] Decide whether `SensorModelBase` provides enough shared implementation to justify retaining the CRTP base.
- [x] Constrain `KalmanFilter::observation_update` and measurement-statistics storage at the public policy boundary.
- [x] Verify GNSS position, GNSS velocity, and barometer model conformance.
- [x] Add negative compile-time tests for missing types and operations.

## Sensors and noise

- [x] Define a noise-policy compatibility concept for a measurement model and measurement sample.
- [x] Constrain `Sensor<Model, BufferSize, NoisePolicy>`.
- [x] Defer `SensorPolicy` until Navigator or another generic consumer has a real capability boundary that needs it.
- [x] Verify fixed-capacity, allocation-free behavior remains intact.

## Diagnostics

- [x] Decide whether `MeasurementStatistics` needs `StateDef` explicitly or can remain model-derived.
- [x] Preserve innovation, innovation covariance, measurement covariance, Jacobian, gain, NIS, timestamp, validity, and acceptance logging.
- [x] Add runtime regression tests for accepted/rejected update behavior and statistics.

**Exit evidence:** estimator templates are constrained at meaningful current public boundaries: state definition, injection, reset, measurement model, and sensor noise compatibility. Positive and negative concept tests are part of the configured `navkit_tests` target. Accepted and rejected measurement-statistics behavior has runtime regression coverage. `SensorPolicy` is intentionally deferred because no current generic consumer needs a dedicated sensor capability concept; Phase 3 will define the actual Navigator-facing sensor collection boundary. The stationary GNSS Debug build, tests, simulation, logs, and headless analysis were verified during the Phase 2 passes.

**Status:** complete for the current estimator-boundary refactor scope. Remaining filter, sensor collection, propagation, and update concepts move to Phase 3.

---

# Phase 3 — Navigator and propagation seam

**Goal:** Introduce propagation as an orchestration capability without implementing full INS mechanization yet.

- [x] Define the minimum `FilterPolicy` actually required by Navigator.
- [x] Define `SensorCollectionPolicy` only around operations Navigator truly uses.
- [x] Formalize the existing update-policy capability.
- [ ] Define candidate-first `PropagationPolicy` using concrete prediction inputs rather than speculative APIs.
- [ ] Implement `NoOpPropagation` to preserve current GNSS-only behavior.
- [ ] Refactor Navigator to orchestrate prediction, sensor processing, and update policy application.
- [ ] Keep Navigator unaware of planet, gravity, and frame types; those belong inside propagation/mechanization configuration.
- [ ] Add valid and invalid compile-time tests for filter, sensor collection, propagation, and update boundaries.
- [ ] Verify the stationary GNSS numerical baseline remains unchanged with `NoOpPropagation`.

**Exit criteria:** Navigator accepts a propagation policy, the no-op configuration reproduces current behavior, and the public orchestration boundary is concept-tested.

---

# Phase 4 — Navigation physics and simulation contracts

**Goal:** Establish correct truth and sensor contracts before trusting an INS implementation.

## Frames, coordinates, and environment

- [ ] Define the minimum coordinate operations needed by PCPF/ECEF mechanization and local-vertical measurements.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code and documentation.
- [ ] Integrate `planet::Wgs84` and the selected gravity policy into physics code without adding an Earth-specific framework layer.
- [ ] Add tested helpers for required ECEF/geodetic/local-vertical conversions.

## Truth generation

- [ ] Correct stationary ECEF truth, including Earth rotation and nonzero stationary gyro truth.
- [ ] Define acceleration versus specific-force semantics in `TruthSample`.
- [ ] Add straight-line truth.
- [ ] Add constant-rate-turn truth.
- [ ] Add circular motion only if it adds distinct validation value.

## Sensor contracts

- [ ] Define a timestamped IMU sample type with documented increment/rate semantics.
- [ ] Implement an ideal IMU simulator first.
- [ ] Add IMU white noise, bias, bias random walk, scale factor, misalignment, and quantization incrementally with deterministic seeds.
- [ ] Implement the barometer simulator and a physically meaningful altitude model/Jacobian.
- [ ] Integrate the existing GNSS velocity model into simulation.
- [ ] Add explicit sensor scheduling and multi-rate behavior.

**Exit criteria:** ideal stationary and moving truth cases have physics-based tests; ideal sensors reproduce expected measurements; frame and unit conventions are documented at their APIs.

---

# Phase 5 — First complete PCPF/ECEF INS

**Goal:** Cross from a measurement-only estimator to a validated strapdown INS/GNSS navigation system.

## Nominal mechanization

- [ ] Implement a PCPF/ECEF mechanization policy configured by planet and gravity policies.
- [ ] Propagate quaternion attitude with normalization and documented convention.
- [ ] Propagate velocity with gravity, Coriolis, and specific force.
- [ ] Propagate position consistently with the chosen integration scheme.
- [ ] Keep PCI/ECI and local-level mechanizations deferred until the PCPF interface is proven.

## Error-state filter prediction

- [ ] Define the continuous-time error dynamics and process-noise mapping.
- [ ] Implement state-transition and process-noise discretization with documented approximation order.
- [ ] Add covariance prediction and symmetry/positive-semidefinite checks where practical.
- [ ] Connect prediction to the Navigator propagation seam.

## Attitude error handling

- [ ] Implement quaternion/attitude-error injection.
- [ ] Implement the corresponding covariance reset mapping.
- [ ] Verify attitude sign, perturbation side, and frame conventions analytically and numerically.

## Aiding integration

- [ ] Validate GNSS position aiding with the propagating filter.
- [ ] Validate GNSS velocity aiding.
- [ ] Validate barometric altitude aiding after its local-vertical model is corrected.

## Scenarios

- [ ] Stationary Earth alignment/hold test.
- [ ] Straight-line IMU/GNSS test.
- [ ] Constant-turn test.
- [ ] Short GNSS-outage drift test.

**Exit criteria:** ideal-data mechanization tests pass; noisy aided scenarios remain bounded; covariance and innovation outputs are available; the full simulation and analysis workflow runs from repository tools.

---

# Phase 6 — Estimator validation and consistency

**Goal:** Turn plots into repeatable engineering evidence.

## Near-term analysis improvements

- [ ] Add a skip-first-N or time-window option for transient exclusion.
- [ ] Add innovation summary statistics: mean, variance, standard deviation, RMS, and sample count.
- [ ] Add NIS summary statistics: mean, expected mean, 95%/99% exceedance rates, and mean p-value.
- [ ] Overlay expected Gaussian distributions on innovation histograms.
- [ ] Add whitened-innovation time histories and autocorrelation checks.
- [ ] Add a Kolmogorov-Smirnov or other justified uniformity test for p-values, documenting its limitations.

## State consistency

- [ ] Add NEES for scenarios with known truth.
- [ ] Add confidence bounds and exceedance statistics.
- [ ] Produce a machine-readable consistency summary.
- [ ] Generate a human-readable automatic validation report.
- [ ] Define pass/fail thresholds carefully; avoid treating a single stochastic run as proof of consistency.

**Exit criteria:** selected scenarios generate reproducible metrics and reports, statistical assumptions are documented, and failures produce actionable diagnostics.

---

# Phase 7 — Monte Carlo, logging, and performance

**Goal:** Support trade studies and quantify estimator behavior across randomized runs.

## Monte Carlo

- [ ] Add a seeded batch/Monte Carlo driver.
- [ ] Parallelize independent runs without changing determinism.
- [ ] Aggregate RMSE, NEES, NIS, failure counts, and runtime.
- [ ] Generate comparison tables and reports across configurations.

## Logging

- [ ] Add log/schema version metadata and compatibility checks.
- [ ] Add optional runtime performance measurements.
- [ ] Add memory/high-water statistics suitable for embedded evaluation.
- [ ] Consider an optional binary backend only after CSV/JSON throughput becomes a demonstrated bottleneck.

**Exit criteria:** a repeatable batch command produces deterministic aggregate results and records enough version/configuration metadata to reproduce them.

---

# Phase 8 — Robust estimator and embedded readiness

**Goal:** Mature the framework after the first INS is demonstrably correct.

## Time and robustness

- [ ] Define explicit status/error handling for numerical and data-quality failures.
- [ ] Add innovation gating, fault detection, and measurement rejection policy.
- [ ] Add state history and latency handling.
- [ ] Add delayed-measurement replay.
- [ ] Add RTS smoothing after forward propagation/history interfaces stabilize.

## Type and frame safety

- [ ] Extend the existing unit/frame types based on observed misuse risks.
- [ ] Add compile-time DCM composition/result-frame checks.
- [ ] Add tested unit conversions and arithmetic where they improve safety without obscuring Eigen interoperability.
- [ ] Keep frame/type abstractions zero-overhead and verify generated/runtime behavior where important.

## Embedded deployment

- [ ] Define allocation, exception, RTTI, logging, and timing constraints for supported embedded profiles.
- [ ] Separate desktop simulation dependencies from the flight-capable library boundary.
- [ ] Add embedded toolchain profiles and a hardware abstraction boundary when a target is selected.
- [ ] Add runtime and memory budgets to CI or target qualification tests.

## Documentation

- [ ] Add Doxygen/API documentation.
- [ ] Grow the navigation theory/reference manual alongside implemented equations.
- [ ] Add measurement-model and mechanization tutorials.
- [ ] Add end-to-end example walkthroughs and a developer architecture guide.

**Exit criteria:** supported embedded constraints are explicit and tested; robustness features have scenario coverage; public extension points are documented.

---

# Long-term product horizon

These are not scheduled until the first INS, validation pipeline, and extension boundaries are proven.

## Additional environments and mechanizations

- [ ] PCI/ECI mechanization.
- [ ] Local-level and wander-azimuth mechanizations.
- [ ] Atmosphere, magnetic-field, Earth-orientation, geoid, and terrain policies driven by concrete use cases.
- [ ] Multi-planet scenarios using the existing planet-policy direction.

## Additional aiding and navigation modes

- [ ] Magnetometer, radar, lidar, camera, visual odometry, star tracker, and celestial aiding.
- [ ] GPS-denied navigation demonstrations.
- [ ] Integrity monitoring and fault detection/exclusion.
- [ ] Multi-hypothesis and robust estimation.

## Alternative estimators

- [ ] Sliding-window optimization.
- [ ] Factor-graph backend and incremental smoothing.
- [ ] Shared measurement/environment interfaces only where EKF and graph implementations genuinely benefit from them.

## Simulation platform

- [ ] Multi-vehicle simulation.
- [ ] Hardware-in-the-loop integration.
- [ ] Production-grade scenario management and automatic qualification reports.

---

# Milestone summary

1. **Trusted baseline:** configured tests, CI, numerical baseline, aligned documentation.
2. **Constrained estimator:** estimator and diagnostic policy boundaries implemented and tested.
3. **Propagation-ready Navigator:** no-op propagation preserves GNSS-only behavior.
4. **Physics-ready simulation:** correct truth, IMU contracts, and multi-rate ideal sensors.
5. **Complete first INS:** PCPF/ECEF mechanization, covariance prediction, and GNSS/barometer aiding.
6. **Professional validation:** consistency metrics, Monte Carlo, and reproducible reports.
7. **Embedded-ready toolkit:** robustness, type safety, resource budgets, and target profiles.
8. **Advanced navigation platform:** new aiding modes, mechanizations, and estimator backends.
