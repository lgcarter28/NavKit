# NavKit Master Roadmap

This document is the canonical current-state handoff and working roadmap. It is reconciled against the repository as it exists today, not merely against older planning notes.

The important status change: NavKit now has a working desktop simulation path for a GPS/GNSS-aided ECEF INS. The remaining roadmap is therefore organized around hardening, validation, scenario breadth, embedded readiness, and future product expansion rather than "first make the INS exist."

Detailed historical phase notes are preserved under `docs/roadmap_details/`. This file is intentionally short; use the detail files only when you need design memory or expanded completed-phase context.

## How to use this roadmap

- Treat checked items as verified current behavior or completed owner/project milestones.
- Treat unchecked items as future work, grouped by the boundary they affect.
- Before starting an unchecked pass, turn it into a small implementation plan with named files, tests, and acceptance evidence.
- Keep normal startup clean: runtime PVA initialization, filter covariance initialization, truth/error simulation, transfer alignment, and advanced analysis/restart features are distinct concepts.
- Keep embedded-facing product-core code free of simulator truth/error context.
- Preserve the working ECEF INS/GNSS scenarios while refactoring.

## Current verified baseline

- [x] Phase 0 owner/provenance work is complete.
- [x] CMake and Conan build with Eigen, nlohmann-json, and doctest.
- [x] The repository uses C++23.
- [x] Python wrappers support bootstrap, build, test, simulation, scenario execution, analysis, formatting, copyright checks, and selected compile-time configs.
- [x] Ninja is the default local build generator through the Python tooling.
- [x] Debug and Release build folders are config-rooted under `build/<type>/apps/navkit_sim/<ConfigName>`.
- [x] Fixed-capacity `RingBuffer`, fixed-size state/covariance aliases, and named state segments exist.
- [x] Product boundaries are split into `navkit::core`, `navkit::sim`, `navkit::io`, `navkit::app_support`, and app executables.
- [x] Public headers are organized by product boundary and engineering domain.
- [x] Planet, gravity, frame, local-level, quaternion, triad-calibration, and basic unit/frame infrastructure exists.
- [x] Environment policy concepts and WGS-84/Moon/Mars/spherical/J2 concrete policies exist.
- [x] State, segment, filter, sensor, measurement-model, update, propagation, logging, initialization, and app-config policy concepts exist where currently useful.
- [x] `KalmanFilter` owns Joseph-form covariance update, covariance propagation, injection/reset hooks, measurement statistics, and optional per-sensor diagnostics.
- [x] `Navigator` owns app-facing orchestration for IMU increments, covariance propagation accumulation, GNSS position/velocity updates, and selected logging hooks.
- [x] The selected simulation app runs ECEF strapdown INS propagation from generated IMU increments before GNSS aiding updates.
- [x] Nominal attitude is quaternion-based with documented body-to-ECEF convention and multiplicative error injection.
- [x] GNSS position and velocity aiding are wired through simulation, emulation, measurement models, update products, and plots.
- [x] GNSS antenna lever-arm support exists in simulator truth generation and measurement-model Jacobians.
- [x] IMU simulation generates deterministic increments from consecutive ECEF truth samples, including Earth rate, specific force, bias, bias random walk, white noise, scale factor, misalignment, non-orthogonality, quantization, and compile-time coning/sculling compensation compatibility.
- [x] Runtime JSON configs are decomposed into reusable components for logging, trajectory, IMU, GNSS, PVA initialization, and filter initialization.
- [x] Runtime initialization is split into `pva_initialization` and `filter_initialization`.
- [x] PVA initialization supports random error, explicit error, no-error, and direct-value component examples.
- [x] Filter initial covariance supports compile-time defaults and runtime overrides with diagonal, full, and PVA-plus-remaining-error-state forms.
- [x] Runtime scenario configs include default ECEF INS/GNSS, runtime covariance override, IMU debug, modeled/unmodeled IMU-error, and truth-reconstruction scenarios.
- [x] Simulation outputs are organized under `output/logs/<run_name>/data` and `output/logs/<run_name>/figures`.
- [x] Offline analysis produces ECEF/NED error/covariance plots, dashboard plots, filter-correction plots, GNSS debug plots, IMU increment/debug/error plots, innovation plots, NIS/p-value plots, and histograms.
- [x] `tools/run_scenario.py` provides a one-liner sim-plus-plot workflow.
- [x] LaTeX algorithm references exist for ECEF navigator v1 and IMU emulator v1.
- [x] Setup, configuration, architecture, naming, founding, license, changelog, copyright, profiling, and roadmap documentation exists.
- [x] Compile-time and runtime tests cover the current policy, config, simulation, logging, and initialization seams.

## Completed milestone history

These are preserved at high level so the roadmap stays readable. Detailed pass-by-pass history lives in Git.

- [x] Phase 0: owner/provenance safeguards completed.
- [x] Phase 1: baseline build/test/config/tooling/docs foundation established.
- [x] Phase 2: estimator policy boundaries implemented and tested.
- [x] Phase 3: compile-time/runtime config architecture, app composition, logging architecture, profiling vocabulary, build/install/output layout, and initialization boundaries implemented.
- [x] Phase 4: Navigator propagation seam established and evolved into the working ECEF INS/GNSS path.
- [x] Phase 5.1: focused ECEF navigator algorithm document written and refined.
- [x] Phase 5.2: IMU emulator algorithm document written and refined.
- [x] Phase 5.3: IMU increment contract and simulator implementation completed.
- [x] Phase 5.4: first full ECEF strapdown aided Navigator implementation completed.
- [x] Phase 5.5 through 5.7: stabilization, covariance propagation ownership, logging/plotting, GNSS velocity, GNSS lever arm, runtime JSON decomposition, initialization split, and scenario tooling completed.

## Current phase

NavKit is currently in Phase 5: ECEF INS/GNSS stabilization and hardening. The active passes below are the current working scope; unrelated future backlog lives in the dedicated phase detail files linked at the end of this roadmap.

## Pass 5.10: full-rate IMU cumulative increment logging

- [ ] Move IMU cumulative increment ownership out of `ImuIncrementLogProduct` and into the full-rate IMU runtime/simulator path so cumulative sums are updated for every generated IMU sample, not only for rows that pass the runtime logging-rate gate.
- [ ] Extend the IMU log payload to carry full-rate run cumulative sums for truth/ideal and measured increments; make the log product write supplied cumulative snapshots rather than accumulating internally.
- [ ] Preserve lower-rate logging as a snapshot mechanism: when IMU logging is decimated, each logged row should contain the latest increment plus the full-rate run cumulative sums up to that timestamp.
- [ ] Update IMU cumsum plot titles/labels or metadata to make clear they are full-rate cumulative increment snapshots, and keep any future log-interval sums distinct from run cumulative sums.
- [ ] Add a focused test or scenario check that verifies cumulative Z specific-force/velocity increment growth remains correct when IMU generation runs faster than IMU logging.

## Pass 5.11: first-order Gauss-Markov IMU dynamics

- [ ] Update the LaTeX algorithm references first: `imu_emulator_v1` for first-order Gauss-Markov IMU modeling, and `navigator_ecef_v1` for the matching IMU error-state modeling and strapdown navigation dynamic equations.
- [ ] Upgrade IMU bias/error dynamics from pure random walk to first-order Gauss-Markov models where appropriate.
- [ ] Keep the IMU simulator/emulator dynamics, filter STM, process-noise discretization, runtime config, docs, logs, and plots consistent.
- [ ] Add tests or scenario diagnostics that distinguish the first-order Gauss-Markov behavior from the existing pure-random-walk behavior.

## Pass 5.12: covariance floors and unused-parameter cleanup

- [ ] Add configurable covariance floors per state-family/diagonal block to prevent covariance from becoming ill-conditioned or singular during idealized analysis runs.
- [ ] Sweep app-support and core seams for stale unused-parameter breadcrumbs such as `(void)cfg;`. Remove unused parameters and simplify call sites when the signature no longer needs the value; keep explicit `(void)name;` only when preserving a required concept/API signature is intentional.

## Pass 5.13: advanced restart initialization groundwork

- [ ] Preserve the normal startup path as the clean default: `pva_initialization` provides the required nominal PVA message, and `filter_initialization.initial_covariance` provides the full Kalman filter covariance belief. Do not pollute this path with Monte Carlo, simulator truth-error, or restore-only concepts.
- [ ] Add an optional direct non-PVA nominal state override for restore/manual analysis use cases. This belongs under `filter_initialization` as an explicit advanced feature, should initialize selected non-PVA nominal estimated states such as gyro and accelerometer bias estimates, and should not masquerade as transfer alignment or PVA initialization.
- [ ] Keep transfer alignment observation-driven. Transfer alignment may provide timestamped aiding data that lets the filter estimate corrections through normal measurement updates; it must not become a hidden mechanism for directly writing filter nominal states or pending correction vectors.
- [ ] Keep product-core embedded code free of simulator truth/error context. Direct restore-style nominal state overrides may be product/app configuration, but Monte Carlo truth-relative initialization is app/sim/analysis infrastructure only.
- [ ] Add focused tests demonstrating the separation: normal PVA/covariance startup remains unchanged, direct non-PVA nominal override initializes only selected state segments, and transfer alignment remains independent.

## Future phase details

Future backlog unrelated to the current Phase 5 stabilization scope lives in dedicated detail files:

- [`phase_6_monte_carlo.md`](roadmap_details/phase_6_monte_carlo.md): Monte Carlo and batch-analysis support.
- [`phase_7_trajectory_provider.md`](roadmap_details/phase_7_trajectory_provider.md): trajectory provider, timebase, scenario, and reusable navigation-math expansion.
- [`phase_8_estimator_validation.md`](roadmap_details/phase_8_estimator_validation.md): estimator validation, consistency metrics, and repeatable reports.
- [`phase_9_status_error_handling.md`](roadmap_details/phase_9_status_error_handling.md): robust status/error handling before the later high-complexity phases.
- [`phase_10_sensor_model_cleanup.md`](roadmap_details/phase_10_sensor_model_cleanup.md): loosely coupled GNSS cleanup, altimeter/pressure models, pitot tube, magnetometer aiding, and sensor scheduling.
- [`phase_11_tightly_coupled_gnss.md`](roadmap_details/phase_11_tightly_coupled_gnss.md): tightly coupled GNSS, raw observables, constellations, receiver adapters, and integrity seams.
- [`phase_12_latent_measurement_handling.md`](roadmap_details/phase_12_latent_measurement_handling.md): latent measurement context, buffering, replay, and smoothing foundations.
- [`phase_13_transfer_alignment_stationary_modes.md`](roadmap_details/phase_13_transfer_alignment_stationary_modes.md): transfer alignment, coarse/fine alignment, and ZUPTs after buffering support.
- [`phase_14_profiling_resource_validation.md`](roadmap_details/phase_14_profiling_resource_validation.md): profiling, resource, allocation, and target evidence.
- [`phase_15_embedded_hardening.md`](roadmap_details/phase_15_embedded_hardening.md): remaining embedded readiness, type/API hygiene, documentation, and CI/release workflow.
- [`phase_16_advanced_algorithms.md`](roadmap_details/phase_16_advanced_algorithms.md): advanced GNSS techniques, vision/LiDAR/SLAM, celestial/radar/external aiding, GPS-denied demonstrations, and robust/multi-hypothesis algorithms.
- [`phase_17_additional_mechanizations_environments.md`](roadmap_details/phase_17_additional_mechanizations_environments.md): additional mechanizations, environments, and physical models.
- [`phase_18_alternative_estimators.md`](roadmap_details/phase_18_alternative_estimators.md): sliding-window, factor-graph, and smoothing backends.
- [`phase_19_simulation_platform.md`](roadmap_details/phase_19_simulation_platform.md): HIL, multi-vehicle simulation, production scenario management, and qualification reports.
