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
- [x] Runtime JSON configs are decomposed into explicit role-keyed components for trajectory, IMU, GNSS, PVA initialization, filter initialization, and propagation overrides, while run-level logging stays inline in each scenario.
- [x] Runtime initialization is split into `pva_initialization` and `filter_initialization`.
- [x] PVA initialization supports random error, explicit error, no-error, and direct-value component examples.
- [x] Filter initial covariance supports compile-time defaults and runtime overrides with diagonal, full, and PVA-plus-remaining-error-state forms.
- [x] Filter covariance floors support compile-time defaults and runtime overrides with diagonal and PVA-plus-remaining-error-state forms.
- [x] Runtime scenario configs include default ECEF INS/GNSS, runtime covariance override, IMU debug, modeled/unmodeled IMU-error, and truth-reconstruction scenarios.
- [x] Simulation outputs are organized under `output/logs/<run_name>/data` and `output/logs/<run_name>/figures`.
- [x] Offline analysis produces ECEF/NED error/covariance plots, dashboard plots, filter-correction plots, GNSS debug plots, IMU increment/debug/error plots, innovation plots, NIS/p-value plots, and histograms.
- [x] `tools/run_scenario.py` provides a one-liner sim-plus-plot workflow.
- [x] `tools/run_monte_carlo.py` provides a seeded campaign workflow with replayable generated run configs and first-pass aggregate covariance plots.
- [x] Versioned Monte Carlo HDF5 bundles cache time-indexed joint NEES/NIS and marginal per-axis series and support interactive consistency dashboards plus machine-readable reports.
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

Phase 7: trajectory-provider and timebase expansion is active. Phase 6 established repeatable statistical evidence; Phase 7 first makes multi-rate timing exact and inspectable before broadening trajectory sources and scenarios.

## Next pass

## Pass 7.4: planned-time application loop and streaming trajectory sources

- [ ] Make the simulation application an embedded-facing planned-time orchestrator. Add a separately configurable rational application rate and reject configurations whose application cadence cannot meet the fastest required consumer deadline.
- [ ] Evolve the simulation-only trajectory boundary from an eagerly synthesized container into a narrow virtual streaming source contract: initialize, `advance_to(t)`, bounded `query(t, sample)`, and completion/status access. Keep virtual dispatch confined to app/simulation infrastructure; product-core navigation remains static/policy composed.
- [ ] Implement stationary generation and CSV playback behind that common source contract. Let each source own its internal resolution and retained query history; consumers query exact timestamps and never depend on a source cadence being an integer multiple of their own.
- [ ] Extend `RationalSchedule` rather than introducing a duplicated ticker: retain consumer `due(t)` semantics and add a producer `next(t)` path with exact sample-index timing. Clearly define initialization/reset ownership and ensure the first `next()` timestamp is strictly after the epoch.
- [ ] Add simulation and real-time clock policies with a common `wait_until(const Timestamp&)` contract. The simulated clock immediately adopts planned time; the real-time/HWIL clock waits to the mapped wall-clock deadline and exposes failure/lateness through the future status seam.
- [ ] Split emulator runtime work into typed two-phase operations. `prepare(source, t, prepared_updates)` may query truth and advance synthetic stochastic state before the deadline but must not mutate Navigator-visible buffers; `publish(prepared_updates, navigator, logger)` makes data observable only after `wait_until(t)`. Keep real hardware acquisition as a distinct post-deadline/asynchronous path rather than pretending it can be precomputed.
- [ ] Drive the app loop with the explicit ownership sequence: obtain `t_curr` from the application scheduler; `trajectory.advance_to(t_curr)`; prepare synthetic emulator updates; `clock.wait_until(t_curr)`; publish prepared updates; then call `navigator.update()`.
- [ ] Document the master-clock, source-resolution, preparation/publication, SWIL, and HWIL contracts in architecture/configuration documentation. Add deterministic timing tests for exact planned timestamps, no early publication, source query bounds/interpolation, and simulation-clock versus real-time-clock boundary behavior.

## Pass 7.5: scenario trajectory expansion

- [ ] Add a simple ballistic trajectory: stationary launch-pad initialization, optional transfer-alignment window, initial heading/pitch definition, and a simple axial body-x boost profile before ballistic/coast behavior. Keep this intentionally simple before adding aero or guidance complexity.
- [ ] Add a constant-altitude, constant-speed trajectory on the curved Earth rather than flat-Earth kinematics.
- [ ] Add calibration-maneuver trajectories: horizontal S-turn, vertical S-turn, and bank-left/bank-right excitation for observability and calibration studies.
- [ ] Add a basic waypoint trajectory with simple bank-to-turn behavior once coordinate, attitude, and trajectory-source contracts are stable.

## Future phase details

Future backlog unrelated to the current Phase 6 Monte Carlo scope lives in dedicated detail files:

- [`phase_6_monte_carlo.md`](roadmap_details/phase_6_monte_carlo.md): current Phase 6 completed-pass history and follow-forward detail.
- [`phase_7_trajectory_provider.md`](roadmap_details/phase_7_trajectory_provider.md): trajectory provider, timebase, planned-time orchestration, scenario, and reusable navigation-math expansion.
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
