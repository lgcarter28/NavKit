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
- [x] Phase 6: seeded Monte Carlo campaigns, versioned HDF5 analysis bundles, aggregate covariance/error products, and interactive NEES/NIS consistency diagnostics completed.
- [x] Phase 7.1 through 7.14: exact multi-rate scheduling,
  queryable/streaming truth sources, planned-time application orchestration,
  reusable trajectory math, frame-explicit attitude inputs, ECI trajectory
  integration, generated dynamic profiles, trajectory inspection products,
  the source-agnostic low-fidelity Guidance/Autopilot/Vehicle loop, and the
  evidence-driven trajectory/estimator correctness follow-up completed. The
  latest profile set adds a longer five-g ballistic reference, matched
  skid-to-turn and bank-to-turn horizontal S-turns, planar vertical
  calibration, and a sustained Dutch-roll calibration combining horizontal
  excitation at its base frequency with a twice-frequency vertical excitation
  that traces a front-view half-pipe. Its analysis suite is consolidated into
  frame-explicit kinematics, focused Guidance/Autopilot products, one combined
  Guidance/Control view, tracking errors, and LLA/relative-local 3-D views.
  Pass 7.14 replaces profile-owned transition branches with a validated runtime
  Guidance state graph, separates Guidance, Autopilot, and Vehicle ownership,
  and preserves the accepted seven-scenario Release output suite.

## Current phase

Phase 8.1: deterministic estimator regression baselines is active. Phase 7.1
through 7.14 are complete with repeatable static and dynamic trajectories, closed-loop
source-agnostic Guidance/Autopilot/Vehicle behavior, corrected
truth-minus-estimate and same-epoch aiding semantics, rotating-Earth and
midpoint-attitude consistency, full single-run diagnostics, and six verified
post-Pass-7.12 500-run Monte Carlo campaigns. Pass 7.13 adds seven updated
500-run campaigns spanning ballistic, constant-altitude, horizontal
skid-to-turn, horizontal bank-to-turn, vertical S-turn, half-pipe Dutch roll,
and waypoint bank-to-turn. All 3,500 runs completed successfully with full
analysis bundles and consistency products. Pass 7.14 preserves those accepted
profiles through one runtime-configurable Guidance state machine and explicit
Guidance-to-Autopilot-to-Vehicle contracts. Phase 8 now turns the complete
evidence sequence into deterministic regressions, explicit thresholds,
statistical diagnosis, and qualification reports.

## Pass 8.1: deterministic estimator regression baselines

- [ ] Establish named baseline scenarios, supported compile-time
  product/config combinations, numerical metrics, and explicit pass/fail
  thresholds.
- [ ] Add a single regression command for the default ECEF INS/GNSS simulation
  and analysis pipeline, with machine-readable results suitable for CI.
- [ ] Add a free-inertial truth-reconstruction regression using truth initial
  PVA and ideal IMU increments with GNSS disabled.
- [ ] Add a GNSS-aided truth-reconstruction regression using truth initial PVA,
  ideal IMU increments, and truth GNSS position/velocity measurements.
- [ ] Run both regressions on static trajectories long enough to expose
  cadence/interpolation defects. Interpolate truth to Navigator output
  timestamps and compare ECEF position, velocity, and attitude using justified
  numerical integration tolerances rather than bitwise equality.
- [ ] Preserve the existing perfect/truth-reconstruction runtime inputs and
  execute the applicable regression matrix for each supported selected product
  configuration.
- [ ] Carry the Phase 7 dynamic-trajectory Monte Carlo findings into
  qualification. The earlier six 1,000-run campaigns, the post-Pass-7.11 six
  500-run campaigns, and the post-Pass-7.12 six 500-run campaigns all
  completed. The latest evidence shows generally healthy PVA families and GNSS
  NIS, while constant-altitude and waypoint cases retain elevated full-state
  NEES associated with accelerometer-bias/cross-covariance behavior and weak
  gyro-z observability. Treat this as evidence to diagnose, not as a passing
  consistency result. The seven post-Pass-7.13 campaigns completed 3,500 of
  3,500 runs and produced the full report, HDF5, covariance, NEES, and NIS
  product set. Preserve the skid-to-turn versus bank-to-turn comparison and
  the added sustained horizontal/vertical Dutch-roll excitation as distinct
  qualification evidence.

## Future phase details

Future backlog and completed-phase detail outside the current Phase 8 scope lives in dedicated detail files:

- [`phase_6_monte_carlo.md`](roadmap_details/phase_6_monte_carlo.md): completed Monte Carlo campaign, analysis-bundle, and consistency-diagnostic history.
- [`phase_7_trajectory_provider.md`](roadmap_details/phase_7_trajectory_provider.md): completed trajectory-provider, timebase, planned-time orchestration, scenario, runtime Guidance-state-machine, and reusable navigation-math history.
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
- [`phase_18_guidance_control_vehicle_dynamics.md`](roadmap_details/phase_18_guidance_control_vehicle_dynamics.md): guidance/control signal flow, controlled-attitude point-mass models, and future rigid-body vehicle dynamics.
- [`phase_19_alternative_estimators.md`](roadmap_details/phase_19_alternative_estimators.md): sliding-window, factor-graph, and smoothing backends.
- [`phase_20_simulation_platform.md`](roadmap_details/phase_20_simulation_platform.md): HIL, multi-vehicle simulation, production scenario management, trajectory analysis, and qualification reports.
