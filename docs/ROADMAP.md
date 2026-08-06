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
- [x] A versioned deterministic regression runner covers stationary
  free-inertial, stationary truth-GNSS, ballistic, and bank-to-turn truth
  reconstruction with strict time alignment, declared numerical contracts,
  compact reports, and failure-only artifact retention.
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
- [x] Phase 8.1: deterministic stationary and dynamic truth-reconstruction
  regressions, compact reports, and failure-only artifact retention completed.

## Current phase

Phase 8 turns the existing single-run, Monte Carlo, HDF5, and interactive
consistency evidence into deterministic estimator regressions, runtime
measurement acceptance, explicit qualification criteria, and diagnosis of the
remaining dynamic-profile consistency findings. Phase 7.1 through 7.14 provide
the repeatable static and dynamic truth sources required for this work.

## Pass 8.2: runtime innovation acceptance and correction integrity

- [ ] Add configurable chi-square innovation gates for each GNSS observation
  family. Evaluate the gate before measurement injection, reject invalid
  observations deterministically, and log the threshold, NIS, and accepted
  status.
- [ ] Add unit and end-to-end tests for accepted and rejected position and
  velocity observations, including sequential position/velocity updates at one
  epoch.
- [ ] Add a replay/reconstruction regression proving filter-correction logging
  is exact when multiple accepted updates occur at one epoch: emit one event
  per injection or record an equivalent exact composed result.

## Pass 8.3: observability analysis and interactive visualization

- [ ] Define a versioned, state-definition-aware observability data contract
  for Python analysis. Preserve state labels/order, frames, units, reference
  epochs, measurement families/timestamps, measurement covariance or whitened
  Jacobians, and the discrete state transitions needed to transport each
  measurement sensitivity through a declared analysis window. Add only the
  runtime-enabled analysis logging needed to populate that contract; do not
  burden the embedded hot path with desktop visualization concerns.
- [ ] Implement numerically scaled local-linear observability/information
  analysis. Form measurement-whitened sensitivities, retain full cross-state
  coupling, and support cumulative and sliding finite windows, individual
  sensors, sensor combinations, and maneuver/state-machine intervals. Use
  SVD/QR or equivalent stable factorizations to report effective rank,
  singular spectrum, condition, information growth, and strongest/weakest
  observable modes without confusing mixed state units with observability.
- [ ] Add an explicitly separate empirical observability mode based on paired,
  deterministically seeded state perturbations and central-difference output
  sensitivities. Use it to cross-check the local linearized result on nonlinear
  trajectories; do not present either finite-window diagnostic as proof of
  global nonlinear or structural observability.
- [ ] Add reusable HDF5-backed Python derivations and responsive Plotly views
  for singular-value/rank history, cumulative and sliding-window information,
  state/state-family sensitivity heatmaps, mode-composition heatmaps,
  weakest-mode evolution, per-sensor information contribution, and
  maneuver-to-maneuver comparison. Provide interactive epoch/window and state
  selection plus publication-quality Matplotlib export from the same derived
  data rather than duplicating analysis logic.
- [ ] Validate the tooling against small systems with known observable and
  unobservable modes, rank tolerances, state rescaling, sensor combinations,
  and window boundaries. Apply it to stationary, ballistic, S-turn,
  Dutch-roll, and waypoint cases to quantify attitude and IMU-bias
  observability—especially yaw and gyro-z—rather than inferring observability
  solely from covariance contraction.
- [ ] Document the equations, scaling/whitening conventions, numerical rank
  policy, interpretation limits, required logs, and interactive workflow in
  the analysis documentation, with a future standalone LaTeX treatment if the
  reference grows beyond a concise implementation contract.

## Pass 8.4: stochastic qualification and dynamic-profile diagnosis

- [ ] Define named Monte Carlo campaign sizes, analysis windows, and
  statistically justified pass/fail criteria. Existing NEES/NIS, coverage,
  CDF/PIT, QQ, and interactive products are the evidence surface; do not
  duplicate them with another plotting stack.
- [ ] Diagnose elevated full-state NEES in the constant-altitude and waypoint
  profiles, beginning with accelerometer-bias cross-covariance, reset,
  linearization, process-noise, and numerical covariance effects while
  preserving the credible PVA-family and GNSS-NIS evidence.
- [ ] Use the Pass 8.3 observability products alongside covariance and
  consistency evidence to explain maneuver-dependent attitude and modeled
  IMU-bias behavior rather than treating covariance contraction alone as
  observability proof.

## Pass 8.5: qualification reports and CI baseline management

- [ ] Produce a compact qualification report from deterministic and stochastic
  results: threshold outcomes, configuration/build/schema provenance, baseline
  deltas, diagnostic-artifact links, and explicit disposition of known
  consistency findings.
- [ ] Integrate the deterministic regression matrix into CI and retain only
  failure artifacts plus explicitly requested qualification bundles.

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
