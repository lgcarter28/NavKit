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
- [x] Debug and Release build folders are config-rooted under `build/<type>/apps/navkit_swil/<ConfigName>`.
- [x] Fixed-capacity `RingBuffer`, fixed-size state/covariance aliases, and named state segments exist.
- [x] Product boundaries are split into `navkit::core`, `navkit::sim`, `navkit::io`, target-neutral `navkit::app_support_common`, SWIL `navkit::swil_support`, and thin app executables.
- [x] Public headers are organized by product boundary and engineering domain.
- [x] Planet, gravity, frame, local-level, quaternion, triad-calibration, and basic unit/frame infrastructure exists.
- [x] Environment policy concepts and WGS-84/Moon/Mars/spherical/J2 concrete policies exist.
- [x] State, segment, filter, sensor, measurement-model, update, propagation, logging, initialization, and app-config policy concepts exist where currently useful.
- [x] `KalmanFilter` owns Joseph-form covariance update, covariance propagation, injection/reset hooks, measurement statistics, and optional per-sensor diagnostics.
- [x] `Navigator` owns app-facing orchestration for IMU increments, covariance propagation accumulation, GNSS position/velocity updates, and selected logging hooks.
- [x] The selected SWIL app runs ECEF strapdown INS propagation from generated IMU increments before GNSS aiding updates.
- [x] Nominal attitude is quaternion-based with documented body-to-ECEF convention and multiplicative error injection.
- [x] GNSS position and velocity aiding are wired through simulation, emulation, measurement models, update products, and plots.
- [x] GNSS antenna lever-arm support exists in simulator truth generation and measurement-model Jacobians.
- [x] IMU simulation generates deterministic increments from consecutive ECEF truth samples, including Earth rate, specific force, bias, bias random walk, white noise, scale factor, misalignment, non-orthogonality, quantization, and compile-time coning/sculling compensation compatibility.
- [x] Runtime JSON configs are decomposed into explicit role-keyed components for mission, IMU, GNSS, PVA initialization, filter initialization, and propagation overrides, while run-level logging stays inline in each scenario.
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
- [x] Phase 8.2: runtime-configured chi-square innovation acceptance for
  separate GNSS position and velocity observations, deterministic pre-mutation
  rejection diagnostics, and exact composed same-epoch correction logging
  completed.

## Current phase

Phase 8 has turned the existing single-run, Monte Carlo, HDF5, and interactive
consistency evidence into deterministic estimator regressions, runtime
measurement acceptance, and a fail-closed qualification/reporting framework.
The initial seven-profile current-config campaign exposed nine required
failures caused by startup innovation-gate lockout. Runtime state-entry actions
now keep GNSS position/velocity chi-square rejection disabled during each
declared acquisition phase and restore the configured thresholds afterward. A
fresh 3,500-run qualification matrix passes all 39 required stochastic checks;
the ballistic gate/PVA tail and narrow bank/skid confidence misses are gone.
Constant-altitude and waypoint full-state cross-covariance findings remain
explicitly declared, and managed-baseline creation is still deliberately
pending. Phase 7.1 through 7.14 provide the repeatable static and dynamic truth
sources required for this work.

Pass 8.4 qualification/reporting, Pass 8.5 mission/subsystem/target
configuration boundaries, Pass 8.6 explicit runtime-graph migration, Pass 8.7
target-agnostic mission runtime/execution ownership, and Pass 8.8
target-specific SWIL application composition are complete and archived in the
Phase 8 detail file.

## Pass 8.9: managed qualification baseline promotion

- [ ] Generate and check in a current-config 500-run managed baseline, validate
  it through the read-only baseline command, and verify an ordinary
  qualification run reports a compatible current-schema/current-input
  comparison.

## Pass 8.10: observability analysis and interactive visualization

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
  observability, especially yaw and gyro-z, rather than inferring observability
  solely from covariance contraction.
- [ ] Document the equations, scaling/whitening conventions, numerical rank
  policy, interpretation limits, required logs, and interactive workflow in
  the analysis documentation, with a future standalone LaTeX treatment if the
  reference grows beyond a concise implementation contract.

## Future phase details

Future backlog and completed-phase detail outside the current Phase 8 scope lives in dedicated detail files:

The prioritized path after Phase 8 is deliberately hardware-facing:

```text
Phase 8  estimator validation and managed evidence
Phase 9  robust status/error handling
Phase 10 flight-relevant sensor models and legacy-hardware characterization
Phase 11 latency, ordered measurements, bounded history, and replay
Phase 12 replay/mixed-HWIL/external-flight-computer HWIL and flight MVP
Phase 13 alignment, transfer alignment, and stationary aiding
Phase 14 tightly coupled GNSS
Phase 15 remaining profiling, resource, embedded, telemetry, and release hardening
```

This order uses the existing loosely coupled GNSS-aided INS to expose real
timing, calibration, transport, resource, and operational problems before raw
GNSS observables expand estimator complexity. Phase 10 prioritizes the
now-reconciled 2018-2019 capstone inventory: an MPU9250-family
IMU/magnetometer, BNO055, BMP280-class barometer, MPXV7002DP-class
pitot/differential-pressure assembly, u-blox NEO-6M, microSD logging, and legacy
Arduino/STM32 sensor bridges. Phase 12 first establishes WSL/Linux/container
parity and performs the budget prototype trade only after that hardware reveals
which upgrades matter. The incremental hardware budget targets a few hundred
dollars and requires an explicit review before crossing approximately $500;
workstation replacement remains a separate priority. The first airborne
demonstration uses a proven ArduPilot/PX4-compatible multicopter with NavKit in
replay/shadow or companion mode, not a custom flight-critical controller or a
revived capstone airframe.

The immediate hardware path deliberately reuses the Mega/Blue Pills as
sensor bridges and cross-toolchain targets while NavKit executes on the host.
The first milestone is timestamped raw capture, identical live/replay adapter
paths, and synchronized legacy-sensor operation—not forcing the full Navigator
onto undersized boards. Initial purchases are limited to bench visibility and
electrical-safety tools plus one STM32H7-class development board (roughly
$100-$150 total before legacy bring-up evidence). Timing/power infrastructure,
a proven ArduPilot/PX4 flight platform, and modern raw-observation/PPS GNSS are
subsequent evidence-gated steps; a better IMU is not the default first buy.

Phase 12 also establishes a reusable hardware-qualification evidence tier for
all later work. Every hardware-relevant capability added after that milestone
must define and execute recorded-data replay and HWIL acceptance when the owned
or selected hardware can exercise it, followed by field or flight evidence when
that is safe and proportionate. If available hardware, receiver observables,
stimulus equipment, environment, or safety constraints make HWIL infeasible,
the pass must document the exact limitation, complete the highest feasible
evidence tier, and retain an explicit deferred hardware-acceptance item. A
simulation result must never be presented as HWIL or flight qualification.

- [`phase_6_monte_carlo.md`](roadmap_details/phase_6_monte_carlo.md): completed Monte Carlo campaign, analysis-bundle, and consistency-diagnostic history.
- [`phase_7_trajectory_provider.md`](roadmap_details/phase_7_trajectory_provider.md): completed trajectory-provider, timebase, planned-time orchestration, scenario, runtime Guidance-state-machine, and reusable navigation-math history.
- [`phase_8_estimator_validation.md`](roadmap_details/phase_8_estimator_validation.md): estimator validation, consistency metrics, and repeatable reports.
- [`phase_9_status_error_handling.md`](roadmap_details/phase_9_status_error_handling.md): robust status/error handling before the later high-complexity phases.
- [`phase_10_sensor_model_cleanup.md`](roadmap_details/phase_10_sensor_model_cleanup.md): flight-relevant loosely coupled GNSS, IMU/magnetometer, barometer, pitot/air-data, legacy-hardware characterization, and Allan-deviation V&V.
- [`phase_11_latent_measurement_handling.md`](roadmap_details/phase_11_latent_measurement_handling.md): timestamped measurement events, latency, ordered buffering, bounded state history, replay, and smoothing foundations.
- [`phase_12_hardware_flight_mvp.md`](roadmap_details/phase_12_hardware_flight_mvp.md): Linux/WSL/container parity, embedded smoke target, replay, mixed in-process HWIL, external-flight-computer HWIL orchestration, a budget prototype trade study, and a graduated flight demonstration.
- [`phase_13_transfer_alignment_stationary_modes.md`](roadmap_details/phase_13_transfer_alignment_stationary_modes.md): transfer alignment, coarse/fine alignment, and ZUPTs after buffering support.
- [`phase_14_tightly_coupled_gnss.md`](roadmap_details/phase_14_tightly_coupled_gnss.md): tightly coupled raw GNSS observables, clock states, constellation/receiver adapters, and integrity seams after latency and hardware foundations exist.
- [`phase_15_embedded_hardening.md`](roadmap_details/phase_15_embedded_hardening.md): remaining profiling, resource qualification, embedded readiness, type/API hygiene, binary telemetry, documentation, and CI/release workflow.
- [`phase_16_advanced_algorithms.md`](roadmap_details/phase_16_advanced_algorithms.md): advanced GNSS techniques, vision/LiDAR/SLAM, celestial/radar/external aiding, GPS-denied demonstrations, and robust/multi-hypothesis algorithms.
- [`phase_17_additional_mechanizations_environments.md`](roadmap_details/phase_17_additional_mechanizations_environments.md): additional mechanizations, environments, and physical models.
- [`phase_18_guidance_control_vehicle_dynamics.md`](roadmap_details/phase_18_guidance_control_vehicle_dynamics.md): guidance/control signal flow, controlled-attitude point-mass models, and future rigid-body vehicle dynamics.
- [`phase_19_alternative_estimators.md`](roadmap_details/phase_19_alternative_estimators.md): sliding-window, factor-graph, and smoothing backends.
- [`phase_20_simulation_platform.md`](roadmap_details/phase_20_simulation_platform.md): advanced multi-target HWIL, multi-vehicle simulation, production scenario management, trajectory analysis, and qualification reports beyond the Phase 12 MVP.
