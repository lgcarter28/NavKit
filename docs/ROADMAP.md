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

## Pass 7.7: reusable trajectory math cleanup

- [ ] Move remaining reusable Earth-rate, triad-calibration, frame-transform, and trajectory-provider conversion helpers into their owning core math/frames/environment locations instead of leaving them buried in trajectory or simulator code. Concrete examples to sweep include `earth_rate_e_radps`, `nonorthogonality_matrix`, `misalignment_matrix`, and the frame-transform helpers currently living in trajectory/simulator implementation files.
- [ ] Define and test the minimum coordinate operations needed by PCPF/ECEF mechanization, local-vertical measurements, NED plots, and future local-level mechanizations.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code, algorithm docs, JSON schemas, and plot labels.
- [ ] Ensure the selected planet and gravity policies are threaded through physics code wherever Earth-specific constants still leak into reusable math.
- [ ] Add straight-line, constant-turn, and short GNSS-outage validation scenarios for the existing ECEF INS/GNSS path.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.

## Pass 7.8: frame-explicit attitude input conventions

- [ ] Accept exactly one configured attitude payload in every supported representation/frame direction: `q_*`, `dcm_*`, and `rpy_*_rad` for `e2b`, `b2e`, `i2b`, `b2i`, `n2b`, and `b2n`.
- [ ] Convert every accepted input at the app-support boundary to the canonical passive body-to-ECEF quaternion `q_b2e`; Navigator and downstream simulation must consume only that canonical form.
- [ ] Add reusable frame-pair conversion utilities and require the context each conversion needs: initial ECEF position for NED forms and timestamp plus the selected ECI/ECEF Earth-orientation convention for inertial forms.
- [ ] Document `rpy_start2end_rad = [roll, pitch, yaw]` as aerospace 3-2-1 yaw-pitch-roll composition encoding the same passive `C_start2end`; explicitly prohibit treating inverse Euler forms as componentwise sign negations.
- [ ] Add positive conversion and negative ambiguity/context regression coverage for every supported attitude input form.

## Pass 7.9: ECI trajectory integration and velocity-aligned attitude

- [ ] Make ECI Cartesian position/velocity/acceleration the canonical dynamic-truth integration state. Keep geodetic integration only as a temporary kinematic/helper path, not the default dynamic propagation scheme.
- [ ] Add reusable translational integration utilities with runtime-selected integration method. Start with explicit/semi-implicit Euler and second-order predictor/corrector trapezoidal integration; require each generated profile to select its method explicitly, document its accuracy/stability assumptions, and reject unknown methods during runtime validation.
- [ ] Add quaternion rotation integration utilities with explicit predictor/corrector endpoint evaluation where angular rate depends on state. Convert all current generated profiles, including ballistic boost/coast, to the ECI integration path and transform to ECEF/local-level outputs only at the boundary.
- [ ] Add and test a velocity-aligned local-level attitude helper: default body forward along velocity, body down aligned with local NED down, and zero NED roll; let clients override the down/reference direction for coordinated-bank behavior and reject degenerate geometry.
- [ ] Define two shallow simulation-only pure-virtual runtime boundaries with explicit typed payloads: a `FlightControlModel` maps trajectory guidance commands plus current realized state to control outputs, and a `VehicleResponseModel` maps those control outputs plus current state/environment to realized body rates, specific force, and state derivatives. Own the selected implementations through `std::unique_ptr` in the trajectory generator and construct them from validated runtime JSON discriminators. Do not introduce compile-time policies or deep inheritance for this application-only seam.
- [ ] Keep model lifecycle and ownership explicit and small: initialize once from runtime configuration and initial truth, advance once per planned trajectory step, and expose command/response diagnostics without leaking concrete implementation types into `SimulationApp` or `TrajectorySource`.
- [ ] Add runtime-configured low-fidelity rotational response for generated trajectories: derive `q_cmd_b2e` from the constrained local-level velocity/bank command, form the body-frame quaternion-error rotation vector, and map it to a desired body rate. Model both flight-control/actuator lag and body rotational response as independently configurable cascaded first-order systems in `p`, `q`, and `r`; apply configured per-axis body-rate saturation only after the final body-response stage, then integrate the realized quaternion. Define `tau_s = 0` as the exact instantaneous-response limit for that stage/axis; reject negative or non-finite time constants.
- [ ] Add the matching runtime-configured translational response model in body coordinates. Treat the response state as commanded non-gravitational acceleration/specific force and model both flight-control/actuator lag and body translational response as independently configurable cascaded first-order systems on its body axes. Apply configured per-axis acceleration saturation only after the final body-response stage, transform realized response to ECI, add gravity, and integrate realized ECI velocity/position. Define `tau_s = 0` as exact command tracking for that stage/axis. IMU truth must use the realized body specific force and body rate, not command quantities.
- [ ] Implement the initial concrete pair as `FirstOrderFlightControlModel` and `FirstOrderVehicleResponseModel`, while keeping the interfaces capable of accepting later second-order control/response implementations. A future coupled rigid-body six-DOF `VehicleResponseModel` should replace the decoupled first-order vehicle model at the same boundary rather than being forced through separate translational and rotational inheritance trees.
- [ ] Extend trajectory truth inspection/logging with explicitly named command and realized response signals: commanded/realized velocity, non-gravitational acceleration or specific force, body angular rate, and attitude, plus tracking errors and applied limits. Add command-versus-response plots so response tuning is inspectable rather than implicit.
- [ ] Make the profile/controller ownership explicit: a velocity-constrained profile produces a smooth velocity/acceleration command, but response dynamics integrate the realized state and therefore may lag the command. Do not simultaneously force the commanded velocity as truth after enabling response dynamics. Add bounded tracking/error diagnostics and stationary/turning regression cases.
- [ ] Keep the selected ECI/ECEF Earth-orientation convention, truth acceleration, attitude-rate, and output-frame assumptions explicit in code, configuration, and the trajectory math documentation.

## Pass 7.10: barebones trajectory truth inspection

- [ ] Add a small, directly inspectable trajectory-truth log and plot suite for generated and CSV scenarios. Keep this deliberately CSV/Plotly-oriented; HDF5 packaging, high-volume caching, and trajectory-analysis optimization remain Phase 20 work.
- [ ] Log and plot, versus time, canonical ECEF and derived ECI position, velocity, acceleration, body-to-reference attitude, and attitude rate.
- [ ] Log and plot derived LLA position; NED velocity, acceleration, attitude, and attitude rate; and body-frame velocity, acceleration, and specific force. Record frame, units, selected Earth-orientation convention, and any derivative/integration assumptions in the log metadata.
- [ ] Make the products independently runtime log-rate configurable and verify them against stationary, ballistic, and turning scenarios so they serve as a compact physics sanity check rather than a second monolithic analysis system.

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
- [`phase_18_guidance_control_vehicle_dynamics.md`](roadmap_details/phase_18_guidance_control_vehicle_dynamics.md): guidance/control signal flow, controlled-attitude point-mass models, and future rigid-body vehicle dynamics.
- [`phase_19_alternative_estimators.md`](roadmap_details/phase_19_alternative_estimators.md): sliding-window, factor-graph, and smoothing backends.
- [`phase_20_simulation_platform.md`](roadmap_details/phase_20_simulation_platform.md): HIL, multi-vehicle simulation, production scenario management, trajectory analysis, and qualification reports.
