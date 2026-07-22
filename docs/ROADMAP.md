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

NavKit is currently in Phase 6: Monte Carlo and batch analysis. Phase 5 produced the first working ECEF INS/GNSS baseline; Phase 6 turns that single-run product into repeatable statistical evidence.

## Active passes

## Pass 6.3: schema versioning and compatibility

- [ ] Add Monte Carlo-specific schema/version metadata and compatibility rules for campaign manifests, aggregate reports, scenario-expanded runtime inputs, log schemas, and plot inputs.
- [ ] Define schema/version metadata for the future analysis bundle format, including bundle root metadata, per-run raw-data tables, per-run derived products, aggregate products, plot-ready cached arrays, units, frames, and derivation assumptions.
- [ ] Define a small plot-spec/data-series schema that separates "what to plot" from "how to render it" so Matplotlib and interactive backends can share the same prepared data without duplicating domain plotting logic.
- [ ] Define compatibility expectations for config schemas, log schemas, plot inputs, and aggregate report formats so older analysis artifacts can either be read deliberately or rejected with clear diagnostics.
- [ ] Document the expected compatibility behavior for CSV-backed analysis, HDF5-backed analysis bundles, and mixed workflows where existing CSV campaign folders are packaged after the fact.
- [ ] Document the migration/update policy for generated logs and reports before they become qualification evidence.

## Pass 6.4: analysis bundle and interactive plotting infrastructure

- [ ] Preserve current CSV logs as the simple portable raw desktop simulation artifact while prototyping an optional packed analysis artifact for large single-run and Monte Carlo workflows.
- [ ] Add a Python packaging entry point, such as `tools/package_analysis.py`, that reads existing run/campaign folders, validates schemas, computes/cache common derived products, and writes an analysis bundle without changing C++ logging.
- [ ] Prototype HDF5 as the first packed analysis bundle format for both single runs and Monte Carlo campaigns, with a reusable hierarchy such as `/runs/<run_id>/data`, `/runs/<run_id>/derived`, `/aggregate`, and `/metadata`.
- [ ] Store schema version, source runtime configs, compile-time config metadata, seeds, log schema names, units, frame conventions, time-window/decimation choices, and derivation assumptions inside the bundle to avoid future post-processing footguns.
- [ ] Cache high-cost derived products such as truth-aligned navigation errors, ECEF-to-NED transformed errors/covariances, NIS/NEES arrays, empirical covariance summaries, and selected downsampled plot-ready arrays.
- [ ] Refactor plotting loaders so single-run and Monte Carlo plots can consume either raw CSV folders or a packed analysis bundle through a shared data-access interface with minimal duplicated plotting code.
- [ ] Refactor plotting around shared prepared plot data/spec objects: domain plot builders should produce common series, labels, units, bounds, legends, and metadata once; backend renderers should only decide how to draw/export them.
- [ ] Keep Matplotlib as the publication-quality static export backend while adding an interactive backend path for fast pan/zoom/hover inspection of selected large-data products and time windows.
- [ ] Prototype Plotly as the first interactive backend, then evaluate whether Bokeh/HoloViews/Datashader is needed for very large campaigns; avoid duplicating every plot by routing both static and interactive rendering through the shared plot-spec layer.
- [ ] Provide both named domain plot functions for common workflows, such as `plot_position_ned` and `plot_gyro_bias_body`, and a generic quick-XY plotting utility for ad hoc inspection of arbitrary bundle/CSV fields.
- [ ] Benchmark raw CSV reload/regeneration against packaged-bundle reload/regeneration for representative single-run, 100-run, and 500-run campaigns.
- [ ] Keep direct C++ HDF5 logging out of scope for this pass; capture the longer-term direction as embedded-optimized binary logging plus Python conversion/repackaging into HDF5 or other analysis formats.

## Pass 6.5: Monte Carlo initialization support

- [ ] Connect Monte Carlo execution to the Phase 5 advanced analysis/restart initialization path for deterministic seeded initial estimate errors and mid-trajectory restart studies.
- [ ] Support deterministic seeded draws for initial estimate errors and simulator error terms without leaking simulator truth/error context into product-core code.
- [ ] Add a separate simulation/analysis-only Monte Carlo initial-estimate-error path. This path should support deterministic seeds, explicit error vectors, and covariance-colored random draws in the selected `StateDef::Error` ordering so analysis runs can start mid-trajectory with statistically controlled estimator errors.
- [ ] For Monte Carlo estimate-error initialization, distinguish between directly setting nominal estimates and sampling estimate errors relative to truth. For persistent estimated states such as IMU biases, the statistically consistent form is `estimated_state = true_state + sampled_estimate_error`, so the actual initial estimate error falls within the configured covariance.
- [ ] Introduce an explicit app/sim-side reference context before implementing truth-relative non-PVA initialization. A future `InitialEstimateReference`-style object should carry truth kinematics plus truth sensor error/calibration states, such as IMU turn-on/in-run bias, without letting app-support initialization reach into simulator internals ad hoc.

## Pass 6.6: Monte Carlo covariance matching and bias-analysis scenarios

- [ ] Add matched-covariance Monte Carlo scenarios where simulator truth-error distributions and filter initial covariance are intentionally aligned for consistency analysis.
- [ ] Add conservative-covariance scenarios where the filter initial covariance intentionally exceeds the actual simulated initial error distribution.
- [ ] Add gyro/accelerometer bias-specific Monte Carlo plots and report metrics that make initial bias-estimate covariance mismatch easy to diagnose.
- [ ] Revisit default gyro and accelerometer bias initial covariance values after matched/non-matched scenarios exist, rather than tuning from a single conservative run.

## Future phase details

Future backlog unrelated to the current Phase 6 Monte Carlo scope lives in dedicated detail files:

- [`phase_6_monte_carlo.md`](roadmap_details/phase_6_monte_carlo.md): current Phase 6 completed-pass history and follow-forward detail.
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
