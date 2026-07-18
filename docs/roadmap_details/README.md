# Roadmap Details

This folder preserves the detailed roadmap history and expanded phase notes that used to live in `docs/ROADMAP.md`.

Use `docs/ROADMAP.md` as the active roadmap. It is intentionally short and should contain the current verified baseline, active passes, and grouped future backlog. Use these detail files when you need design memory, phase history, or the original expanded checklist context.

## Files

- `overview_and_reconciliation.md` - original roadmap introduction, reconciliation notes, and verified baseline snapshot.
- `phase_0_provenance.md` - owner/provenance safeguards.
- `phase_1_baseline.md` - baseline integrity and documentation alignment.
- `phase_2_estimator_policies.md` - estimator policy boundaries.
- `phase_3_config_logging_profiling.md` - configuration, logging, compiler/tooling, tests, and profiling detail.
- `phase_4_navigator_seam.md` - original Navigator/propagation seam work.
- `phase_5_ecef_ins_gnss.md` - ECEF INS/GNSS algorithm, IMU simulator, Navigator implementation, logging, plotting, and runtime-config history.
- `phase_6_monte_carlo.md` - Monte Carlo and batch-analysis support.
- `phase_7_trajectory_provider.md` - trajectory provider, timebase, scenario, and reusable navigation-math expansion.
- `phase_8_estimator_validation.md` - estimator validation, consistency metrics, and repeatable reports.
- `phase_9_status_error_handling.md` - robust status/error handling before the later high-complexity phases.
- `phase_10_sensor_model_cleanup.md` - loosely coupled GNSS cleanup, altimeter/pressure models, pitot tube, magnetometer aiding, and sensor scheduling.
- `phase_11_tightly_coupled_gnss.md` - tightly coupled GNSS, raw observables, constellations, receiver adapters, and integrity seams.
- `phase_12_latent_measurement_handling.md` - latent measurement context, buffering, replay, and smoothing foundations.
- `phase_13_transfer_alignment_stationary_modes.md` - transfer alignment, coarse/fine alignment, and ZUPTs after buffering support.
- `phase_14_profiling_resource_validation.md` - profiling, resource, allocation, and target evidence.
- `phase_15_embedded_hardening.md` - remaining embedded readiness, type/API hygiene, documentation, and CI/release workflow.
- `phase_16_advanced_algorithms.md` - advanced GNSS techniques, vision/LiDAR/SLAM, celestial/radar/external aiding, GPS-denied demonstrations, and robust/multi-hypothesis algorithms.
- `phase_17_additional_mechanizations_environments.md` - additional mechanizations, environments, and physical models.
- `phase_18_alternative_estimators.md` - sliding-window, factor-graph, and smoothing backends.
- `phase_19_simulation_platform.md` - HIL, multi-vehicle simulation, production scenario management, and qualification reports.
