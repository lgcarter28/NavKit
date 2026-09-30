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
- `phase_10_sensor_model_cleanup.md` - flight-relevant sensor cleanup and validation for loosely coupled GNSS, IMU/magnetometer, barometer, pitot/air data, and available legacy hardware.
- `phase_11_latent_measurement_handling.md` - timestamped events, latency, bounded history, replay, and smoothing foundations.
- `phase_12_hardware_flight_mvp.md` - Linux/WSL/container parity, embedded smoke and resource evidence, replay, capstone-hardware bench integration, HWIL, budget prototype trade study, and a graduated flight demonstration.
- `phase_13_transfer_alignment_stationary_modes.md` - transfer alignment, coarse/fine alignment, and ZUPTs after buffering support.
- `phase_14_tightly_coupled_gnss.md` - tightly coupled GNSS, raw observables, constellations, receiver adapters, and integrity seams after the hardware MVP.
- `phase_15_embedded_hardening.md` - profiling, resource qualification, remaining embedded readiness, type/API hygiene, documentation, telemetry, and release workflow.
- `phase_16_advanced_algorithms.md` - advanced GNSS techniques, vision/LiDAR/SLAM, celestial/radar/external aiding, GPS-denied demonstrations, and robust/multi-hypothesis algorithms.
- `phase_17_additional_mechanizations_environments.md` - additional mechanizations, environments, and physical models.
- `phase_18_guidance_control_vehicle_dynamics.md` - guidance/control signal flow, controlled-attitude point-mass models, and future rigid-body vehicle dynamics.
- `phase_19_alternative_estimators.md` - sliding-window, factor-graph, and smoothing backends.
- `phase_20_simulation_platform.md` - HIL, multi-vehicle simulation, production scenario management, trajectory analysis, and qualification reports.
