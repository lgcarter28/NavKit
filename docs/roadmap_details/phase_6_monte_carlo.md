# Phase 6 - Monte Carlo and Batch Analysis

**Status:** active phase detail. Current active pass ownership remains in `docs/ROADMAP.md`; this file preserves completed pass history and follow-forward context for later Phase 6 passes.

Monte Carlo support comes immediately after Phase 5 because it is the gold-standard workflow for navigation analysis and will turn the current single-run scenario tooling into repeatable statistical evidence.

## Pass 6.1: Monte Carlo campaign runner and first aggregate covariance analysis

- [x] Added a Monte Carlo campaign runtime JSON schema with `campaign_name`, linked `nominal_config`, `runs.count`, optional `runs.start_index`, `randomization.master_seed`, `randomization.seed_policy`, `execution.build_type`, `execution.parallel_jobs`, and `output.root`.
- [x] Defaulted the first supported seed policy to `derive_all` and rejected unsupported policies with clear diagnostics.
- [x] Added `tools/run_monte_carlo.py` as the one-line campaign entry point with CLI overrides for build type and output root.
- [x] Reused the existing runtime component-linking loader and normal `run_sim.py` execution path so Monte Carlo runs remain ordinary replayable simulation runs.
- [x] Resolved the nominal runtime scenario into an effective JSON object, recursively discovered all `seed` fields by JSON-pointer path, and derived deterministic per-run/per-path seeds from the campaign master seed.
- [x] Wrote campaign and per-run manifests containing schema metadata, run indices, derived seed maps, output directories, subprocess status, and timing.
- [x] Generated replayable per-run `input.effective.json` files with run-specific names, output directories, and derived seeds.
- [x] Added process-level parallel campaign execution while preserving deterministic per-run seed derivation and isolated output folders.
- [x] Added reusable Python Monte Carlo aggregation helpers with a narrow loader that reads only the logs needed by aggregate covariance plots while preserving the same truth-error, NED conversion, scaling, and covariance conventions as the single-run analysis.
- [x] Added a lean Monte Carlo runtime scenario with inline run-level logging that keeps low-rate truth, navigation estimate, and IMU nominal logs while disabling high-volume debug/correction/statistics outputs.
- [x] Added aggregate-analysis controls for plot time-grid decimation plus CLI overrides for run count, start index, parallel jobs, and maximum plot points.
- [x] Added first-pass campaign-level Monte Carlo error/covariance figures for ECEF position, velocity, attitude, body gyro bias, body accelerometer bias, plus NED position, velocity, and attitude.
- [x] Kept Monte Carlo covariance figures broken out by quantity and frame, with one subplot per axis instead of dashboard/RGB roll-up plots.
- [x] Each Monte Carlo axis subplot shows faint individual run errors, bold ensemble mean error, empirical ensemble `+/-3 sigma` bounds about zero, and mean reported filter `+/-3 sigma` bounds about zero.
- [x] Added a small ECEF INS/GNSS Monte Carlo smoke config under `config/runtime/monte_carlo`.
- [x] Documented the campaign schema, seed derivation behavior, output layout, replay workflow, and first-pass plot interpretation.
- [x] Validated the pass with a three-run Release smoke campaign and inspected the generated seed maps, manifests, effective configs, aggregate covariance figures, and plotting elapsed time.

## Pass 6.2: aggregate outputs and reports

- [ ] Aggregate RMSE, NEES, NIS, failure counts, and runtime.
- [ ] Generate comparison tables and reports across configurations.
- [ ] Extend timing/resource artifacts into Monte Carlo aggregate reports.
- [ ] Add runtime and memory summaries appropriate for simulator qualification runs.

## Pass 6.3: Monte Carlo initialization support

- [ ] Connect Monte Carlo execution to the Phase 5 advanced analysis/restart initialization path once that path exists.
- [ ] Support deterministic seeded draws for initial estimate errors and simulator error terms without leaking simulator truth/error context into product-core code.
- [ ] Add a separate simulation/analysis-only Monte Carlo initial-estimate-error path. This path should support deterministic seeds, explicit error vectors, and covariance-colored random draws in the selected `StateDef::Error` ordering so analysis runs can start mid-trajectory with statistically controlled estimator errors.
- [ ] For Monte Carlo estimate-error initialization, distinguish between directly setting nominal estimates and sampling estimate errors relative to truth. For persistent estimated states such as IMU biases, the statistically consistent form is `estimated_state = true_state + sampled_estimate_error`, so the actual initial estimate error falls within the configured covariance.
- [ ] Introduce an explicit app/sim-side reference context before implementing truth-relative non-PVA initialization. A future `InitialEstimateReference`-style object should carry truth kinematics plus truth sensor error/calibration states, such as IMU turn-on/in-run bias, without letting app-support initialization reach into simulator internals ad hoc.

## Pass 6.4: schema versioning and compatibility

- [ ] Add explicit schema/version metadata for Monte Carlo run manifests, aggregate reports, and scenario-expanded runtime inputs.
- [ ] Define compatibility expectations for config schemas, log schemas, plot inputs, and aggregate report formats so older analysis artifacts can either be read deliberately or rejected with clear diagnostics.
- [ ] Document the migration/update policy for generated logs and reports before they become qualification evidence.
