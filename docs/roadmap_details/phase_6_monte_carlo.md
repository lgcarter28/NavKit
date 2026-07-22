# Phase 6 - Monte Carlo and Batch Analysis

**Status:** completed pass history. Current active Phase 6 pass ownership lives in `docs/ROADMAP.md`; this file preserves completed pass detail after each pass is fully complete.

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
- [x] Each Monte Carlo axis subplot shows faint individual run errors, bold ensemble mean error, empirical ensemble `+/-3 sigma` bounds about the ensemble mean, and mean reported filter `+/-3 sigma` bounds about zero.
- [x] Added a small ECEF INS/GNSS Monte Carlo smoke config under `config/runtime/monte_carlo`.
- [x] Documented the campaign schema, seed derivation behavior, output layout, replay workflow, and first-pass plot interpretation.
- [x] Validated the pass with a three-run Release smoke campaign and inspected the generated seed maps, manifests, effective configs, aggregate covariance figures, and plotting elapsed time.

## Pass 6.2: aggregate outputs and reports

- [x] Added campaign-level Monte Carlo aggregate reports under `summary/reports/`.
- [x] Aggregated per-axis RMSE, final RMSE, final mean error, empirical `+/-3 sigma`, mean filter `+/-3 sigma`, empirical/filter sigma ratio, and filter/empirical coverage.
- [x] Aggregated state-family NEES summaries for ECEF position, ECEF velocity, ECEF attitude, body gyro bias, and body accelerometer bias.
- [x] Aggregated GNSS position/velocity NIS summaries when measurement-statistics logs are enabled.
- [x] Added run timing and output-size/resource summaries across campaign runs.
- [x] Extended campaign manifests with plot/report/analysis timing and paths to generated report artifacts.
- [x] Added a lightweight `tools/compare_monte_carlo.py` utility that builds comparison CSV/Markdown tables from existing campaign report folders without re-reading raw run logs.
- [x] Enabled low-rate measurement-statistics logging in the runtime-covariance scenario used for Pass 6.2 validation so GNSS NIS metrics are populated.
- [x] Documented the aggregate report layout, report interpretation, and comparison workflow.
- [x] Validated the pass with a 100-run Release campaign from `config/runtime/monte_carlo/ecef_ins_gnss_runtime_covariance.json`; all 100 runs passed, aggregate figures/reports were generated, and the comparison utility was smoke-tested on the generated report.
