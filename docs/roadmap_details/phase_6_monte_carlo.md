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

## Pass 6.3: schema versioning and compatibility

- [x] Added central major-version compatibility validation for Monte Carlo campaign manifests, per-run manifests, aggregate reports, packed analysis bundles, and renderer-neutral plot specifications.
- [x] Made incompatible schema families/major versions fail with explicit diagnostics; legacy raw CSV remains a deliberate, documented unversioned input path that is recorded when packaged.
- [x] Kept scenario-expanded input JSON associated with its versioned Monte Carlo run manifest instead of adding a second runtime-parser-specific schema contract.
- [x] Recorded source runtime inputs, simulation manifests, seeds, log schemas, units, frames, decimation, and truth/NED/covariance derivation assumptions in bundle metadata when available.
- [x] Documented the compatibility and migration policy in `docs/ANALYSIS.md`: generated campaign/report/bundle artifacts require compatible schemas, while historical CSV can be packaged intentionally.

## Pass 6.4: analysis bundle and interactive plotting infrastructure

- [x] Preserved CSV/JSON as the portable raw desktop-simulation artifact and added optional offline HDF5 packaging without changing C++ logging.
- [x] Added `tools/package_analysis.py` to package one run or an entire campaign into `navkit.analysis_bundle.v1` and enabled automatic packaging for the supplied Monte Carlo configs.
- [x] Implemented reusable HDF5 layout with `/runs/<run_id>/data`, `/runs/<run_id>/derived`, and `/aggregate` groups. Per-run raw tables and truth-aligned errors are stored alongside campaign metadata.
- [x] Cached Monte Carlo plot-ready time/error/covariance arrays, empirical covariance, and GNSS NIS plus state-family NEES samples in campaign bundles.
- [x] Added shared CSV/HDF5 run loading, including matching time-window behavior for either source, so existing single-run static analysis can consume a packed bundle.
- [x] Added shared `PlotSpec`/`PlotAxis`/`PlotTrace` preparation objects; Monte Carlo domain builders use them once while Matplotlib and Plotly renderers remain backend-specific drawing layers.
- [x] Kept Matplotlib for publication-quality PNGs and added Plotly `Scattergl` HTML output for targeted interactive pan/zoom/hover inspection. `tools/plot_analysis.py` provides generic CSV/HDF5 quick-XY inspection.
- [x] Refined Plotly Monte Carlo figures to keep individual histories visible, use one shared Unicode legend, and toggle the otherwise-disabled unified hover panel on demand.
- [x] Made HDF5 bundle packaging and the full Plotly HTML aggregate-figure set the default campaign analysis artifacts; `analysis.renderer: "matplotlib"` remains the explicit static-PNG option.
- [x] Added `docs/ANALYSIS.md`, updated setup/documentation maps, and kept direct C++ HDF5 logging explicitly out of scope in favor of future embedded binary telemetry plus Python conversion/repackaging.
- [x] Validated a 500-run and a 1,000-run Release campaign from `config/runtime/monte_carlo/ecef_ins_gnss_runtime_covariance.json`; all requested run bundles, aggregate reports, static plots, and interactive plots completed successfully.
- [x] Measured representative 1,000-run NED-position regeneration: raw CSV plus Matplotlib took 213.198 s; cached HDF5 plus Matplotlib took 10.275 s; cached HDF5 plus Plotly HTML took 16.868 s. The 1,000-run full-fidelity bundle was 2.32 GB and required 1,319.129 s to package, making raw-table retention the clear future optimization seam.
