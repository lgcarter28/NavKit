# Phase 6 - Monte Carlo and Batch Analysis

**Status:** completed. Current active pass ownership lives in `docs/ROADMAP.md`; completed pass detail is preserved here after verification.

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
- [x] Added a lightweight internal comparison utility that builds comparison CSV/Markdown tables from existing campaign report folders without re-reading raw run logs.
- [x] Enabled low-rate measurement-statistics logging in the runtime-covariance scenario used for Pass 6.2 validation so GNSS NIS metrics are populated.
- [x] Documented the aggregate report layout, report interpretation, and comparison workflow.
- [x] Validated the pass with a 100-run Release campaign from the current runtime-covariance-override campaign; all 100 runs passed, aggregate figures/reports were generated, and the comparison utility was smoke-tested on the generated report.

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
- [x] Kept Matplotlib for publication-quality PNGs and added Plotly `Scattergl` HTML output for targeted interactive pan/zoom/hover inspection. `tools/plot_field.py` provides generic CSV/HDF5 quick-XY inspection.
- [x] Refined Plotly Monte Carlo figures to keep individual histories visible, use one shared Unicode legend, and toggle the otherwise-disabled unified hover panel on demand.
- [x] Made HDF5 bundle packaging and the full Plotly HTML aggregate-figure set the default campaign analysis artifacts; `analysis.renderer: "matplotlib"` remains the explicit static-PNG option.
- [x] Added `docs/ANALYSIS.md`, updated setup/documentation maps, and kept direct C++ HDF5 logging explicitly out of scope in favor of future embedded binary telemetry plus Python conversion/repackaging.
- [x] Validated a 500-run and a 1,000-run Release campaign from the current runtime-covariance-override campaign; all requested run bundles, aggregate reports, static plots, and interactive plots completed successfully.
- [x] Measured representative 1,000-run NED-position regeneration: raw CSV plus Matplotlib took 213.198 s; cached HDF5 plus Matplotlib took 10.275 s; cached HDF5 plus Plotly HTML took 16.868 s. The 1,000-run full-fidelity bundle was 2.32 GB and required 1,319.129 s to package, making raw-table retention the clear future optimization seam.

## Pass 6.5: Monte Carlo consistency dashboards and statistical evidence

- [x] Extended campaign HDF5 bundles with cached time-indexed joint NEES histories for the full 15-state INS, PVA, position, velocity, attitude, combined IMU bias, gyro bias, and accelerometer bias; added position and velocity GNSS NIS histories as separate 3-DOF observation-family products.
- [x] Kept joint NEES/NIS as the primary consistency evidence, retaining each selected covariance submatrix and its cross-correlation terms. Preserved scalar 1/2/3-sigma coverage as an explicitly secondary marginal diagnostic.
- [x] Added interactive Plotly consistency dashboards with an ensemble-density heatmap, ensemble mean, statistically valid 95-percent mean chi-square bounds, persistent epoch markers, and browser-side snapshot export. Each selected epoch can toggle its right-side distributions between PDF, CDF, and QQ views without producing a redundant final-epoch page.
- [x] Added dedicated position, velocity, attitude, gyro-bias, and accelerometer-bias axis dashboards. Each contains X/Y/Z marginal normalized-squared-error (1-DOF) panels as a clearly labeled drill-down companion to, not replacement for, the cross-correlation-aware joint NEES products.
- [x] Made right-side epoch distributions follow heatmap hover by default. A click or typed epoch value pins the distribution; the same **Follow heatmap** control resumes hover-following.
- [x] Added focused full-INS, navigation, IMU-bias, and GNSS-observation dashboards. Their PDF/CDF/QQ material deliberately uses only one selected epoch per run and never treats time-pooled samples as independent hypothesis-test evidence.
- [x] Added machine-readable JSON/CSV and Markdown reports for final and steady-state mean NEES/NIS, normalized consistency status, joint 1/2/3-sigma ellipsoidal coverage, scalar coverage, and GNSS acceptance rate.
- [x] Preserved NIS by observation family and measurement dimension; no heterogeneous whole-filter NIS is synthesized.
- [x] Added matching empirical-CDF heatmaps in raw NEES/NIS space and normalized PIT-space CDF-residual heatmaps for every joint, observation-family, and marginal dashboard. Preserved the selected-epoch PDF/CDF/QQ drill-downs and organized all consistency HTML under diagnostic-family subdirectories with a generated index.
- [x] Validated directly against the existing 1,000-run HDF5 campaign without re-running the simulation. The original consistency cache refresh took 133.490 s; after adding all three diagnostic modes, representative cached dashboard/report regeneration took between 58.712 s and 68.418 s and produced 8 NEES groups, 2 NIS groups, 15 marginal groups, 27 interactive dashboards, one generated dashboard index, and four report artifacts.

## Pass 6.6: pointwise statistical uncertainty for CDF residuals

- [x] Preserved the existing raw probability-space CDF-residual dashboards and added a distinct uncertainty-normalized diagnostic family.
- [x] Normalized each pointwise empirical-CDF residual by its finite-ensemble binomial standard error, leaving the zero-variance probability endpoints undefined instead of inventing a numerical fallback.
- [x] Added explicit mathematical interpretation and limitations: the result is pointwise evidence measured in standard-error units, not a simultaneous confidence band over probability and time.
- [x] Positioned consistency-dashboard colorbars from the actual Plotly subplot domains so they remain centered in the inter-column gap rather than overlapping the distribution panels.
- [x] Verified the new grid numerically with synthetic samples and generated a structural HTML smoke dashboard without changing the existing raw residual products.

## Pass 6.7: Monte Carlo covariance matching and bias-analysis scenarios

- [x] Added matched-covariance Monte Carlo scenarios where HG1700 simulator turn-on-bias distributions and filter initial bias covariance are intentionally aligned.
- [x] Added conservative-covariance scenarios where the filter initial bias variance is ten times the actual simulated turn-on-bias variance while all other scenario components remain fixed.
- [x] Added first-epoch gyro/accelerometer bias metrics for empirical sigma, filter sigma, their ratio, mean error, RMSE, and filter three-sigma coverage to JSON, CSV, Markdown, and cross-campaign comparison outputs.
- [x] Retained the dedicated body gyro-bias and accelerometer-bias Monte Carlo error/covariance plots as the visual companion to the new numerical matching metrics.
- [x] Revisited the default bias covariance: the compile-time default now agrees with the moderate conservative runtime default instead of retaining the formerly oversized gyro-bias variance.
- [x] Validated both controlled scenarios with 100-run Release campaigns. Matched empirical/filter sigma ratios were 0.97--1.06 for gyro and 0.99--1.11 for accelerometer; conservative ratios were 0.31--0.35 across all six axes, agreeing with the designed value of `1 / sqrt(10)`.

## Pass 6.8: scenario taxonomy and Monte Carlo initialization support

- [x] Moved runnable `navkit_sim` compositions into `config/runtime/navkit_sim/scenario/`, retaining reusable physical, estimator, and initialization fragments under role-keyed `components/`.
- [x] Standardized scenario names as `<product>_<trajectory>_<purpose>.json`, with matching `run_name`, default `output/logs/<run_name>`, and component links relative to the scenario file.
- [x] Renamed Monte Carlo campaigns to the matching scenario stem plus `_mc`, updated tools/docs/examples/defaults, and removed stale flat scenario paths.
- [x] Documented the runtime/compile-time config taxonomy in `docs/CONFIGURATION.md` and added a concise normative pointer from `docs/NAMING_CONVENTIONS.md`.
- [x] Made single-scenario execution resolve component links into an explicit replayable `effective_runtime_config.json`; direct `navkit_sim` invocation now clearly requires that resolved input.
- [x] Added a simulation/analysis-only `InitialTruthReference` carrying initial truth PVA plus realized sensor calibration/error truth, keeping simulator truth/error context out of `navkit::core`.
- [x] Added `filter_initialization.initial_estimate_error` support for explicit vectors and deterministic covariance-colored `random_error` draws in the selected `StateDef::Error` ordering; JSON covariance is validated as symmetric positive semidefinite.
- [x] Applied estimate errors with `estimated_state = true_state + sampled_estimate_error`, including persistent IMU bias estimates, and rejected this truth-relative feature from startup paths without an explicit simulation reference.
- [x] Added a restart Monte Carlo scenario/campaign and validated component resolution, the Release single-run restart path, and a three-run deterministic Release campaign. The campaign derived independent seeds for IMU, GNSS, and initial-estimate-error draws; all three runs and generated aggregate artifacts succeeded.

## Pass 6.9: analysis performance characterization and low-risk scaling

- [x] Added `summary/analysis_performance.json` with campaign scale, raw/HDF5 storage, retained-table mode, portable process-memory observations, simulation totals, and per-stage aggregate, report, consistency, and package timing evidence.
- [x] Preserved lean Monte Carlo source loading, made package/aggregate/dashboard selection explicit, and added `full` versus `derived_only` HDF5 retention with explicit numeric chunking plus `lzf`, `gzip`, or uncompressed storage.
- [x] Added deterministic source-manifest and settings fingerprints for package reuse, aggregate-figure reuse, and consistency-dashboard reuse; package metadata records the effective storage and derivation settings.
- [x] Added opt-in `analysis.parallel_jobs` for independent post-bundle Plotly aggregate figures and consistency dashboards. HDF5 writes, aggregate reduction, manifests, and reports remain single-owner/serial.
- [x] Added an internal analysis-equivalence verifier, which renders selected aggregate plots serially and in parallel, normalizes Plotly's intentional random div ID, and requires matching rendered content. The three-run Release smoke campaign and this equivalence check both passed.
- [x] Reorganized tooling into stable public commands at `tools/`, explicit profiling diagnostics under `tools/profile/`, and repository-only support/verification helpers under `tools/internal/`.
- [x] Added a controlled HDF5-backed 500/1,000-run Plotly scaling benchmark. It creates benchmark-owned copied/warmed bundles to avoid source mutation and Windows HDF5 locks, then measures aggregate and forced consistency-dashboard rendering independently at one, two, and four workers.
- [x] Added `plot_consistency.py --force` so renderer changes can regenerate dashboard/report artifacts without unnecessarily recomputing the HDF5 statistical cache.
- [x] Measured complete warmed repeated-analysis workloads: 500 runs completed in 91.925 s, 85.378 s, and 81.308 s at one, two, and four workers; 1,000 runs completed in 139.339 s, 133.100 s, and 115.942 s. Four workers was the best tested setting, improving the full workload by 11.5% and 16.8% over serial respectively; one-time consistency-cache construction remained distinct at 104.133 s and 209.189 s.

## Pass 6.10: cache-preparation optimization and CDF reference cues

- [x] Added a slim expected-reference strip to each interactive empirical-CDF heatmap. It renders the ideal `F(p) = p` vertical slice beside the measured heatmap and is intentionally absent from CDF-residual products.
- [x] Profiled named cache-preparation stages: truth-error loading, NEES/marginal derivation, NIS loading/derivation, and the single-owner HDF5 write.
- [x] Reused campaign-wide ECEF-to-NED transforms when runs share the same truth/time grid, reused selected time grids, and derived NEES plus marginal series from one truth-error frame scan without changing numerical products.
- [x] Made consistency-cache materialization demand-driven: package creation retains raw/derived data and aggregate series, while selected NEES, NIS, and marginal cache families are materialized only by the prepare or consistency-plot workflow that needs them.
- [x] Tuned derived HDF5 writes with explicit bounded chunk shapes and LZF compression while retaining deterministic, serial ownership of bundle writes.
- [x] Added `tools/prepare_analysis.py` for explicit cache-only preparation from either a CSV run/campaign directory or an existing bundle, including selected series families and named timing evidence.
- [x] Re-measured cold preparation and warmed rendering. Full cache preparation fell from 104.133 s to 40.484 s for 500 runs (61.1% faster) and from 209.189 s to 75.145 s for 1,000 runs (64.1% faster). Warmed four-worker consistency rendering remained 36.393 s for 500 and 53.684 s for 1,000 runs; HDF5 source reading is now the dominant remaining cold-cache cost, so no brittle cache-construction concurrency was added.
