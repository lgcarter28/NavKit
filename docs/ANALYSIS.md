# NavKit Offline Analysis Artifacts

NavKit desktop simulation writes CSV/JSON artifacts first. They remain the
portable, human-inspectable raw output of a run. Python analysis can consume
those folders directly or package them into a versioned HDF5 analysis bundle for
faster repeated aggregate analysis and interactive inspection.

## Schema compatibility

Versioned analysis artifacts use names of the form `navkit.<artifact>.v<major>`.
Readers require the same artifact family and major version; a mismatch is a
deliberate error rather than a best-effort parse.

| Artifact | Current schema |
| --- | --- |
| Monte Carlo campaign config/manifest | `navkit.monte_carlo_campaign.v2` |
| Per-run Monte Carlo manifest | `navkit.monte_carlo_run.v1` |
| Aggregate Monte Carlo report | `navkit.monte_carlo_report.v1` |
| HDF5 analysis bundle | `navkit.analysis_bundle.v1` |
| Renderer-neutral plot specification | `navkit.plot_spec.v1` |

Existing raw CSV logs intentionally remain unversioned legacy inputs. They can
be packaged through the explicit CSV compatibility path, which records their
source paths and derivation assumptions in the resulting bundle. New generated
campaign and report JSON must declare their schemas.

## HDF5 bundle layout

The first bundle format is intentionally simple:

```text
analysis_bundle.h5
  attributes: schema, metadata JSON
  /runs/<run_id>/metadata
  /runs/<run_id>/data/<table>/<column>
  /runs/<run_id>/derived/<table>/<column>
  /aggregate/monte_carlo_series/<quantity>/<array>
  /aggregate/consistency/series/<nees|nis|marginal>/<group>/<array>  (when prepared)
```

`data` contains the raw analysis tables needed by current plotting. `derived`
contains cached calculations such as truth-aligned navigation error. Campaign
bundles additionally cache downsampled aggregate Monte Carlo series. Bundle
metadata records source runtime configs/manifests when available, seeds,
compile-time metadata carried by the source logs, units/frame conventions, plot
decimation, and the truth/NED/covariance derivation assumptions.

The prepare workflow materializes time-indexed joint consistency histories on
demand. The NEES
cache contains full INS (15 DOF), PVA (9 DOF), position, velocity, attitude,
combined IMU bias, gyro bias, and accelerometer bias products. Each product is
calculated from the selected full covariance submatrix, preserving relevant
cross-correlation terms. NIS remains separate by observation family and
dimension (currently GNSS position and velocity, each 3 DOF); NavKit does not
form a meaningless heterogeneous "whole-filter NIS."

The bundle also caches marginal per-axis normalized squared-error histories for
position, velocity, attitude, gyro bias, and accelerometer bias. Those are
explicit 1-DOF drill-down diagnostics; they intentionally do not replace the
joint NEES products, which retain covariance cross terms.

HDF5 is an offline analysis artifact, not an embedded logging format. Future
targets should emit an embedded-appropriate NavKit binary stream and use Python
to decode/repackage it into this analysis layer.

## Choosing the right tool

The analysis tools form layers. Use the highest-level command that matches the
job:

| Tool | Use it when |
| --- | --- |
| `run_scenario.py` | Run one scenario and immediately generate its standard analysis |
| `run_sim.py` | Run only the C++ simulation |
| `plot_run.py` | Render the standard domain-aware suite for an existing single run |
| `plot_trajectory.py` | Render available frame-explicit truth, Guidance, Autopilot/Vehicle, tracking-error, and 3-D trajectory dashboards |
| `run_monte_carlo.py` | Execute, package, report, and plot a seeded campaign |
| `plot_monte_carlo.py` | Regenerate selected aggregate campaign figures |
| `plot_consistency.py` | Generate or refresh joint NEES/NIS dashboards and reports |
| `package_analysis.py` | Convert existing CSV run/campaign output into HDF5 |
| `prepare_analysis.py` | Build selected reusable HDF5 consistency-cache families without rendering figures |
| `plot_field.py` | Quickly inspect arbitrary named CSV/HDF5 fields |
| `profile/benchmark_analysis_scaling.py` | Measure identical HDF5-backed plotting workloads at multiple Plotly worker counts |

`run_scenario.py` and `run_monte_carlo.py` are the normal entry points.
Lower-level tools are useful when simulation is already complete or when
iterating on analysis without paying simulation cost again.

## Single-run workflow

Run the default scenario and its standard analysis:

```powershell
python tools/run_scenario.py --build-type Debug
```

For realistic-duration simulations, use Release and select both the runtime
input and output location explicitly:

```powershell
python tools/run_scenario.py --build-type Release `
  --config config/runtime/navkit_sim/scenario/ecef_ins_gnss_lc_gyro_accel_bias_stationary_covariance_override.json `
  --output-dir output/logs/my_case
```

`run_scenario.py` resolves a component-linked scenario into a self-contained
`effective_runtime_config.json`, executes the ordinary simulator, then invokes
`plot_run.py` and `plot_trajectory.py`. The trajectory tool exits cleanly when
the scenario did not enable any trajectory-inspection products. The resolved
file makes every run replayable and is the input required when invoking the
simulator executable directly. Add `--no-plot` to retain the one-command
scenario setup while skipping post-processing.

Use the lower-level runner when no analysis should run:

```powershell
python tools/run_sim.py --build-type Release `
  --config config/runtime/navkit_sim/scenario/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal.json
```

Analyze existing logs without rerunning the simulation:

```powershell
python tools/plot_run.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal
python tools/plot_run.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal `
  --plot state_ned --start-time 10 --show
python tools/plot_trajectory.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_ballistic_nominal `
  --renderer plotly --show
```

Trajectory inspection remains a deliberately lightweight CSV/Plotly workflow.
When enabled, the simulator can emit independent
`trajectory_kinematics_ecef`, `trajectory_kinematics_eci`,
`trajectory_kinematics_ned`, `trajectory_kinematics_body`, and
the split `trajectory_guidance` and `trajectory_autopilot_vehicle` products.
The kinematic products make the same truth state inspectable in complementary
frames. The focused Guidance figure compares inertial-acceleration command and
response in NED, the same command/response resolved in body, and
commanded/realized NED bank. The focused Autopilot figure compares
body-to-NED roll/pitch/yaw command and response plus body inertial angular-rate
command/response with explicit `p`, `q`, and `r` legends. The actual-IMU moving
average, deterministic feedforward, Vehicle-internal response stages, and
detailed saturation flags remain available in CSV diagnostics without
crowding the default plots.

The body dashboard places the ECI-facing plant and ideal-IMU physics beside
their rotating-Earth counterparts. It shows `v_ib_b`, `a_ib_b`, `v_eb_b`,
`a_eb_b`, `specific_force_ib_b`, and `w_ib_b` (the code form of
\(\omega_{ib}^{b}\)) as six explicitly titled panels. Here
\(\omega_{ib}^{b}\) is the angular rate of body with respect to ECI, resolved
in body coordinates; its plotted components are the body `p`, `q`, and `r`
rates. The paired ECI/ECEF velocity and acceleration panels make the
Earth-rotation contribution visible instead of silently mixing the two
contracts. Body tracking errors remain command minus final response and are
not hidden as implicit differences between traces.

Guidance and Autopilot command traces are zero-order held between their exact
producer epochs. When the trajectory log cadence is faster than either
controller, repeated stepwise command samples are intentional; they are not
interpolated commands or evidence that the controller ran at the logging
rate. Vehicle-response and plant-state traces remain continuous products.

The standard interactive suite also includes 3-D LLA and relative-local
position views. `trajectory_position_relative_ned_3d.html` uses the initial
local NED frame for its transformation, but labels and displays the axes as
North, East, and Up, where `Up = -Down`. Its 3-D aspect follows the relative
data extents so a shallow trajectory is not visually stretched into a large
vertical maneuver. Quaternion fields remain available for canonical
state/debug work, but Euler angles are the default attitude visualization
because their named reference frame is directly interpretable.

When the matching products exist, `plot_trajectory.py` writes:

```text
trajectory_kinematics_ecef.html
trajectory_kinematics_eci.html
trajectory_kinematics_ned.html
trajectory_kinematics_body.html
trajectory_position_lla_3d.html
trajectory_position_relative_ned_3d.html
trajectory_guidance.html
trajectory_autopilot_response.html
trajectory_guidance_control.html
trajectory_tracking_error.html
```

The 3-D figures include available Guidance phase transitions and waypoint
markers. `trajectory_guidance_control.html` aligns Guidance inertial
acceleration command/response in NED and body, the raw and permanently
filtered body-X/Y/Z specific-force command, body-to-NED roll/pitch/yaw
command/response, and Autopilot body `p/q/r` command/response on one shared
time axis. The Guidance filter traces expose the stateful interface between
Guidance and the downstream Autopilot/Vehicle consumers; zero time constant
is an exact per-channel bypass. The focused Guidance and Autopilot figures
present the remaining boundaries without duplicating lower-level Vehicle-only
or nested-loop dashboards. Legends use LLA, NED/NEU, XYZ, roll/pitch/yaw, and
`p/q/r` vocabulary according to the displayed quantity.
The HTML dashboards share NavKit's
renderer-neutral plot specifications, so Matplotlib PNG output remains
available with `--renderer matplotlib`, but Plotly is the default for fast
zooming and signal toggling.

The ECEF acceleration log is the rotating-frame derivative of ECEF-relative
velocity, `a_e = (d/dt)_e v_eb^e`, equivalently the second derivative of ECEF
position coordinates. The NED product resolves ECEF-relative velocity and
coordinate acceleration locally. The body product instead records inertial
`v_ib_b`, `a_ib_b`, and `specific_force_ib_b`, matching the ECI plant and ideal
IMU physics rather than rotating-frame ECEF body quantities.
Body-rate subscripts retain the full reference and resolution convention
(`w_ib_b`, `w_eb_b`, and `w_nb_b`). Each CSV has a matching metadata file that
records this convention and the selected constant-rate ECI/ECEF orientation
assumption. Earth orientation is evaluated from elapsed time relative to the
trajectory source epoch, even when that epoch is nonzero.

Logging is configured independently of simulator update rates in the runtime
scenario's `logging` object. An enabled log must provide `rate_hz` or `dt_s`;
disabled high-rate/debug products should use `"enabled": false`. This is
particularly important for long simulations and Monte Carlo campaigns, where
unnecessary CSV I/O can dominate runtime.

Typical single-run output is:

```text
output/logs/<run_name>/
  effective_runtime_config.json
  run_manifest.json
  timing.json
  data/
    *.csv
    *.meta.json
  figures/
    *.png
    trajectory_*.html
```

The desktop simulation has a centralized typed product list; the exact emitted
products depend on the runtime `logging` configuration.

## Monte Carlo workflow

Run a campaign from its JSON configuration:

```powershell
python tools/run_monte_carlo.py `
  config/runtime/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc.json `
  --build-type Release
```

Override common campaign controls without editing the JSON:

```powershell
python tools/run_monte_carlo.py `
  config/runtime/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc.json `
  --build-type Release `
  --run-count 100 `
  --output-root output/monte_carlo_scratch `
  --max-plot-points 1000
```

The campaign runner resolves the linked nominal scenario, derives deterministic
run-local seeds from the master seed, writes each replayable
`input.effective.json`, executes the normal simulation binary, builds aggregate
reports, packages HDF5, and generates interactive Plotly HTML by default.
Campaign scenarios should use lean logging rates and disable products not
needed by aggregate analysis.

Typical campaign output is:

```text
output/monte_carlo/<campaign_name>/
  campaign_config.effective.json
  campaign_manifest.json
  analysis_bundle.h5
  runs/
    run_000000/
      input.effective.json
      run_manifest.json
      data/
  summary/
    figures/
    consistency_figures/
    reports/
      monte_carlo_summary.json
      monte_carlo_report.md
      state_axis_metrics.csv
      state_group_metrics.csv
      bias_initialization_metrics.csv
      nis_metrics.csv
      run_timing.csv
      output_sizes.csv
```

Replay one campaign member by passing its effective input to `run_sim.py` or
`run_scenario.py`. Regenerate selected aggregates without rerunning:

```powershell
python tools/plot_monte_carlo.py `
  output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc `
  --plot attitude_ned --start-time 10 --show
```

Two supplied campaigns isolate initial-covariance matching:

```powershell
python tools/run_monte_carlo.py `
  config/runtime/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_covariance_matched_mc.json `
  --build-type Release

python tools/run_monte_carlo.py `
  config/runtime/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_covariance_conservative_mc.json `
  --build-type Release
```

Both campaigns use the same PVA-error, HG1700, process-noise, bias-dynamics,
and GNSS components. The matched campaign makes the filter's initial PVA and
IMU-bias covariance equal to the distributions that generate those initial
truth errors. The conservative campaign increases only the filter's initial
covariance. Compare their `bias_initialization_metrics.csv` files directly.

The bias-initialization report records, per body axis, the initial empirical
error sigma, initial mean filter sigma, their ratio, and initial filter
three-sigma coverage. A ratio near one is expected for a sufficiently large
matched campaign. A ratio below one is expected when filter covariance is
deliberately conservative. The existing gyro- and accelerometer-bias
error/covariance figures provide the time-history companion to these
initial-epoch metrics.

## Packaging and inspecting HDF5

Package an existing run or campaign:

```powershell
python tools/package_analysis.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal
python tools/package_analysis.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc
python tools/package_analysis.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc --bundle-mode derived_only --compression lzf
```

Prepare the expensive consistency cache once, without producing figures. This
accepts either a CSV run/campaign directory or an existing HDF5 bundle. Repeat
`--series-kind` to prepare only the family needed for the next investigation;
omit it to prepare the full NEES/NIS/marginal set.

```powershell
python tools/prepare_analysis.py <campaign>/analysis_bundle.h5
python tools/prepare_analysis.py <campaign>/analysis_bundle.h5 --series-kind nees
python tools/prepare_analysis.py <campaign> --bundle-mode derived_only --compression lzf
```

Monte Carlo campaigns package HDF5 and generate Plotly interactive aggregate
figures by default after their reports. Set `analysis.renderer` to `matplotlib`
when static PNG output is specifically needed. Package historical CSV output
manually when needed.

Use the normal static plotting path on either a CSV directory or a single-run
bundle:

```powershell
python tools/plot_run.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal
python -m navkit_analysis.plots output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal/data/analysis_bundle.h5
```

Regenerate a cached campaign aggregate from either raw CSV runs or the bundle:

```powershell
python tools/plot_monte_carlo.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc --plot attitude_ned
python tools/plot_monte_carlo.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc/analysis_bundle.h5 --plot attitude_ned --renderer plotly
```

Generate the joint NEES/NIS consistency dashboards and reports from an existing
campaign bundle. Use `--refresh-cache` only after changing the source run data
or when packaging an older bundle that lacks the cache:

```powershell
python tools/plot_consistency.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc/analysis_bundle.h5
python tools/plot_consistency.py output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc/analysis_bundle.h5 --refresh-cache --max-plot-points 1000
```

For quick exploration, select arbitrary named fields without creating a custom
script:

```powershell
python tools/plot_field.py output/logs/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal --table nav --y p_e_x_m --renderer plotly
```

Use HDF5 for repeated analysis of large campaigns. It avoids reopening hundreds
or thousands of CSV files and stores derived truth-aligned errors and
consistency histories alongside provenance metadata. Retain raw run folders
until the campaign has been validated; the bundle is a derived analysis cache,
not the embedded source log.

`full` bundles retain each selected raw log table and the derived tables needed
by current analysis. `derived_only` bundles retain the truth-aligned error
tables and aggregate covariance series without duplicating raw CSV tables.
Consistency families are materialized on demand by `prepare_analysis.py` or the
consistency renderer, so campaigns that only need aggregate error/covariance
plots do not pay the NEES/NIS cache cost. Numeric datasets are explicitly
chunked and use fast `lzf`
compression by default; `gzip` trades packaging/load time for a smaller bundle,
and `none` is useful only for controlled storage experiments. The bundle
metadata records the selected mode, compression, source-input manifest digest,
and packaging-stage timing evidence.

## Plotting architecture

Domain data preparation produces shared `PlotSpec`, `PlotAxis`, and `PlotTrace`
objects. They contain prepared series, labels, units, bounds, legend order, and
plot metadata but no renderer-specific calculations.

- Matplotlib consumes that prepared data for static PNG/report-quality output.
- Plotly consumes the same prepared data for responsive HTML pan/zoom/hover
  inspection.

Interactive Monte Carlo figures keep individual histories visible while opening
with the expensive unified hover panel disabled. Use the right-side **Toggle
hover details** control when a full time-slice readout is useful; the permanent
legend intentionally contains only the shared aggregate semantics.

Use Plotly for responsive browser pan, zoom, hover, epoch selection, and HTML
sharing. Use Matplotlib when producing static publication figures. Both
renderers consume the same prepared domain data; switching renderers does not
duplicate error, covariance, scaling, or frame-transform calculations.

## Cache refresh and performance

Normal plotting should reuse `analysis_bundle.h5`. Use `prepare_analysis.py`
to materialize a cache ahead of renderer iteration. Use `--refresh-cache` only
after source run data changes, consistency derivations change, or an older
bundle lacks the required cache:

```powershell
python tools/plot_consistency.py `
  output/monte_carlo/ecef_ins_gnss_lc_gyro_accel_bias_stationary_smoke_mc/analysis_bundle.h5 `
  --refresh-cache --max-plot-points 1000
```

`--max-plot-points` controls browser and renderer density; it does not change
the underlying statistical calculations. For large campaigns:

1. Build and execute simulation in Release.
2. Disable unnecessary runtime logs.
3. Package once.
4. Iterate on plots from HDF5.
5. Increase plot-point density only when the current diagnostic needs it.

The prepare command records named cache stages in the bundle itself and prints
them for quick diagnosis: truth-error frame loading, NEES/marginal derivation,
NIS loading/derivation, and the single-owner HDF5 cache write. It builds
truth-error products in one frame scan, reuses the selected campaign time grid,
and writes each requested family incrementally with explicit chunk shapes.
The Monte Carlo NED aggregate path likewise reuses a campaign transform when
runs share the same truth/time grid.

Timing for simulation, packaging, reporting, and plotting is printed by the
campaign workflow and retained in campaign artifacts where applicable.

Each Monte Carlo campaign additionally writes
`summary/analysis_performance.json`. It records run/sample scale, raw and HDF5
storage sizes, retained-table mode, optional portable process-memory observation,
and independent timing for loading, aggregate reduction, rendering, reporting,
and HDF5 packaging. Package and figure cache fingerprints include the relevant
source manifest, schema, bundle settings, selected artifacts, renderer, time
window, and decimation settings. Matching artifacts are safely reused unless a
tool receives `--force`. `plot_consistency.py --force` regenerates selected
dashboard/report artifacts while reusing the packed HDF5 consistency cache;
`--refresh-cache` is reserved for recomputing that cache from per-run data.

For large validated campaigns, first package once, then render only the desired
families from the bundle:

```powershell
python tools/plot_monte_carlo.py <campaign>/analysis_bundle.h5 `
  --renderer plotly --plot position_ned --parallel-jobs 2
python tools/plot_consistency.py <campaign>/analysis_bundle.h5 `
  --heatmap-mode cdf_probability_residual --parallel-jobs 2
```

Parallel jobs apply only to independent post-bundle Plotly rendering and default
to one. Their input data and output names are deterministic; HDF5 writes,
aggregate reduction, reports, and manifests remain serial. Use serial mode for
a baseline before comparing a parallel rendering run; the generated artifacts
must retain the same cache fingerprints and rendered Plotly data.

## Analysis scaling benchmark

Use the profiling benchmark to compare the same HDF5-backed aggregate and
consistency plotting workload with one, two, and four Plotly workers. It does
not rerun simulations or mutate source campaigns; it writes isolated output
under the selected benchmark root and records machine-readable wall-clock
results.

```powershell
python tools/profile/benchmark_analysis_scaling.py `
  --bundle phase_6_500=output/phase_6_500/ecef_ins_gnss_runtime_covariance_mc/analysis_bundle.h5 `
  --bundle phase_6_1000=output/phase_6_1000/ecef_ins_gnss_runtime_covariance_mc/analysis_bundle.h5 `
  --workers 1 --workers 2 --workers 4
```

The report is written to
`output/analysis_benchmarks/phase_6_9_scaling/analysis_scaling_benchmark.json`.
It records aggregate Plotly rendering, consistency-dashboard rendering, and
combined elapsed time for every campaign/worker pair. HDF5 reduction and
writing remain single-owner stages and are intentionally excluded.

The controlled Phase 6 baseline copied each source bundle, built its consistency
cache once, then rendered identical fresh outputs at each worker count. Cache
warmup is therefore reported separately from repeat-analysis performance:

| Campaign | 1 worker | 2 workers | 4 workers | Best repeated result |
| --- | ---: | ---: | ---: | --- |
| 500 runs | 91.925 s | 85.378 s | 81.308 s | 4 workers, 11.5% faster than serial |
| 1,000 runs | 139.339 s | 133.100 s | 115.942 s | 4 workers, 16.8% faster than serial |

On this machine, four workers are the best tested Plotly setting for complete
repeated analysis. The gain is intentionally modest because HDF5 loading,
data preparation, report writing, and parts of HTML serialization remain
serial. First-use consistency-cache construction is a separate one-time cost:
104.133 s for the 500-run bundle and 209.189 s for the 1,000-run bundle.

After the cache-preparation pass, controlled cold-cache refreshes of the same
copied bundles took 40.484 s for 500 runs and 75.145 s for 1,000 runs. The
largest remaining cost is reading packaged truth-error tables (22.180 s and
38.122 s respectively), followed by NEES/marginal derivation (13.814 s and
27.933 s). This is a deterministic single-owner path; parallel cache writes
remain intentionally out of scope until a future profile isolates a safe
independent CPU-bound stage.

## Monte Carlo consistency evidence

`tools/run_monte_carlo.py` now packages and renders consistency evidence after
the normal aggregate analysis. The generated `summary/consistency_figures/`
directory contains focused full-INS, navigation, IMU-bias, and GNSS dashboards.
Each left panel is a time-indexed ensemble-density heatmap with the mean
statistic and a 95-percent confidence interval for the mean. Each right panel
starts at the final epoch, then follows heatmap hover to update the across-run
histogram; the black curve is the expected chi-square probability density. The
dashboard adds a persistent vertical marker and **Download snapshot** control
for review artifacts. Click a heatmap or type an `Epoch [s]` value to pin it,
then use **Follow heatmap** to resume live hover selection. The mutually
exclusive **PDF**, **CDF**, and **QQ** controls switch every right panel between
the histogram/chi-square density comparison, empirical/chi-square cumulative
distribution comparison, and observed-versus-expected quantile comparison.
The QQ points should follow the identity line when the selected epoch is
consistent. Separate position, velocity,
attitude, gyro-bias, and accelerometer-bias axis dashboards expose the X/Y/Z
marginal normalized-squared-error drill-down alongside the joint products.

Empirical-CDF heatmap dashboards add a slim reference strip immediately right
of each measured heatmap. It shows the calibrated chi-square expectation at
the same statistic threshold (`F_chi-square(x)`), making the ideal vertical
slice visually comparable without covering the measured time history. The
residual heatmaps intentionally omit this strip because their expected value is
trivially zero everywhere.

The report directory contains JSON, CSV, and Markdown summaries. Mean NEES/NIS
confidence bounds correctly use `N * dof` chi-square degrees of freedom for an
ensemble of `N` independent runs, then normalize by `N`. Joint 1/2/3-sigma
coverage means the fraction of samples inside the corresponding multi-variate
chi-square ellipsoid; it is distinct from the secondary per-axis scalar
coverage. Every selected epoch uses one sample per run for its PDF, CDF, and QQ
views. Do not pool samples across time for a hypothesis-test interpretation
because successive estimator epochs are correlated.

The two black dashed heatmap curves are the lower and upper limits of the
two-sided 95-percent confidence interval for the ensemble mean—not duplicate
thresholds. The white curve is the observed ensemble mean. Heatmap color uses
`log(1 + count)` so both dense and sparse portions of the selected-epoch
distribution remain visible.

The dashboard right column uses a shared title above its first row and a shared
statistic label below its last row. Use **PDF** for density shape, **CDF** for
threshold and exceedance interpretation, and **QQ** for tail and
distribution-shape diagnosis. Hovering follows the selected heatmap epoch;
clicking or entering an epoch freezes it, and **Follow heatmap** resumes hover
selection.

This keeps error calculation, covariance bounds, decimation, state scaling, and
frame transforms out of renderer implementations. Add new domain plot builders
first; add renderer features only when they apply generally to prepared traces.

## Mathematical interpretation of consistency heatmaps

The consistency output is organized under:

```text
summary/consistency_figures/
  index.html
  density/
  empirical_cdf/
  cdf_probability_residual/
  cdf_probability_residual_uncertainty/
```

Every directory contains the same joint NEES, observation-family NIS, and
marginal axis dashboard groupings. Only the left time-history representation
changes. The right selected-epoch PDF/CDF/QQ controls remain available in every
diagnostic family.

### NEES and NIS reference distributions

For state error `e_i(t)` and its covariance `P_i(t)`, the normalized estimation
error squared is:

```text
NEES_i(t) = e_i(t)^T P_i(t)^(-1) e_i(t)
```

For a consistent state group with dimension `d`:

```text
NEES_i(t) ~ chi-square(d)
```

For innovation `nu_i(t)` and innovation covariance `S_i(t)`, the normalized
innovation squared is:

```text
NIS_i(t) = nu_i(t)^T S_i(t)^(-1) nu_i(t)
NIS_i(t) ~ chi-square(m)
```

Here, `m` is the effective measurement dimension. NIS products remain
separated by observation family and effective measurement dimension.

### Occurrence-density heatmap

The occurrence-density view uses time on the horizontal axis, raw NEES/NIS on
the vertical axis, and `log(1 + n[j,k])` as color. Here, `n[j,k]` is the number
of Monte Carlo samples in statistic bin `j` at epoch `k`. The white curve is
the ensemble mean. For `N` independent runs, the two black dashed curves are
the two-sided 95-percent consistency interval for that mean:

```text
chi-square-quantile(N*d, 0.025) / N
    <= mean_NEES(t) <=
chi-square-quantile(N*d, 0.975) / N
```

Both sides are meaningful: exceeding the upper limit indicates covariance that
is too small relative to observed errors, while falling below the lower limit
indicates covariance that is overly conservative relative to observed errors.
These curves bound the ensemble mean, not 95 percent of the heatmap samples.

### Empirical-CDF heatmap in raw statistic space

At each epoch, define the empirical CDF over the `N` Monte Carlo samples:

```text
ECDF_t(z) = count(x_i(t) <= z) / N
```

Here, `x_i(t)` is the applicable NEES or NIS statistic. This diagnostic uses:

- horizontal axis: time;
- vertical axis: raw NEES/NIS threshold `z`;
- color: empirical cumulative probability `ECDF_t(z)`.

The ECDF is calculated directly from the samples; its statistical definition
does not depend on histogram bins. It is evaluated on a finite vertical grid
only for rendering. Horizontal reference lines show the chi-square quantiles
for probabilities `0.6827`, `0.95`, and `0.99`, using the applicable state or
measurement dimension. These make familiar containment levels visible without
overlaying the entire theoretical CDF on the heatmap.

### CDF probability-space residual heatmap

Raw NEES/NIS magnitudes cannot be compared directly across products with
different dimensions. Transform every statistic through its expected
chi-square CDF:

```text
u_i(t) = chi-square-CDF(x_i(t), d)
```

This is the probability integral transform (PIT). Under consistency, `u_i(t)`
is uniformly distributed from zero to one, regardless of `d`. Let
`ECDF_u,t(p)` be the empirical CDF of the transformed samples. The plotted
residual is:

```text
R_t(p) = ECDF_u,t(p) - p
```

The normalized diagnostic uses:

- horizontal axis: time;
- vertical axis: CDF probability `p` from zero to one;
- color: empirical-minus-expected cumulative probability `R_t(p)`;
- expected result: zero everywhere.

A positive residual means more samples fall below that theoretical percentile
than expected; a negative residual means fewer do. The sign therefore
distinguishes where probability mass has moved and should not be discarded by
converting the result into a two-sided scalar test p-value. Horizontal
probability references at 0.6827, 0.95, and 0.99 provide common landmarks across
all state and observation dimensions.

The raw residual colorbar remains literally fixed from `-1` to `+1`, matching
the possible range of `R_t(p)`. Its pigment uses one symmetric generalized
logistic transfer function:

```text
Y(r) = A + (K - A) / (1 + Q * exp(-B * (r - M)))^(1 / nu)
```

The current symmetric visualization settings are `A=-1`, `K=+1`, `Q=1`,
`B=20`, `M=0`, and `nu=1`. `Y(r)` selects pigment while the displayed heatmap
value remains the raw residual `r`. Thus ordinary residuals near zero remain
visible without changing plotted values, hover values, or colorbar labels.
Color intensity is deliberately nonlinear, but every displayed residual value
stays in raw probability units. The uncertainty-normalized dashboard remains
the separate diagnostic for pointwise statistical significance.

Here, `u = F(x)` is a CDF probability or PIT value. It is not the conventional
upper-tail hypothesis-test p-value `1 - F(x)`. Keeping that terminology
distinct avoids reversing the interpretation of large and small values.

### CDF-residual statistical-uncertainty heatmap

The raw CDF-residual view measures the magnitude of the distribution mismatch.
Its companion uncertainty-normalized view answers whether that mismatch is
large relative to the finite number of independent Monte Carlo runs available
at the epoch. For `N_t` finite samples, the pointwise sampling standard error
of the empirical CDF at probability `p` is:

```text
SE_t(p) = sqrt(p * (1 - p) / N_t)
```

The uncertainty-normalized residual is:

```text
Z_t(p) = R_t(p) / SE_t(p)
```

The dashboard preserves time and CDF probability on the horizontal and
vertical axes. Color is the signed residual in pointwise standard-error units,
clipped visually at `+/-5` while hover text retains the calculated value.
Positive and negative signs retain the same probability-mass interpretation as
the raw residual. The endpoints `p = 0` and `p = 1` have zero binomial variance
and are intentionally left blank in the standardized view.

These are pointwise uncertainty units, not independent hypothesis-test results
for every heatmap pixel. Probability levels within an epoch share the same
empirical sample, and successive estimator epochs are temporally correlated.
A future simultaneous-inference layer may add DKW or campaign-bootstrap
envelopes, but naive multiple-testing claims across the entire heatmap would
not be valid.

### Selected-epoch PDF, CDF, and QQ views

The linked right panels retain the raw selected-epoch samples:

- **PDF** compares empirical density against the chi-square density.
- **CDF** compares the empirical and theoretical cumulative distributions.
- **QQ** plots observed quantiles against theoretical chi-square quantiles; a
  calibrated distribution follows the identity line.

These panels explain the distribution shape behind a time-local heatmap
feature. They always use one sample per run at the selected epoch. Pooling
successive estimator epochs would violate the independence assumption because
time-adjacent filter statistics are correlated.

The mathematical material in this section may move into a standalone LaTeX
consistency-analysis reference when the validation workflow expands beyond the
current NEES/NIS products.

## Troubleshooting

- If a plot appears stale, confirm the HDF5 and HTML modification times, then
  use `--refresh-cache` only if the source-derived cache is stale.
- If an expected table is missing, inspect the run's logging configuration and
  `*.meta.json`; analysis cannot reconstruct a product that was not logged.
- If interactive HTML becomes sluggish, reduce `--max-plot-points` before
  reducing campaign run count. Individual histories remain available in HDF5.
- If NEES/NIS is unavailable, confirm that the required covariance,
  truth/error, innovation, and innovation-covariance products were logged.
- If a campaign run must be reproduced, use its `input.effective.json`; do not
  manually recreate its derived seeds.
