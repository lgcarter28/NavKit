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
| Monte Carlo campaign config/manifest | `navkit.monte_carlo_campaign.v1` |
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
```

`data` contains the raw analysis tables needed by current plotting. `derived`
contains cached calculations such as truth-aligned navigation error. Campaign
bundles additionally cache downsampled aggregate Monte Carlo series. Bundle
metadata records source runtime configs/manifests when available, seeds,
compile-time metadata carried by the source logs, units/frame conventions, plot
decimation, and the truth/NED/covariance derivation assumptions.

HDF5 is an offline analysis artifact, not an embedded logging format. Future
targets should emit an embedded-appropriate NavKit binary stream and use Python
to decode/repackage it into this analysis layer.

## Workflow

Package an existing run or campaign:

```powershell
python tools/package_analysis.py output/logs/ecef_ins_gnss_demo
python tools/package_analysis.py output/monte_carlo/ecef_ins_gnss_smoke_mc
```

Monte Carlo campaigns package HDF5 and generate Plotly interactive aggregate
figures by default after their reports. Set `analysis.renderer` to `matplotlib`
when static PNG output is specifically needed. Package historical CSV output
manually when needed.

Use the normal static plotting path on either a CSV directory or a single-run
bundle:

```powershell
python tools/run_analysis.py output/logs/ecef_ins_gnss_demo
python -m navkit_analysis.plots output/logs/ecef_ins_gnss_demo/data/analysis_bundle.h5
```

Regenerate a cached campaign aggregate from either raw CSV runs or the bundle:

```powershell
python tools/plot_monte_carlo.py output/monte_carlo/ecef_ins_gnss_smoke_mc --plot attitude_ned
python tools/plot_monte_carlo.py output/monte_carlo/ecef_ins_gnss_smoke_mc/analysis_bundle.h5 --plot attitude_ned --renderer plotly
```

For quick exploration, select arbitrary named fields without creating a custom
script:

```powershell
python tools/plot_analysis.py output/logs/ecef_ins_gnss_demo --table nav --y p_e_x_m --renderer plotly
```

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

This keeps error calculation, covariance bounds, decimation, state scaling, and
frame transforms out of renderer implementations. Add new domain plot builders
first; add renderer features only when they apply generally to prepared traces.
