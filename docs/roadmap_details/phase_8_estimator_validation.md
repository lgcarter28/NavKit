# Phase 8 - Estimator Validation and Consistency

**Status:** active. Pass 8.1 is maintained in `docs/ROADMAP.md`; Passes 8.2 and
8.3 remain future backlog detail here.

## Pass 8.1: deterministic estimator regression baselines

Active ownership is maintained in `docs/ROADMAP.md`.

## Pass 8.2: innovation and covariance consistency

- [ ] Add per-axis and state-family innovation/residual summaries: mean, covariance, standard deviation, RMS, and sample count over explicit analysis windows.
- [ ] Add standardized and jointly whitened innovation time histories.
- [ ] Add Gaussian overlays and QQ diagnostics for standardized/whitened innovations.
- [ ] Add lagged innovation autocorrelation and statistically justified whiteness tests.
- [ ] Define stochastic pass/fail thresholds and required campaign sizes explicitly; never treat one stochastic run as proof of consistency.
- [ ] Diagnose the Phase 7 dynamic-profile full-state joint-NEES failures, including cross-correlation, reset, linearization, process-noise, and numerical covariance effects, while retaining the state-family and NIS evidence that already behaves credibly.
- [ ] Make filter-correction logs exact under multiple accepted updates at one
  epoch. Either emit one correction/injection event per accepted update or
  record explicit cycle-start/cycle-end nominal states and compose attitude
  corrections multiplicatively; do not present a first-order vector sum of
  small-angle corrections as an exact nonlinear cycle correction. Add a
  replay/reconstruction test.
- [ ] Add maneuver- and state-family observability diagnostics that expose
  information growth for attitude and modeled IMU biases across static,
  ballistic, calibration, and turning trajectories. Preserve the intentional
  high-fidelity simulator versus lower-order estimator-state mismatch while
  evaluating consider-state methods and justified process-noise inflation
  before expanding the filter state.

## Pass 8.3: validation reports

- [ ] Produce machine-readable qualification/regression reports with named threshold results, pass/fail status, schema/build/config provenance, and baseline deltas.
- [ ] Produce concise human-readable qualification reports and CI summaries with links to retained diagnostic artifacts.

## Phase 8 completed foundation

- [x] Basic plots, innovation histories, NIS/p-value plots, histograms, ECEF/NED covariance/error plots, and dashboard plots exist.
- [x] Current scenario tooling can run simulations and generate analysis artifacts from one command.
- [x] Phase 6 provides full-INS, PVA, position, velocity, attitude, combined-bias, gyro-bias, and accelerometer-bias joint NEES plus GNSS position/velocity NIS.
- [x] Phase 6 provides density, empirical-CDF, probability-residual, uncertainty-normalized residual, QQ, mean-confidence, coverage, HDF5, and machine-readable reporting foundations.
- [x] Six pre-Pass-7.11 dynamic campaigns completed with 1,000/1,000 successful
  runs each and generated HDF5 bundles, aggregate covariance/error plots,
  consistency dashboards, and reports.
- [x] Six post-Pass-7.11 campaigns completed with 500/500 successful runs each
  (3,000/3,000 total), full HDF5 bundles, aggregate reports, and interactive
  consistency products under the dedicated Pass 7.11 analysis root.
- [x] Six post-Pass-7.12 campaigns completed with 500/500 successful runs each
  under `output/analysis/pass_7_12/monte_carlo_500_final`; all six have valid
  analysis evidence, including the recovered waypoint consistency bundle and
  rebuilt constant-altitude analysis bundle.
- [x] These campaign generations exposed valuable Phase 8 work: GNSS
  position/velocity NIS was generally near unity and many marginal
  state-family metrics were credible, but several dynamic profiles showed
  substantial full-state joint-NEES inconsistency. Phase 7 therefore
  establishes repeatable evidence, not a blanket estimator-consistency claim.
- [x] The post-Pass-7.12 evidence narrows the remaining concern: PVA families
  and GNSS NIS are generally healthy, while constant-altitude and waypoint
  retain elevated full-state NEES associated with
  accelerometer-bias/cross-covariance behavior and gyro-z remains weakly
  observable.
