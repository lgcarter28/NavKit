# Phase 8 - Estimator Validation and Consistency

**Status:** active. The current Phase 8 passes are maintained in
`docs/ROADMAP.md`. This file preserves only the completed validation foundation
and the design intent behind the active phase.

## Active pass intent

- Pass 8.2 adds runtime chi-square measurement rejection and proves exact
  correction logging across same-epoch accepted updates.
- Pass 8.3 adds state-definition-aware local-linear and empirical observability
  analysis with reusable HDF5 derivations and interactive Python
  visualizations.
- Pass 8.4 turns existing Monte Carlo consistency and observability products
  into declared qualification criteria and diagnoses the remaining joint-NEES
  findings.
- Pass 8.5 creates CI-facing qualification reports and baseline management.

## Completed passes

### Pass 8.1: deterministic truth-reconstruction regressions

- [x] Added stationary free-inertial reconstruction from exact initial PVA and
  ideal IMU increments with GNSS availability explicitly outside the run.
- [x] Added stationary truth-GNSS position/velocity reconstruction from exact
  initial PVA and ideal IMU increments.
- [x] Added ballistic and horizontal bank-to-turn dynamic reconstruction using
  truth-passthrough control so the estimator cannot perturb its own oracle.
- [x] Added strict finite/monotonic timestamp validation, truth coverage checks,
  ECEF position/velocity interpolation, quaternion SLERP, relative-rotation
  attitude error, and focused numerical-contract tests.
- [x] Added one versioned regression command and report with compile-time
  product, build, suite, scenario, effective-config, metric, threshold, and
  pass/fail provenance. Passing logs are temporary; failures and explicit
  `--retain-artifacts` runs preserve complete evidence.
- [x] Added explicit sensor-path evidence: required statistics products count
  distinct accepted GNSS position/velocity timestamps, the free-inertial
  contract requires zero, and aided contracts require both observation
  families throughout their declared windows.
- [x] Verified all four Release cases against evidence-calibrated contracts.
  Stationary reconstruction remained near floating-point precision; ballistic
  and bank-to-turn maximum position errors remained below 0.8 mm and 0.14 mm,
  respectively.

## Earlier completed validation foundation

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
