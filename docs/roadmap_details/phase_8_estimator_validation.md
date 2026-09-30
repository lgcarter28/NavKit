# Phase 8 - Estimator Validation and Consistency

**Status:** active. The current Phase 8 passes are maintained in
`docs/ROADMAP.md`. This file preserves only the completed validation foundation
and the design intent behind the active phase.

## Active pass intent

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

### Pass 8.2: runtime innovation acceptance and correction integrity

- [x] Added runtime-configured chi-square innovation gates for separate GNSS
  position and velocity observations. Gate degrees of freedom come from the
  measurement-model dimension, thresholds are derived from configured
  probabilities, and invalid or over-threshold observations are rejected
  before persistent filter mutation.
- [x] Added unit, runtime-configuration, and end-to-end coverage for accepted
  and rejected observations, including sequential position/velocity updates
  at one Navigator epoch.
- [x] Made filter-correction logging exact for multiple accepted updates in one
  Navigator cycle by composing corrections in injection order, including
  quaternion composition for attitude, and added CSV replay coverage for the
  composed result.

### Pass 8.7: target-agnostic mission runtime and execution adapters

- [x] Replaced the SWIL-specific host loop with target-neutral `MissionApp`
  ownership of planned-time sequencing, mission synchronization, Navigator
  updates, cleanup, and logging.
- [x] Added `MissionRuntime` ownership of stable phase IDs, validated graph
  topology, checked transitions, active-phase state, and Navigation-phase
  application at the planned deadline.
- [x] Added a shallow `MissionAdapter` lifecycle separating pre-deadline
  preparation, at-deadline publication/acquisition, and post-navigation
  feedback. Synthetic truth and emulator state remain SWIL-owned.
- [x] Added exact terminal-sample, unaligned finite-source, cleanup, lifecycle,
  graph, transition, and fail-closed target tests. Renamed the remaining
  trajectory diagnostic identity to `mission_phase_index`.

### Pass 8.8: target-specific applications and mixed-HWIL composition seam

- [x] Renamed the in-process executable from `navkit_sim` to `navkit_swil`,
  retained `navkit::sim` as reusable simulation infrastructure, and removed
  the empty replay executable.
- [x] Made the thin executable supply a fixed `SwilMissionAdapterFactory` to
  the common `MissionApp`. The factory creates the concrete adapter behind
  `std::unique_ptr<MissionAdapter<Navigator, Logger>>`; product configs no
  longer duplicate application or target selection.
- [x] Split common mission, Navigation, filter, initialization, and propagation
  validation from SWIL-owned truth, plant, emulator, and rate validation.
  Executable/runtime target mismatches fail before target-specific schema
  parsing.
- [x] Established `navkit::app_support_common` over core/IO and
  `navkit::swil_support` over common/simulation. Concrete SWIL composition
  lives under `navkit::swil`; reusable simulators remain under `navkit::sim`.
- [x] Strengthened pure-SWIL composition so every configured Navigator aiding
  sensor has exactly one emulator binding. Mixed emulator/hardware ownership
  remains a deliberate seam until the first real transport exists.
- [x] Kept `hwil` recognized but unavailable rather than inventing a fake
  adapter. Updated config paths, build roots, presets, tools, install/export
  names, tests, and VS Code launch preparation for `navkit_swil`.
- [x] Verified copyright/format checks, all 83 Python tests, clean Default and
  Profiled Debug builds and tests, an installed-package smoke, a clean Release
  build, and all four deterministic truth-reconstruction regressions.

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
