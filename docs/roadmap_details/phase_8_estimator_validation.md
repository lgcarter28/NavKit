# Phase 8 - Estimator Validation and Consistency

**Status:** active. The current Phase 8 passes are maintained in
`docs/ROADMAP.md`. This file preserves only the completed validation foundation
and the design intent behind the active phase.

## Active pass intent

- Pass 8.9 promotes and validates the current-config managed qualification
  baseline now that all required statistical checks pass.
- Pass 8.10 adds state-definition-aware local-linear and empirical observability
  analysis with reusable HDF5 derivations and interactive Python
  visualizations.

Pass 8.4 has resolved the nine required stochastic failures. Application-owned
state-entry actions now bypass GNSS chi-square rejection only during declared
startup acquisition, preserve each sensor's configured probability/threshold,
and restore rejection before the scored window. Thirteen targeted pathological
replays, a 100-run diagnostic, and a fresh 500-run-per-profile matrix all
recovered; the final matrix passes 39 of 39 required checks. Baseline creation
and read-only baseline validation remain outstanding.

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

### Pass 8.3: stochastic qualification and dynamic-profile diagnosis

- [x] Added a versioned qualification suite with named 20-run smoke, 100-run
  diagnostic, and 500-run qualification tiers, explicit absolute/fractional
  analysis windows, and deterministic links to all seven dynamic campaigns.
- [x] Reduced each declared window to one scalar per independent Monte Carlo
  run before applying Student-t confidence intervals to required PVA NEES,
  GNSS position/velocity NIS, observation acceptance, selected full-INS NEES,
  and informational accelerometer-bias criteria.
- [x] Reused the HDF5 NEES/NIS evidence and added joint-to-marginal summaries
  plus sampled exact Schur-complement PVA/IMU-bias diagnostics instead of
  creating a parallel analysis stack.
- [x] Bound evidence to canonical runtime inputs, deterministic seeds, and the
  selected build/executable; rejected stale or incompatible reuse; and
  completed 500/500 runs for each of seven Release campaigns (3,500/3,500)
  plus 4/4 deterministic regressions.
- [x] Recorded the current-config result as a valid qualification failure: 30
  of 39 required checks passed. Ballistic gate lockout/tail behavior, narrow
  bank/skid-turn confidence misses, and elevated full-state
  bias/cross-covariance diagnostics remain visible rather than being converted
  into a permissive baseline.

### Pass 8.4: qualification reports and CI baseline management

- [x] Added versioned JSON and Markdown qualification reports with complete
  threshold, provenance, evidence-fingerprint, and known-finding disposition.
- [x] Added fail-closed read-only baseline comparison and explicit baseline
  update/replacement controls, plus cross-platform CI coverage for Python,
  scenario analysis, deterministic regressions, and retained failure evidence.
- [x] Added evidence-reuse, current-artifact, input-identity, compatibility, and
  baseline-policy tests so stale or incompatible evidence cannot be attributed
  to the selected suite.
- [x] Resolved the nine required stochastic failures without weakening their
  limits: acquisition phases bypass chi-square rejection, operational phases
  restore each configured threshold, thirteen pathological replays recovered,
  and the final 500-run matrix passed 39 of 39 required checks.

### Pass 8.5: mission, subsystem-phase, and execution-target configuration boundaries

- [x] Split target-independent mission phases and transitions from named,
  product-specific Navigation phases, reusable GNC/platform configuration,
  simulation truth/plant/control inputs, and execution-target ownership.
- [x] Added complete per-sensor Navigation-phase innovation-gate selections
  with optional probability overrides, GNSS-baseline fallback, reference
  validation, and fail-before-mutation application on mission transitions.
- [x] Added the fail-closed `navkit_swil` execution target and an explicit
  app-support compatibility adapter into the existing generated-trajectory
  engine. Mission and embedded NavKit contracts remain free of synthetic truth,
  clocks, sensor emulators, and external-framework dependencies.
- [x] Migrated all 29 supported scenario manifests to role-keyed mission,
  Navigation, GNC, simulation, execution, sensor, initialization, logging, and
  analysis composition. Updated tests and documentation, and verified Debug,
  Release, stationary, and dynamic SWIL paths.

### Pass 8.6: explicit runtime configuration graph and target-ready composition

- [x] Renamed the reusable runtime root from `config/runtime/navkit_sim/` to
  `config/runtime/navkit/` without a compatibility alias, while retaining
  `navkit_sim` only for the executable, compile-time application selection, and
  implemented SWIL adapter. Updated all scenarios, campaigns, regressions,
  qualification inputs, tools, tests, CI paths, examples, and provenance.
- [x] Established the coherent runtime hierarchy
  `components/mission/`,
  `components/gnc/{guidance,navigation,autopilot,filters}/`,
  `components/simulation/`, and `components/execution/`, with execution-target
  ownership kept separate from simulated truth and plant ownership.
- [x] Made each mission's ordered `phases[]` array the sole mission state
  machine. Removed the parallel Guidance, Navigation, state-machine, and
  platform catalogs; each phase now owns or explicitly references Navigation,
  Guidance, Autopilot, and transition/terminal behavior.
- [x] Added a reusable recursive inline-or-reference contract with references
  resolved relative to the containing file, deep object overrides, whole-array
  replacement, and fail-closed validation for ambiguous shapes, missing files,
  reference cycles, incompatible payloads, and orphan overrides. C++ and Python
  resolvers have parity coverage, including complete missing/cycle chains.
- [x] Kept truth, environment, vehicle/plant, sensor emulation, and integration
  under simulation ownership. Simulation phase behavior is keyed explicitly by
  stable mission phase IDs and must map the mission phase set exactly.
- [x] Kept scenarios as thin explicit composition roots selecting mission,
  execution target, optional simulation, sensors, initialization, logging, and
  purpose. Missions remain target-independent; the implemented SWIL adapter
  supplies simulated lifecycle dependencies, while unsupported HWIL, flight,
  and third-party targets fail closed pending their adapters.
- [x] Migrated and recursively validated all 29 supported scenarios. Added
  topology, exact phase-map, target-compatibility, nested-reference, relative
  asset, and effective-config relocation coverage. Stationary and CSV sources
  explicitly reject multi-phase missions until a future reusable
  `MissionRuntime` extracts phase progression from generated trajectories.
- [x] Verified 83 Python tests, a clean Debug build, all 279 C++ test cases
  (43,077 assertions), a clean Release build, and all four deterministic
  truth-reconstruction regressions. Re-ran the complete seven-profile,
  500-run-per-profile qualification matrix: all 39 required stochastic checks
  passed and the stochastic result is `pass_with_known_findings`; the overall
  report remains fail-closed only because managed-baseline promotion is
  deliberately reserved for Pass 8.9.

### Pass 8.7: target-agnostic mission runtime and execution adapters

- [x] Replaced the SWIL-specific host loop with `MissionApp`, whose visible
  responsibilities are planned-time sequencing, mission synchronization,
  adapter lifecycle calls, Navigator updates, cleanup, and logging.
- [x] Added target-neutral `MissionRuntime` ownership of stable phase IDs,
  checked transition indices, graph topology validation, active-phase state,
  and destination Navigation-phase application at the planned deadline.
- [x] Added a shallow app-support `MissionAdapter` lifecycle separating
  pre-deadline preparation, at-deadline publication/acquisition, and
  post-navigation feedback. The concrete SWIL adapter owns synthetic truth,
  emulators, initialization, prepared payloads, and controller feedback.
- [x] Recognized `hwil` as an execution-target value while keeping validation
  and adapter construction fail closed until a real transport/runtime adapter
  exists; flight and third-party targets remain future explicit adapters.
- [x] Split target-neutral `navkit::app_support_common` over core/IO from the
  SWIL support umbrella over common support/simulation, with a
  compile-only dependency smoke protecting the neutral include boundary.
- [x] Replaced trajectory-owned semantic phase identity with adapter-reported
  stable mission IDs and renamed the remaining diagnostic/log field to
  `mission_phase_index`. Moved simulation-run state out of
  `TrajectoryProvider.hpp` and decoupled PVA/transfer-alignment startup APIs
  from the old simulation aggregate.
- [x] Added phase-graph, transition, lifecycle-order, adapter selection,
  fail-closed HWIL, exact terminal-sample, unaligned finite-source completion,
  and complete host-loop coverage. Reconciled architecture, configuration,
  setup, compile-time-config, and trajectory-purpose documentation.

### Pass 8.8: target-specific applications and mixed-HWIL composition seam

- [x] Renamed the in-process executable from `navkit_sim` to `navkit_swil`,
  retained `navkit::sim` as the reusable compiled simulation library, and
  removed the empty replay executable rather than preserving a nonfunctional
  compatibility target.
- [x] Kept one target-neutral `MissionApp<Config>` host loop and made the thin
  executable supply a fixed `SwilMissionAdapterFactory`. The factory returns a
  `std::unique_ptr<MissionAdapter<Navigator, Logger>>`; the product config no
  longer duplicates application or target selection.
- [x] Split common mission, Navigation, filter, initialization, and propagation
  validation from SWIL-owned truth, plant, emulator, and rate validation.
  Executable/runtime target mismatches fail before either schema is parsed,
  while direct SWIL validation remains fail closed for non-SWIL targets.
- [x] Established the `navkit::app_support_common` target over core/IO and the
  `navkit::swil_support` target over common/simulation, with reusable host-side
  APIs in `navkit::app_support` and concrete SWIL composition in
  `navkit::swil`. Added a compile-only no-simulation dependency smoke.
- [x] Strengthened pure-SWIL compile-time composition so every configured
  Navigator aiding sensor has exactly one emulator binding; duplicate, unknown,
  and incomplete producer graphs are rejected. Mixed emulator/hardware channel
  ownership remains deferred until the first real HWIL transport exists.
- [x] Kept `hwil` recognized but unavailable rather than inventing a fake
  adapter. Updated compile-time config paths, build roots, presets, tools,
  manifests, install/export names, VS Code launch configurations, tests, and
  documentation for the `navkit_swil` application boundary.

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
