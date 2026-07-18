# Phase 6 - Monte Carlo and Batch Analysis

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Monte Carlo support comes immediately after Phase 5 because it is the gold-standard workflow for navigation analysis and will turn the current single-run scenario tooling into repeatable statistical evidence.

## Pass 6.1: seeded Monte Carlo driver

- [ ] Add a seeded batch/Monte Carlo driver.
- [ ] Parallelize independent runs without changing determinism.
- [ ] Preserve scenario-level runtime JSON composition while allowing run-index/seed/output-directory expansion.
- [ ] Keep single-scenario execution and plotting usable without requiring the Monte Carlo driver.

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
