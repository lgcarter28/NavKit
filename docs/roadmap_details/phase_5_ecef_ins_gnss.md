# Phase 5 - ECEF INS/GNSS Implementation History

**Status:** complete through the current working GPS/GNSS-aided ECEF INS baseline. Remaining hardening/expansion items were moved to the active roadmap.

## Pass 5.1: ECEF navigator algorithm documentation

- [x] Created the focused ECEF navigator v1 LaTeX document.
- [x] Refined the document around quaternion nominal attitude propagation, multiplicative attitude-error injection, IMU increments, coning/sculling, continuous/discrete system matrices, process-noise mapping, GNSS position/velocity observations, and lever-arm terms.

## Pass 5.2: IMU emulator algorithm documentation

- [x] Created the focused IMU emulator v1 LaTeX document.
- [x] Refined IMU emulator equations around truth PVA input, ECEF-to-ECI/body-frame conversion, incremental-angle/incremental-velocity output, triad calibration errors, stochastic errors, and discrete-time algorithm requirements.
- [x] Added LaTeX verification guidance to agent/documentation workflow.

## Pass 5.3: IMU increment contract and simulator

- [x] Added `ImuIncrement`.
- [x] Simplified `TruthSample` to truth PVA content and moved IMU-derived quantities into the simulator.
- [x] Implemented deterministic IMU increment generation from consecutive ECEF truth samples.
- [x] Added Earth-rate, specific-force, bias, bias random walk, white noise, scale factor, misalignment, non-orthogonality, and quantization support.
- [x] Reworked simulator initialization/generation around explicit `initialize()` and bool-returning generation.
- [x] Moved reusable triad calibration and rotation/math helpers out of simulator-specific implementation where appropriate.
- [x] Added compile-time coning/sculling compatibility between simulator and navigator configuration.

## Pass 5.4: ECEF INS/GNSS Navigator implementation

- [x] Added a working ECEF strapdown INS propagation path.
- [x] Added quaternion nominal attitude state support and removed legacy roll/pitch/yaw nominal state usage from the primary INS state definition.
- [x] Added body-to-ECEF quaternion convention and multiplicative attitude-error injection.
- [x] Added high-rate IMU increment handling.
- [x] Added medium-rate covariance propagation accumulation with state-transition/process-noise accumulation.
- [x] Moved covariance-propagation ownership into `KalmanFilter`.
- [x] Kept propagation math responsible for dynamics matrices rather than filter business logic.
- [x] Added GNSS position and velocity aiding.
- [x] Added GNSS antenna lever-arm support in simulator truth generation and measurement-model Jacobians.
- [x] Removed obsolete state-definition aliases and clarified nominal/error state definitions.

## Pass 5.5: stabilization, covariance ownership, and aiding cleanup

- [x] Stabilized the first working ECEF INS/GNSS scenario family.
- [x] Cleaned up covariance propagation ownership between `Navigator`, propagation math, and `KalmanFilter`.
- [x] Added GNSS velocity and lever-arm integration across simulator, emulator, measurement model, and plotting paths.

## Pass 5.6: runtime scenarios, logging, and plots

- [x] Decomposed runtime JSON configs into reusable components for logging, trajectory, IMU, GNSS, PVA initialization, and filter initialization.
- [x] Added default ECEF INS/GNSS, runtime covariance override, IMU debug, modeled/unmodeled IMU-error, and truth-reconstruction scenarios.
- [x] Split simulation output folders into `data` and `figures`.
- [x] Added truth trajectory, nominal estimate/covariance, filter correction, GNSS debug, IMU increment/debug/error, innovation, NIS/p-value, histogram, ECEF/NED error/covariance, and dashboard plotting.
- [x] Added runtime log rates and output directory selection.
- [x] Added one-liner scenario run-and-plot workflow.
- [x] Added scenario smoke/diagnostic runs during implementation.

## Pass 5.7: initialization split and scenario tooling

- [x] Split startup config into `pva_initialization` and `filter_initialization`.
- [x] Reworked `NavInitialization` toward PVA nominal data plus full filter covariance.
- [x] Added PVA random-error, explicit-error, no-error, and direct-value initialization examples.
- [x] Added filter initial covariance runtime overrides, including diagonal, full, and PVA-plus-remaining-error-state forms.
- [x] Removed redundant runtime covariance source fields.

## Pass 5.8: runtime hygiene and IMU variance randomization

- [x] Decomposed ECEF INS/GNSS compile-time product and app configs into reusable `components` headers plus scenario-level aggregate headers.
- [x] Updated IMU simulator and runtime JSON vocabulary to explicit `bias_turnon`, `bias_inrun`, `scale_factor`, `angle_random_walk`, and `velocity_random_walk` terms.
- [x] Added seeded runtime variance/covariance draws for deterministic static IMU triad terms and validation that rejects ambiguous direct/random forms.
- [x] Added IMU angular-rate and acceleration saturation limits.
- [x] Added real-spec-inspired IMU runtime examples for consumer, industrial, tactical, and navigation-grade IMUs.
- [x] Improved console status output with aligned LLA position, NED velocity, and NED attitude Euler-angle quantities.
- [x] Updated runtime configuration and IMU emulator algorithm documentation for the new IMU config contract.

## Pass 5.9: simulator/emulator stochastic config ownership

- [x] Moved IMU random draw realization out of runtime JSON parsing and into `ImuSimulator`, which owns the seeded random-number generator and realized stochastic state.
- [x] Reworked IMU runtime parsing to validate and translate declarative direct/variance/covariance forms without stochastic side effects.
- [x] Normalized IMU diagonal variance and full covariance JSON forms into one C++ covariance descriptor per randomizable triad term.
- [x] Preserved direct/static IMU error values, PSD-driven in-run stochastic terms, limits, and quantization as plain parsed config.
- [x] Confirmed GNSS simulator/emulator stochastic ownership already follows the same boundary: runtime config stores seed/covariance, while `GnssSimulator` owns noise draws.
- [x] Added tests for deterministic parsing, seeded simulator realization, and equivalent diagonal/full covariance descriptors.
- [x] Updated configuration and IMU emulator docs to state that runtime parsing is declarative and simulator/emulator objects own stochastic draws.

## Pass 5.10: full-rate IMU cumulative increment logging

- [x] Moved IMU cumulative increment ownership out of `ImuIncrementLogProduct` and into the full-rate IMU runtime path.
- [x] Extended the IMU nominal log payload to carry full-rate run cumulative sums for truth/ideal and measured increments.
- [x] Made lower-rate IMU logging a snapshot mechanism: each logged row writes the latest increment plus the full-rate run cumulative sums up to that timestamp.
- [x] Updated IMU cumulative plot titles and CSV metadata to state that cumulative values are full-rate run snapshots.
- [x] Added a focused runtime test that verifies cumulative body-Z specific-force/velocity increment growth remains correct across unlogged generated IMU samples.
- [x] Ran the default ECEF INS/GNSS scenario and analysis plots to verify the updated CSV schema and figures.

## Pass 5.11: first-order Gauss-Markov IMU dynamics

- [x] Updated the IMU emulator and ECEF navigator LaTeX algorithm references for first-order Gauss-Markov IMU bias dynamics, including bias correlation-rate notation and matching continuous-time F/Fk matrix blocks.
- [x] Added a first-pass compile-time filter-facing dynamics payload for INS process-noise and bias-correlation configuration.
- [x] Wired selected IMU bias dynamics through the ECEF INS propagation policy, Navigator-owned runtime selection, covariance-step construction, and runtime config validation.
- [x] Added first-pass runtime JSON override support for selected propagation dynamics, following the compile-time-default/runtime-override pattern used by initial covariance.
- [x] Added optional first-order Gauss-Markov bias propagation to the IMU simulator while preserving zero-correlation-rate random-walk behavior for existing configs.
- [x] Added compile-time, runtime-validation, simulator, Navigator, and propagation tests that distinguish nonzero bias correlation from the previous pure-random-walk model.
- [x] Verified formatting/copyright checks, Debug build, full Debug tests, default scenario analysis, and both affected LaTeX PDFs.

## Pass 5.12: covariance floors and unused-parameter cleanup

- [x] Added `CovarianceFloor<StateDef>`, diagonal floor construction helpers, and a covariance-floor config policy for immutable compile-time product defaults.
- [x] Added a default INS covariance-floor component using the selected split INS error-state definition directly.
- [x] Wired the selected covariance floor through the NavKit product config and filter initialization path so `KalmanFilter` applies floors after startup selection, covariance propagation, and measurement processing.
- [x] Added runtime JSON override support for covariance floors under `filter_initialization.covariance_floor`, with raw diagonal and frame-aware PVA-plus-remaining-error-state diagonal forms.
- [x] Added validation that rejects malformed, negative, ambiguous, and unsupported full-matrix covariance-floor runtime inputs.
- [x] Added a zero-floor runtime component example that exercises the config path without changing the current ECEF INS/GNSS scenario behavior.
- [x] Swept stale app-support/core unused-parameter breadcrumbs, removing obsolete `(void)cfg;` plumbing and replacing parser-result discard breadcrumbs with explicit validation checks while preserving intentional concept/API no-op parameters.
- [x] Added compile-time, runtime-validation, and Navigator/filter covariance-floor tests.

## Pass 5.13: filter/propagation configuration ownership cleanup

- [x] Split ECEF INS process-noise configuration from first-order Gauss-Markov IMU bias dynamics so PSD values and bias-correlation rates have distinct compile-time and runtime ownership.
- [x] Replaced the combined propagation runtime JSON object with `propagation.process_noise` and `propagation.imu_bias_dynamics`, each with focused validation and compile-time defaults.
- [x] Moved propagation runtime configuration ownership into the selected propagation policy instance instead of storing propagation internals on `Navigator`.
- [x] Moved covariance-floor ownership into `KalmanFilter`, with filter-owned clamping after set-covariance, covariance propagation, accepted measurement updates, and reset-policy covariance changes.
- [x] Routed runtime propagation configuration and covariance-floor selection through `initialize_navigator()` so `SimulationApp` stays focused on orchestration rather than member-object tuning.
- [x] Updated propagation/filter policy tests and runtime-validation tests to exercise the new member-owned configuration seams.
- [x] Updated configuration documentation and changelog entries to describe the split propagation configuration and filter-owned covariance floor.

## Pass 5.14: advanced restart initialization groundwork

- [x] Preserved the normal startup path as the clean default: `pva_initialization` provides the required nominal PVA message, while `filter_initialization.initial_covariance` provides the full Kalman filter covariance belief.
- [x] Added an explicit advanced non-PVA nominal-state override under `filter_initialization.nominal_state.non_pva_values` for restore/manual-analysis use cases.
- [x] Kept the restart override generic over the selected compile-time nominal state definition by validating and applying only values after the nominal PVA prefix.
- [x] Kept PVA startup, filter covariance belief, restore/manual nominal-state overrides, transfer alignment, and future Monte Carlo truth-relative initialization as distinct concepts.
- [x] Kept product-core embedded code free of simulator truth/error context; the restart override is runtime app-support initialization glue applied after normal PVA/covariance initialization.
- [x] Added runtime validation that rejects malformed nominal-state override vectors, unknown `nominal_state` keys, and unknown top-level `filter_initialization` keys.
- [x] Added a reusable example filter-initialization component for direct nominal gyro/accelerometer bias restart values without wiring it into normal scenarios.
- [x] Added tests showing that normal initialization still maps PVA/covariance, direct non-PVA nominal overrides initialize only selected state segments, malformed/unknown restart inputs are rejected, and transfer alignment remains independent.

## Phase 5 follow-forward

Unresolved expansion/hardening work from Phase 5 now lives in future phase files for Monte Carlo, trajectory-source expansion, transfer alignment, sensor scheduling, validation metrics, latent replay, robust status/error handling, and embedded readiness.
