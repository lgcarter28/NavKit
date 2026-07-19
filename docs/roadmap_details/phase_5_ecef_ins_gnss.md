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

## Phase 5 follow-forward

Unresolved expansion/hardening work from Phase 5 now lives in active roadmap passes for Gauss-Markov IMU dynamics, covariance floors, unused-parameter cleanup, and advanced restart initialization, plus future phase files for Monte Carlo, trajectory-source expansion, transfer alignment, sensor scheduling, validation metrics, latent replay, robust status/error handling, and embedded readiness.
