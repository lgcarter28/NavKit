# Phase 2 - Estimator Policy Boundaries

**Status:** complete for the original estimator-boundary refactor scope.

## Pass 2.1: injection and reset

- [x] Defined candidate-first `InjectionPolicy<Candidate, StateDef>`.
- [x] Defined candidate-first `ResetPolicy<Candidate, StateDef>`.
- [x] Constrained `KalmanFilter` on `StateDefPolicy`, injection, and reset policies.
- [x] Preserved the intended INS additive-injection sign convention and zero-error reset behavior at the time of the pass.
- [x] Kept covariance reset explicit rather than hidden.
- [x] Added valid and invalid compile-time policy tests.

## Pass 2.2: measurement models

- [x] Defined `MeasurementModelPolicy<Candidate, StateDef>` around dimensions, fixed-size matrix types, context, observation, Jacobian, covariance, and Kalman-gain operations.
- [x] Retained shared CRTP/base support only where it clarified implementation.
- [x] Constrained `KalmanFilter` observation-update and measurement-statistics boundaries.
- [x] Verified GNSS position, GNSS velocity, and barometer model conformance at the concept boundary.
- [x] Added negative compile-time tests for missing types and operations.

## Pass 2.3: sensors and noise

- [x] Defined noise-policy compatibility for measurement models and samples.
- [x] Constrained `Sensor<Id, MeasurementModel, BufferSize, NoisePolicy>`.
- [x] Added sensor diagnostics compatibility without forcing diagnostics into unrelated consumers.
- [x] Preserved fixed-capacity, allocation-aware behavior.

## Pass 2.4: diagnostics

- [x] Preserved innovation, innovation covariance, measurement covariance, Jacobian, gain, NIS, timestamp, validity, and acceptance logging.
- [x] Added runtime regression coverage for accepted/rejected update behavior and statistics.

## Phase 2 follow-forward

Richer status/error handling, covariance health diagnostics, delayed-measurement context, and advanced validation metrics now live in the active roadmap.
