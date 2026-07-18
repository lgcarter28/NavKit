# Phase 10 - Loosely Coupled Sensor Model Cleanup

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase cleans up receiver-level aiding and simple environmental sensors before the tightly coupled GNSS phase.

## Pass 10.1: loosely coupled GNSS documentation and combined model

- [ ] Split loosely coupled GNSS aiding into its own complete LaTeX algorithm document before major refactors. Cover receiver-level position/velocity observations, lever arms, valid flags, covariance frame transforms, combined position/velocity updates, Jacobian structure, logging/validation expectations, and the intended default configuration.
- [ ] Add a combined GNSS position/velocity emulator output and measurement model as the preferred default GNSS aiding path. Reuse the existing position and velocity machinery where possible, but process the combined observation in one Kalman update so the full position/velocity Jacobian and covariance can improve observability and convergence.
- [ ] Add validity flags to combined GNSS position/velocity measurements so individual epochs can contain position-only, velocity-only, or full position/velocity data. The measurement model should handle invalid components deliberately, such as by selecting only valid rows or otherwise removing invalid contributions from the innovation, Jacobian, and covariance rather than corrupting the update.
- [ ] Add multi-GNSS-receiver configuration examples and tests. Use nonzero lever arms by default in at least one scenario so lever-arm handling is continuously exercised rather than silently validated only at the zero-lever-arm case.

## Pass 10.2: altimeter and pressure-sensor models

- [ ] Add a complete LaTeX algorithm document for barometric altitude and pressure aiding before cleanup implementation.
- [ ] Add shared atmosphere-model requirements before implementing pressure-derived sensors. This should define the pressure, density, temperature, altitude, frame, unit, and runtime-configuration contracts reused by both barometric altitude and pitot/air-data models.
- [ ] Implement the barometer simulator and replace the placeholder altitude model with a physically meaningful ECEF/local-vertical altitude model and Jacobian.
- [ ] Add real atmospheric modeling appropriate for pressure-sensor simulation and measurement conversion.

## Pass 10.3: pitot tube and air-data support

- [ ] Add a complete LaTeX algorithm document for pitot tube, static pressure, angle-of-attack, sideslip, air-data, and wind/relative-air modeling before implementation.
- [ ] Reuse the shared atmosphere-model contract from Pass 10.2 rather than creating pitot-specific atmospheric assumptions.
- [ ] Add a pitot tube / air-data model with enough atmosphere and vehicle-state context to support useful simulation.
- [ ] Model pressure, dynamic pressure, airspeed, angle of attack, and sideslip observations with clear frame conventions, units, covariance/noise configuration, validity flags, and logging/plotting expectations.
- [ ] Add basic wind modeling and optional relative-wind state estimation support. Keep the sensor model focused on air-relative observations, while the filter may optionally estimate wind states such as local-level or ECEF-resolved wind velocity.
- [ ] Add air-data measurement-model Jacobians with respect to navigation velocity, attitude, and optional wind states.
- [ ] Add validation scenarios with enough maneuver excitation to make wind and air-data observability meaningful.

## Pass 10.4: magnetometer aiding

- [ ] Add a complete LaTeX algorithm document for magnetometer aiding before implementation, covering magnetic-field model requirements, body-frame mounting, hard/soft iron calibration assumptions, misalignment, covariance/error models, disturbance detection, gating, and observability limits.
- [ ] Add magnetometer simulator/emulator output and runtime config, including representative low-cost and industrial IMU-adjacent sensor examples.
- [ ] Add magnetometer measurement model and Jacobian support for yaw/attitude aiding where the selected magnetic-field model and scenario make the measurement meaningful.
- [ ] Add logging, plotting, and validation scenarios for stationary and moving magnetic aiding, including disturbed/rejected measurement cases.

## Pass 10.5: sensor scheduling

- [ ] Add explicit sensor scheduling and multi-rate behavior where runtime rates, sensor queues, and Navigator update cadence interact.
- [ ] Add additional sensor simulators and measurement models only when they provide distinct validation value beyond GNSS/IMU.
