# Phase 7 - Trajectory Provider and Timebase Expansion

**Status:** completed-pass history and future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase expands truth generation and scheduling after Monte Carlo exists, so new trajectories can immediately become repeatable analysis cases.

## Pass 7.1: timebase and multi-rate scheduling

- [x] Added the product-core time vocabulary: versioned `Timestamp` with first-field wire version, explicit `TimeScale` (`Monotonic`, `Utc`, `Gps`, `Tai`), unsigned normalized second/nanosecond fields, and non-negative `Duration`. `core::Time_t = double` remains the physical/mechanization scalar; no `std::chrono` time system was introduced.
- [x] Migrated public truth, IMU, GNSS/measurement, statistics, initialization, and logging-gate boundaries to `Timestamp`. Boundaries validate version, scale, ordering, and nanoseconds; integer duration subtraction/borrow is explicitly converted to `Time_t` only for math. Timestamp serialization is documented as field-wise rather than raw struct bytes.
- [x] Implemented exact phase-stable multi-rate scheduling with 64-bit sample indices and canonical rational rates, eliminating repeated rounded `dt_s` accumulation in stationary truth generation, GNSS cadence, and logging. The v1 event gate uses the first available producer sample at/after a due time; arbitrary-time truth interpolation remains Pass 7.3.
- [x] Kept mutually exclusive JSON `rate_hz`/`dt_s` input compatibility and canonicalized both forms once to rational rates, with `rate_hz` documented as the preferred representation for repeating decimal periods.
- [x] Added focused timestamp/rate regression coverage: borrow and scale/order rejection, 600 Hz long-duration phase stability, incommensurate cadence ordering, and JSON rate/period canonicalization.

## Pass 7.2: time vocabulary header hygiene

- [x] Split the core time vocabulary into minimal public headers: `TimeTypes.hpp`,
  `Timestamp.hpp`, `Duration.hpp`, `RationalRate.hpp`, and `RationalSchedule.hpp`.
  `Time.hpp` remains a convenience umbrella only; production consumers include the
  narrowest needed header.
- [x] Added readable storage aliases (`Seconds`, `Nanoseconds`, `SignedSeconds`,
  `SampleIndex`, and `Samples`) in `TimeTypes.hpp`. The aliases document storage
  units without introducing premature strong-unit wrappers or `std::chrono`.
- [x] Standardized absolute timestamp fields and variables on `t`, `t_<modifier>`,
  `s`, and `ns`; scalar physical intervals remain `dt_s`. Public `Timestamp` and
  `Duration` store normalized `s`/`ns` fields, while logging and JSON retain
  descriptive scalar-unit names such as `time_s` and `rate_hz`.
- [x] Kept ordinary elapsed-time handling strictly non-negative. A future
  `SignedDuration` should use `SignedSeconds s` with normalized unsigned `ns`
  using POSIX-style floor semantics (for example, `-0.2 s` is `{.s = -1, .ns =
  800'000'000}`) only when latency/replay or timestamp-offset consumers require it.

## Pass 7.3: trajectory source abstraction

- [ ] Add a trajectory-source abstraction so generated trajectories and CSV/playback trajectories feed the same downstream hooks. Do not create a separate playback driver unless the shared trajectory-provider path cannot express the required replay behavior.
- [ ] Add queryable/interpolated truth sampling so consumers can request truth at arbitrary timestamps without forcing all downstream processing to run at the trajectory generation rate. Keep truth generation/system rate separate from truth logging rate, and use this seam to schedule IMU emulation at a rate distinct from truth generation.
- [ ] Add richer trajectory initial-condition parsing and documentation for ECEF and local-level position, velocity, attitude, and angular-rate conventions.

## Pass 7.4: scenario trajectory expansion

- [ ] Add a simple ballistic trajectory: stationary launch-pad initialization, optional transfer-alignment window, initial heading/pitch definition, and a simple axial body-x boost profile before ballistic/coast behavior. Keep this intentionally simple before adding aero or guidance complexity.
- [ ] Add a constant-altitude, constant-speed trajectory on the curved Earth rather than flat-Earth kinematics.
- [ ] Add calibration-maneuver trajectories: horizontal S-turn, vertical S-turn, and bank-left/bank-right excitation for observability and calibration studies.
- [ ] Add a basic waypoint trajectory with simple bank-to-turn behavior once coordinate, attitude, and trajectory-source contracts are stable.

## Pass 7.5: reusable trajectory math cleanup

- [ ] Move remaining reusable Earth-rate, triad-calibration, frame-transform, and trajectory-provider conversion helpers into their owning core math/frames/environment locations instead of leaving them buried in trajectory or simulator code. Concrete examples to sweep include `earth_rate_e_radps`, `nonorthogonality_matrix`, `misalignment_matrix`, and the frame-transform helpers currently living in trajectory/simulator implementation files.
- [ ] Define and test the minimum coordinate operations needed by PCPF/ECEF mechanization, local-vertical measurements, NED plots, and future local-level mechanizations.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code, algorithm docs, JSON schemas, and plot labels.
- [ ] Ensure the selected planet and gravity policies are threaded through physics code wherever Earth-specific constants still leak into reusable math.
- [ ] Add straight-line, constant-turn, and short GNSS-outage validation scenarios for the existing ECEF INS/GNSS path.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.
