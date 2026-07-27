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

- [x] Added `sim::TruthTrajectory` as the shared generated/CSV tabulated-truth container. The simulation app consumes it through the common source boundary, so CSV playback follows the normal initialization, IMU, emulator, Navigator, and logging path without a dedicated playback driver.
- [x] Added bounded arbitrary-time truth queries: ECEF position, velocity, and angular rate interpolate linearly while quaternion attitude uses SLERP. The IMU runtime now requests truth at its own exact rational sample timestamps, independent of native truth cadence; source logging remains separately cadence-gated.
- [x] Completed and documented richer trajectory source conventions. Stationary initialization accepts ECEF/local-level position, velocity, quaternion/RPY/DCM attitude, and angular-rate forms. `w_nb_b_radps` now applies `w_ib_b = C_e2b w_ie_e + C_n2b w_en_n + w_nb_b`, including local transport rate; CSV sources accept strict ECEF PVA/quaternion rows and optionally supplied body IMU rates.

## Pass 7.4: planned-time application loop and streaming trajectory sources

- [x] Added an explicit `application` rational-rate component to every runnable scenario. Runtime validation now requires it and rejects a cadence that is not an integer multiple of the configured IMU or any synthetic emulator rate.
- [x] Replaced the eager-only app trajectory boundary with the narrow simulation-only virtual `TrajectorySource` contract: `advance_to(const Timestamp&)`, bounded `query(const Timestamp&, TruthSample&)`, `t_start()`, `t_end()`, and completion status. Virtual dispatch remains confined to simulation/app support; product-core navigation remains static/policy composed.
- [x] Added lazy `StationaryTrajectorySource` and bounded `TabulatedTrajectorySource` implementations. The latter wraps generated/CSV `TruthTrajectory` storage and retains its interpolation behavior, so consumers request exact timestamps without a separate playback driver or cadence assumption.
- [x] Added the first planned-time producer path alongside the existing consumer-side rational schedule. Pass 7.5 follows up by separating these roles into dedicated timeline and schedule types.
- [x] Added `SimulatedClock` and `RealtimeClock` with the shared `wait_until(const Timestamp&)` contract. The simulation clock immediately adopts planned time; the real-time implementation maps planned monotonic time to a steady-clock deadline. Lateness/status enrichment remains Phase 9 work.
- [x] Split synthetic emulation into typed `prepare` and `publish` phases. Preparation can query truth, advance deterministic synthetic schedules/RNG state, and build typed output without exposing it; publication after `wait_until(t)` is the only path that mutates Navigator sensor/IMU buffers and writes measurement logs.
- [x] Reworked `SimulationApp` around the owned planned loop: initialize source/IMU/logger/clock at the epoch; prepare and publish the epoch measurements; then repeat `next(t_curr)`, source advance, prepare, clock wait, publish, and `Navigator::update()` until source completion.
- [x] Documented source, clock, and preparation/publication ownership in architecture/configuration references. Added deterministic tests for exact `next()` timestamps, rational application-rate compatibility, bounded source queries, no early IMU-buffer publication, and shared simulated/real-time clock boundary behavior. The default Debug scenario completed with 601 truth/nav samples, 6001 IMU samples, and 61 GNSS position/velocity samples over the 60-second run.

## Pass 7.5: app-support clock and rational-cadence role cleanup

- [x] Added the narrow app-support virtual Clock boundary: initialize(t_epoch), wait_until(t), and now(). SimulatedClock and RealtimeClock implement it; the application component now requires a runtime-selected simulated or realtime mode. Product-core NavKit remains clock-agnostic.
- [x] Moved planned-time initialization before epoch preparation/publication, so invalid application cadence fails before any queue or logging mutation.
- [x] Split SimulationApp failure handling into clock, IMU publication, generic emulator publication, and Navigator update ownership boundaries with distinct messages.
- [x] Split rational cadence roles. RationalSchedule now owns only consumer-side due(t) state; RationalTimeline owns planned timestamp next(t) production. Both use the same canonical rational-rate validity and exact sample-index timestamp arithmetic.
- [x] Added tests for virtual clock construction/mode parsing, supported realtime runtime configuration, invalid/missing clock rejection, and exact RationalTimeline phase behavior. Debug build/tests and the nominal 60-second planned-time scenario completed successfully.

## Pass 7.6: scenario trajectory expansion

- [ ] Add a simple ballistic trajectory: stationary launch-pad initialization, optional transfer-alignment window, initial heading/pitch definition, and a simple axial body-x boost profile before ballistic/coast behavior. Keep this intentionally simple before adding aero or guidance complexity.
- [ ] Add a constant-altitude, constant-speed trajectory on the curved Earth rather than flat-Earth kinematics.
- [ ] Add calibration-maneuver trajectories: horizontal S-turn, vertical S-turn, and bank-left/bank-right excitation for observability and calibration studies.
- [ ] Add a basic waypoint trajectory with simple bank-to-turn behavior once coordinate, attitude, and trajectory-source contracts are stable.

## Pass 7.7: reusable trajectory math cleanup

- [ ] Move remaining reusable Earth-rate, triad-calibration, frame-transform, and trajectory-provider conversion helpers into their owning core math/frames/environment locations instead of leaving them buried in trajectory or simulator code. Concrete examples to sweep include `earth_rate_e_radps`, `nonorthogonality_matrix`, `misalignment_matrix`, and the frame-transform helpers currently living in trajectory/simulator implementation files.
- [ ] Define and test the minimum coordinate operations needed by PCPF/ECEF mechanization, local-vertical measurements, NED plots, and future local-level mechanizations.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code, algorithm docs, JSON schemas, and plot labels.
- [ ] Ensure the selected planet and gravity policies are threaded through physics code wherever Earth-specific constants still leak into reusable math.
- [ ] Add straight-line, constant-turn, and short GNSS-outage validation scenarios for the existing ECEF INS/GNSS path.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.
