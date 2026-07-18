# Phase 7 - Trajectory Provider and Timebase Expansion

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase expands truth generation and scheduling after Monte Carlo exists, so new trajectories can immediately become repeatable analysis cases.

## Pass 7.1: trajectory source abstraction

- [ ] Add a trajectory-source abstraction so generated trajectories and CSV/playback trajectories feed the same downstream hooks. Do not create a separate playback driver unless the shared trajectory-provider path cannot express the required replay behavior.
- [ ] Add queryable/interpolated truth sampling so consumers can request truth at arbitrary times without forcing all downstream processing to run at the trajectory generation rate. Keep truth generation/system rate separate from truth logging rate.
- [ ] Revisit runtime timebase/rate representation for non-terminating update intervals such as 600 Hz. Consider `std::chrono::duration`-style integer tick/fraction representations, rational rate descriptors, or embedded-style fixed-point whole-fraction timebases so multi-rate scheduling does not depend on accumulating rounded decimal `dt_s` values.
- [ ] Add richer trajectory initial-condition parsing and documentation for ECEF and local-level position, velocity, attitude, and angular-rate conventions.

## Pass 7.2: scenario trajectory expansion

- [ ] Add a simple ballistic trajectory: stationary launch-pad initialization, optional transfer-alignment window, initial heading/pitch definition, and a simple axial body-x boost profile before ballistic/coast behavior. Keep this intentionally simple before adding aero or guidance complexity.
- [ ] Add a constant-altitude, constant-speed trajectory on the curved Earth rather than flat-Earth kinematics.
- [ ] Add calibration-maneuver trajectories: horizontal S-turn, vertical S-turn, and bank-left/bank-right excitation for observability and calibration studies.
- [ ] Add a basic waypoint trajectory with simple bank-to-turn behavior once coordinate, attitude, and trajectory-source contracts are stable.

## Pass 7.3: reusable trajectory math cleanup

- [ ] Move remaining reusable Earth-rate, triad-calibration, frame-transform, and trajectory-provider conversion helpers into their owning core math/frames/environment locations instead of leaving them buried in trajectory or simulator code. Concrete examples to sweep include `earth_rate_e_radps`, `nonorthogonality_matrix`, `misalignment_matrix`, and the frame-transform helpers currently living in trajectory/simulator implementation files.
- [ ] Define and test the minimum coordinate operations needed by PCPF/ECEF mechanization, local-vertical measurements, NED plots, and future local-level mechanizations.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code, algorithm docs, JSON schemas, and plot labels.
- [ ] Ensure the selected planet and gravity policies are threaded through physics code wherever Earth-specific constants still leak into reusable math.
- [ ] Add straight-line, constant-turn, and short GNSS-outage validation scenarios for the existing ECEF INS/GNSS path.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.
