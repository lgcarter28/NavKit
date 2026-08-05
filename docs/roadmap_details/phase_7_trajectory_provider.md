# Phase 7 - Trajectory Provider and Timebase Expansion

**Status:** Passes 7.1 through 7.14 are complete.

This phase expanded truth generation and scheduling after Monte Carlo so new trajectories immediately became repeatable analysis cases. It now continues with the low-fidelity Guidance, Autopilot, and Vehicle loop that makes generated maneuver dynamics physically useful for navigation studies.

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
- [x] Completed and documented richer trajectory source conventions. Stationary initialization accepts ECEF/local-level position, velocity, quaternion/RPY/DCM attitude, and angular-rate forms. Runtime `w_nb_b_degps` is converted before applying `w_ib_b = C_e2b w_ie_e + C_n2b w_en_n + w_nb_b`, including local transport rate; CSV sources accept strict ECEF PVA/quaternion rows and optionally supplied body IMU rates.

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

- [x] Added a simple boost/coast ballistic profile with a stationary launch-pad dwell, ECEF/local-level initial position/velocity/attitude, a fixed body-x boost, and ECEF gravity/Coriolis coast. The dwell supplies early truth for a future transfer-alignment provider; transfer alignment itself remains a later Phase 13 capability.
- [x] Added a constant-altitude, constant-speed curved-Earth profile that preserves WGS-84 ellipsoid height while following a local-level great-circle path.
- [x] Added horizontal S-turn, vertical S-turn, and bank-left/bank-right calibration profiles for observability and calibration studies.
- [x] Added a basic bank-limited waypoint profile using local-level waypoint targeting, plus reusable runtime component and full scenario JSON inputs for each generated profile.
- [x] Added direct profile and runtime-validation regression coverage, a Debug scenario check, and Release end-to-end checks for all generated scenarios.

## Pass 7.7: reusable trajectory math cleanup

- [x] Moved reusable Earth-rate, triad-calibration, geodetic/ECEF, local-level, rotating-frame, attitude, and trajectory-conversion math into their owning core math/frames/environment locations instead of leaving it buried in trajectory or simulator code.
- [x] Defined and tested the coordinate operations needed by the ECEF mechanization, ECI trajectory generation, local-vertical measurements, NED plots, and future local-level mechanizations.
- [x] Confirmed position, velocity, acceleration, attitude, angular-rate, and specific-force frame conventions in code, the trajectory algorithm reference, JSON schemas, metadata, and plot labels.
- [x] Kept selected WGS-84 planet/gravity behavior explicit at the physics boundaries used by the first generated profiles.
- [x] Added straight/curved flight profiles and a short GNSS-outage validation scenario for the existing ECEF INS/GNSS path.
- [x] Corrected attitude covariance reset mapping and added covariance health diagnostics and regression coverage.

## Pass 7.8: frame-explicit attitude input conventions

- [x] Accepted exactly one configured attitude payload in every supported representation/frame direction: `q_*`, `dcm_*`, and degree-based runtime `rpy_*_deg` for `e2b`, `b2e`, `i2b`, `b2i`, `n2b`, and `b2n`.
- [x] Converted every accepted input at the app-support boundary to the canonical passive body-to-ECEF quaternion `q_b2e`; downstream simulation and Navigator-facing truth consume that canonical form.
- [x] Added reusable frame-pair conversion utilities with required context: ECEF position for NED forms and timestamp plus the selected uniform Earth-orientation convention for inertial forms.
- [x] Documented runtime `rpy_start2end_deg = [roll, pitch, yaw]` as aerospace 3-2-1 yaw-pitch-roll composition encoding the same passive `C_start2end`, including why inverse Euler forms are not componentwise sign negations.
- [x] Added positive conversion and negative ambiguity/context regression coverage for all 18 supported attitude input forms.

## Pass 7.9: ECI trajectory integration and velocity-aligned attitude

- [x] Made ECI Cartesian position, velocity, and acceleration the canonical dynamic-truth integration state; geodetic integration remains a helper path rather than the dynamic default.
- [x] Added reusable runtime-selected `SemiImplicitEuler` and `TrapezoidalPredictorCorrector` translation methods, explicit profile selection, validation, tests, and documented numerical assumptions.
- [x] Added reusable quaternion endpoint integration and converted ballistic, constant-altitude, calibration, and waypoint profiles to ECI integration with ECEF/local outputs derived at the boundary.
- [x] Added and tested velocity-aligned local-level attitude with body forward along velocity, body down referenced to NED down, zero-roll default behavior, coordinated-bank override, and degenerate-geometry rejection.
- [x] Added shallow simulation-only `FlightControlModel` and `VehicleResponseModel` virtual boundaries with typed payloads, `std::unique_ptr` ownership, and validated runtime construction.
- [x] Kept lifecycle explicit: initialize from runtime configuration and initial truth, advance once per trajectory step, and expose diagnostics without leaking concrete implementation types into `SimulationApp` or `TrajectorySource`.
- [x] Added runtime-configured cascaded first-order rotational command/control and vehicle response in body `p`, `q`, and `r`, followed by per-axis rate limits and quaternion integration. Zero time constant means exact response for that stage.
- [x] Added the analogous body-frame specific-force command/control and vehicle-response cascade, followed by per-axis acceleration limits, ECI transformation, gravity addition, and realized ECI integration. IMU truth consumes realized response.
- [x] Implemented `FirstOrderFlightControlModel` and `FirstOrderVehicleResponseModel` behind the narrow runtime interfaces, preserving a replacement seam for later higher-fidelity models.
- [x] Logged commanded and realized velocity, specific force, body rate, and attitude plus tracking errors and applied limits; added command-versus-response inspection plots.
- [x] Kept commanded profiles distinct from integrated realized truth and added stationary/turning response regressions and limit diagnostics.
- [x] Documented the selected ECI/ECEF Earth-orientation convention, truth acceleration, attitude-rate, output frames, guidance/response ownership, and integration assumptions.

## Pass 7.10: barebones trajectory truth inspection

- [x] Added a small CSV/Plotly trajectory-truth log and plot suite for generated and CSV scenarios; HDF5 optimization remains Phase 20 work.
- [x] Logged and plotted canonical ECEF and derived ECI position, velocity, acceleration, body-to-reference attitude, and attitude rate versus time.
- [x] Logged and plotted LLA position; NED velocity, acceleration, attitude, and attitude rate; and body-frame velocity, acceleration, and specific force with frame/unit/convention metadata.
- [x] Made all five trajectory products independently runtime rate-configurable and verified the products with ballistic, constant-altitude, calibration bank, calibration horizontal S-turn, calibration vertical S-turn, and waypoint bank-to-turn scenarios.

### Pass 7.10 verification evidence

- [x] Full Debug build and test suite passed after source formatting and copyright checks.
- [x] Full clang-tidy completed with warnings treated as errors.
- [x] Release build and six single-run dynamic scenarios completed, each producing the standard analysis suite plus seven interactive trajectory figures.
- [x] Six production Monte Carlo campaigns completed with 1,000/1,000 successful runs each and generated HDF5 bundles, aggregate covariance/error plots, consistency dashboards, and reports.
- [x] The campaigns exposed valuable Phase 8 work: GNSS position/velocity NIS was generally near unity and many marginal state-family metrics were credible, but several dynamic profiles showed substantial full-state joint-NEES inconsistency. Phase 7 therefore establishes repeatable evidence, not a blanket estimator-consistency claim.

## Pass 7.11: source-agnostic low-fidelity guidance, autopilot, and plant loop

- [x] Wrote and implemented the low-fidelity command/response contract:
  `GuidanceModel` consumes a source-agnostic vehicle-state estimate and emits
  inertial acceleration guidance plus local-level bank/mode commands;
  `AutopilotModel` consumes the same estimate plus high-rate IMU observations
  and emits body `p/q/r` commands; `VehicleResponseModel` owns realized body
  rate and non-gravitational specific-force response; the trajectory plant
  alone integrates truth in ECI.
- [x] Kept truth, Navigator estimates, and real/HIL measurements outside
  Guidance and Autopilot implementations. `SimulationApp` selects a typed
  `navigation_estimate` or `truth_passthrough` control-state source at runtime.
- [x] Added stateful runtime-selectable Guidance modes for launch-pad,
  boost/launch-program, gravity turn, free inertial, constant-altitude,
  calibration, and waypoint trajectories without introducing a generic gain
  bag.
- [x] Replaced profile finite-difference command realization in the dynamic
  path with direct Guidance commands and explicit body/inertial conversion at
  the plant boundary.
- [x] Added launch-pad and velocity-alignment guards, physical support-force
  handling, launch-rail consistency validation, configured launch-attitude
  fallback at low speed, and thresholded velocity alignment.
- [x] Added explicit endoatmospheric gravity-turn and free-inertial ballistic
  modes. A configured speed guard holds the launch program through the
  low-speed singularity, powered boost velocity-aligns after that guard, and
  gravity turn commands zero non-gravitational force while Autopilot tracks
  the velocity vector.
- [x] Added exact integer-compatible physics, Autopilot, Guidance, IMU,
  covariance, and aiding schedules while retaining full-rate IMU strapdown.
- [x] Routed actual configured simulated IMU increments into Autopilot and
  implemented a fixed-capacity moving interval average as
  `sum(delta_theta_ib_b) / sum(dt_s)`.
- [x] Enabled closed-loop navigation feedback by default and retained
  truth-passthrough as an explicit controlled-study override.
- [x] Split runtime trajectory diagnostics into Guidance and
  Autopilot/Vehicle products with NED/body views, body-frame tracking errors,
  Euler-angle attitude products, and retained detailed limiter flags in data.
- [x] Added interactive relative-NED and LLA 3-D products plus ECEF, ECI, NED,
  body, Guidance, Autopilot, Vehicle, and tracking-error dashboards.

### Pass 7.11 verification evidence

- [x] Copyright and formatting checks pass; Debug and Release builds pass; the
  complete Debug test target passes; repository-wide clang-tidy passes with
  warnings treated as errors. The final full tidy proof run took 1,171.5 s on
  the local Windows machine.
- [x] The standalone trajectory-generation LaTeX reference builds to 22 pages
  with no undefined references or citations.
- [x] Release single-run validation and complete standard/trajectory plots
  pass for ballistic, constant-altitude, calibration bank, horizontal S-turn,
  vertical S-turn, and waypoint bank-to-turn under
  `output/analysis/pass_7_11/individual_trajectories`.
- [x] Ballistic holds the configured 30-degree launch program through boost,
  reaches approximately 386 m apogee, transitions through zero flight-path
  pitch, and descends under gravity without force/rate limiting.
- [x] Constant-altitude and all three calibration references remain bounded,
  hold their configured speed/altitude targets after startup, produce the
  intended maneuver excitation, and show no limiter activation.
- [x] Waypoint bank-to-turn holds approximately 99--101 m/s through its turns
  and transitions to a stable terminal continuation after final-waypoint
  acceptance rather than repeatedly pursuing the point behind the vehicle.
- [x] Six post-refactor Monte Carlo campaigns completed with 500/500 successful
  runs each (3,000/3,000 total) under
  `output/analysis/pass_7_11/monte_carlo_500`. Every campaign produced a full
  HDF5 bundle, aggregate reports, consistency reports, and 45 interactive HTML
  products. Total campaign wall time was 10,314.7 s.
- [x] Final normalized GNSS position/velocity NIS remained between 0.939 and
  1.015 and within the campaign mean bounds. Full-state joint NEES remains
  materially inconsistent for several dynamic profiles even when many
  marginal state families are credible; this is explicit Phase 8 diagnosis
  evidence, not a blanket estimator-consistency claim.

## Pass 7.12: trajectory/estimator correctness and low-fidelity dynamics follow-up

This pass resolved the following evidence-backed findings before Phase 8
formalizes validation thresholds:

- [x] Made C++ and Python error products consistently use
  truth-minus-estimate, including cross-covariance-sensitive joint NEES, with
  direct known-state regression coverage.
- [x] Corrected same-epoch sequential GNSS position/velocity processing so
  each accepted update injects and resets before the next residual is
  evaluated. A combined GNSS position/velocity observation remains separate
  future sensor-model work.
- [x] Completed GNSS antenna-velocity truth and Jacobian handling with measured
  IMU angular-rate context, Earth-rate and gyro-bias lever-arm terms, a
  non-zero three-axis baseline lever arm, independent deterministic
  position/velocity random substreams, and finite-difference regression tests.
- [x] Restored rotating-Earth consistency by adding the required centrifugal
  acceleration and gradient terms to trajectory/IMU truth generation and ECEF
  strapdown propagation.
- [x] Adopted midpoint attitude as the v1 approximation to the
  interval-average delta-velocity transform, reconciled coning/sculling
  ownership and interval grouping, and documented the choice in the IMU and
  Navigator algorithm references.
- [x] Preserved ECEF ownership for the default symmetric attitude initial
  covariance while transforming declared runtime local-level covariance at the
  initialization position, with direct projection coverage.
- [x] Corrected full-rate coning/sculling execution in the application path:
  paired increments are retained until complete, aiding is deferred while a
  pair is incomplete, and an explicit finalization path flushes a terminal
  singleton and partial covariance interval.
- [x] Smoothed and bounded the low-fidelity Guidance/Autopilot transitions,
  added maximum-bank and command-filter configuration, improved ballistic
  launch-to-gravity-turn handoff and low-speed velocity-alignment behavior,
  propagated through ground impact, and added desired-body-rate feedforward
  without a v1 integral term.
- [x] Expanded frame-explicit trajectory diagnostics and documented the
  intentional high-fidelity IMU versus simplified filter-state trade,
  including the engineering provenance of the HG1700 Gauss-Markov assumptions.

### Pass 7.12 verification evidence

- [x] Focused C++ and Python regressions cover error signs, joint NEES cross
  terms, sequential update/reset behavior, GNSS lever-arm Jacobians and random
  streams, rotating-Earth IMU reconstruction, midpoint-sensitive ballistic
  dynamics, coning/sculling pairing/finalization, ground-impact behavior, and
  noncommuting desired-rate feedforward.
- [x] Debug and Release builds and the complete Debug C++ test target pass.
  The IMU emulator, ECEF Navigator, and trajectory-generation LaTeX references
  build without undefined references or citations.
- [x] Six Release single-run scenarios completed under
  `output/analysis/pass_7_12/individual_trajectories`: ballistic,
  constant-altitude, calibration bank-left/right, calibration horizontal
  S-turn, calibration vertical S-turn, and waypoint bank-to-turn.
- [x] Six 500/500 Monte Carlo campaigns completed under
  `output/analysis/pass_7_12/monte_carlo_500_final`. The waypoint campaign has
  a valid recovered consistency bundle; the constant-altitude campaign has a
  valid rebuilt analysis bundle after recovery from an interrupted package
  close.
- [x] The latest PVA-family and GNSS-NIS evidence is generally healthy.
  Constant-altitude and waypoint retain elevated full-state NEES associated
  with accelerometer-bias/cross-covariance behavior, while gyro-z remains
  weakly observable. These results are Phase 8 diagnosis inputs, not a blanket
  estimator-consistency claim.

## Pass 7.13: coordinated-turn and calibration-profile follow-up

- [x] Extended the nominal ballistic reference to a 180-second safety horizon,
  a 75-degree launch elevation, and approximately five g of powered body-X
  specific force. The generated trajectory still terminates at its first
  descending launch-height crossing rather than treating the safety horizon as
  its nominal end.
- [x] Replaced the ambiguous horizontal-calibration variants with matched
  horizontal S-turn references and explicit skid-to-turn and bank-to-turn
  realizations. The common Guidance profile produces the same NED horizontal,
  vertical, speed, and altitude-feedback command before the realization mode
  is selected.
- [x] Corrected local-level command conversion to retain NED transport,
  Earth-rate Coriolis, and centripetal terms before ECI plant integration.
  Removed the duplicate curved-Earth feedforward that had been applied on top
  of the transport contribution in constant-altitude Guidance.
- [x] Derived coordinated bank from the complete physical specific-force
  requirement, including altitude support, and resolved physical truth force
  through the realized plant attitude rather than the selected
  controller-facing attitude.
- [x] Added an explicit body-Y specific-force option. Its default preserves the
  complete commanded force; the checked-in coordinated-turn scenarios disable
  residual body-Y force so the turn is realized through bank while the matched
  skid-to-turn scenario retains side force.
- [x] Constrained the vertical S-turn to its intended fixed-heading plane and
  added a sustained, acceleration-based Dutch-roll calibration that combines
  a horizontal S-turn phase at the configured base frequency with vertical
  flight-path excitation at twice that frequency. The fixed sign and phase
  produce a front-view half-pipe: lowest altitude at the lateral center and
  highest altitude at both lateral extrema. Runtime `horizontal_amplitude_deg`,
  `vertical_amplitude_deg`, and `period_s` define the profile; the obsolete
  arbitrary vertical-phase-offset input is absent. Horizontal acceleration is
  realized through bank-to-turn, residual body-Y force is suppressed, and the
  resulting coordinated motion targets body `p`/`r` excitation in quadrature.
  The forcing does not decay and is not a damped natural lateral-directional
  mode.
- [x] Added explicit single-run and Monte Carlo scenarios for ballistic,
  constant-altitude, horizontal skid-to-turn, horizontal bank-to-turn,
  vertical S-turn, Dutch-roll bank-to-turn, and waypoint bank-to-turn.
- [x] Consolidated trajectory inspection into four frame-explicit kinematics
  dashboards, focused Guidance and Autopilot command/response figures, one
  combined Guidance/Control dashboard, tracking errors, and LLA/relative-local
  3-D views. The six-panel body view places inertial `v_ib_b` and `a_ib_b`
  beside rotating-Earth `v_eb_b` and `a_eb_b`, then shows
  `specific_force_ib_b` and `w_ib_b` (the code form of
  \(\omega_{ib}^{b}\)); the relative-local 3-D view uses North/East/Up with Up
  equal to negative NED Down and a data-proportional aspect. Redundant
  standalone Vehicle-response and nested-loop figures were removed.
- [x] Replaced the profile-local maneuver-onset envelope with a permanent
  Guidance-output filter for body-X/Y/Z specific force and NED bank. Runtime
  inputs select the initial per-channel time constants, zero is an exact
  bypass, and each Guidance mode may optionally select slower entry-window
  constants and a duration without resetting the filtered states. Entry
  constants shape the first sample in the new mode and automatically return to
  nominal values after the window; ordinary nominal changes begin on the next
  Guidance epoch. Logged raw/filtered body specific force is included in the
  combined Guidance/Control dashboard.

### Pass 7.13 verification evidence

- [x] Focused trajectory regressions cover matched skid/bank references,
  coordinated-force allocation, constant-altitude transport, vertical
  planarity, the sustained Dutch-roll half-pipe geometry, bank-to-turn
  realization, and quadrature body `p`/`r` excitation.
- [x] Release single-run generation and the standard trajectory-analysis suite
  completed for all seven updated profiles.
- [x] Seven 500-run campaigns completed 3,500 of 3,500 simulations and
  generated full HDF5 bundles, covariance products, and interactive NEES/NIS
  consistency suites.
- [x] GNSS position and velocity NIS remained close to their expected
  three-degree-of-freedom distributions across all profiles, with all
  measurements accepted.
- [x] The matched horizontal skid-to-turn and bank-to-turn campaigns produced
  comparable yaw and gyro-z performance under identical seeds. Bank-to-turn
  did not provide a statistically material observability advantage in this
  profile set.
- [x] Horizontal and Dutch-roll excitation drove yaw error well below the
  straight/level and vertical-only cases. Gyro-z covariance remained weakly
  contracting in every 60-second calibration profile.
- [x] Bank-coupled profiles exposed elevated full-state NEES while their
  marginal state families remained conservative. This points to
  cross-covariance/model consistency as a Phase 8 investigation rather than a
  trajectory-generation defect.
- [x] The ballistic campaign verified the powered boost and coast physics but
  also exposed variable impact times; shared-final-epoch statistics therefore
  represent only the surviving runs and steady-state metrics are the more
  reliable comparison.

## Final Phase 7 evidence structure

Phase 8 qualification should consume the complete sequence rather than only
the newest campaign:

1. the original six 1,000-run dynamic campaigns;
2. the six post-Pass-7.11 500-run closed-loop campaigns;
3. the six post-Pass-7.12 500-run correctness campaigns; and
4. the seven post-Pass-7.13 500-run profile campaigns, including the matched
   skid/bank comparison and sustained half-pipe Dutch-roll excitation; and
5. the seven post-Pass-7.14 Release single-run regressions proving that the
   generic runtime Guidance graph preserves the accepted trajectory products.

Each campaign remains evidence to diagnose against explicit Phase 8
thresholds, not a blanket estimator-consistency claim.

## Pass 7.14: runtime-configurable Guidance state machine and simulation-domain separation

- [x] Replaced profile-owned phase/transition `if` chains with one
  simulation-only runtime state-machine engine. JSON must define a unique
  initial state, named states, explicit transition criteria and priorities,
  terminal behavior, and state-entry configuration. The engine owns active
  state, elapsed-in-state time, and transition bookkeeping; it must not
  require a new C++ subclass for every scenario state.
- [x] Retained `GeneratedTrajectorySource` as the generic truth-plant
  orchestrator. It continues to own planned truth advancement, ECI integration,
  interpolation/storage, the Vehicle/plant response, and the persistent
  Guidance-to-control LPF. Guidance, Autopilot, and Vehicle configuration are
  instance-owned persistent defaults across state transitions, never global
  variables or reset-on-transition state.
- [x] Moved simulation-only Guidance behavior out of the trajectory folder.
  Implement selected algorithms as small testable namespaced functions with
  narrow typed inputs/outputs; construct only the stateful runtime wrappers
  needed for JSON-selected behavior. The state machine should compose an
  explicit translation-command pipeline (for example speed hold, altitude
  hold, horizontal/vertical excitation, waypoint following) and one
  attitude/bank-reference policy rather than accepting an unordered mutable
  command list.
- [x] Kept `GuidanceCommand` as the minimal Guidance-to-Autopilot contract and
  removed the redundant `Trajectory` prefix. It carries only filtered
  body-specific force and bank; state-machine execution flags and diagnostics
  live in separate producer-output structures. A focused `AutopilotState`
  adapter prevents the Autopilot from depending on trajectory-wide control and
  environment structs. Autopilot remains responsible for mapping guidance
  intent into its topology-specific attitude/body-rate command and emits a
  narrower plant-facing `VehicleCommand`.
- [x] Separated the two intentional first-order `p/q/r` stages in naming,
  documentation, diagnostics, and configuration: controller/actuator-side
  Autopilot response versus final plant-side realized vehicle response. Avoid
  accidentally treating the former as truth or double-counting time constants.
  Keep Vehicle/plant response and truth physics under `sim/trajectory`; move
  Guidance and Autopilot/control headers and implementations into their own
  simulation domains.
- [x] Made the Guidance-to-control LPF permanently available for every state.
  Nominal body-X/Y/Z specific-force and bank time constants apply continuously;
  a state may optionally provide a separate entry-window set of slower
  constants plus duration. Apply the entry values on the first sample of the
  entered state, preserve filtered values, and automatically restore nominal
  constants after the window. A zero time constant is an exact channel bypass.
- [x] Added strict runtime validation: finite rates/constants/limits; valid and
  unique state IDs; existing transition targets; a valid initial state; no
  orphan/unreachable required states; explicit handling of cycles/terminal
  states; deterministic priority for simultaneously true transitions; and
  required, frame/unit-explicit parameters for every selected Guidance block.
- [x] Migrated ballistic, constant-altitude, calibration, Dutch-roll, and
  waypoint scenarios to the generic graph; retain numerical and plot-regression
  coverage proving profile parity before deleting the profile-specific Guidance
  classes.

### Pass 7.14 verification evidence

- [x] The Debug default product build and full 248-case C++ test suite passed,
  including focused state-ID, transition-priority, cycle-policy, terminal,
  invalid-graph, and persistent/entry-window Guidance-filter regressions.
- [x] The Python trajectory-analysis suite passed all seven tests, and the
  trajectory-generation LaTeX reference compiled without undefined references
  or citations.
- [x] Release simulation and complete standard/trajectory plotting passed for
  ballistic, constant-altitude, horizontal bank-to-turn, horizontal
  skid-to-turn, vertical S-turn, Dutch-roll bank-to-turn, and waypoint
  bank-to-turn scenarios. Each run generated 17 CSV products and 32 figures;
  the combined CSV audit found no NaN or infinity values.
- [x] Ballistic logging recorded the configured generic graph sequence
  `launch_pad -> boost -> gravity_turn` at 0.00, 5.05, and 23.05 seconds. The
  six single-state trajectories remained in state index zero, confirming that
  profile-specific C++ Guidance classes are no longer required for scenario
  selection.
