# Phase 18 - Guidance, Control, and Vehicle Dynamics

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase evolves the deliberately small NavKit-owned trajectory profiles into
an interoperable vehicle-truth contract without coupling NavKit to a future
dedicated trajectory-modeling, guidance, or control repository. The shared
boundary remains `sim::TrajectorySource` and its truth payloads.

## Pass 18.1: guidance/control signal-flow contract and algorithm document

- [ ] Write a complete algorithm/reference document with the signal-flow block diagram and frame/unit conventions:

  ```text
  guidance -> [acc_cmd, q_cmd]
      -> control: [acc_cmd, q_cmd] - [acc_curr, q_curr] = [acc_err, q_err]
      -> actuator/fin deflections for full 6-DOF
      -> [acc_rsp, q_rsp]
      -> integration -> sensors -> navigation -> optional feedback
  ```

- [ ] Define the lower-fidelity substitutions that preserve this contract: a kinematic trajectory may supply response truth directly; a controlled-attitude point-mass model omits fins/moments but still maps command to response; full six-DOF resolves controller output through actuators and forces/moments.
- [ ] Keep commands, response truth, sensor truth, and navigation estimates distinct. IMU truth must be generated from realized motion, never directly from a commanded acceleration or attitude.
- [ ] Extend the source-agnostic navigation-derived control-state contract with
  a valid body-angular-rate estimate. Inactive/free-inertial Autopilot modes
  must not silently substitute a zero rate merely because the selected
  navigation product currently publishes only PVA.
- [ ] Define causal planned-time behavior when application cadence is faster
  than or incommensurate with native trajectory physics cadence. Specify
  exact-time advance/query and terminal-epoch publication semantics so an
  application tick beyond a native sample neither exposes future truth nor
  terminates before the final prepared/published epoch; test both SWIL and
  future HIL timing cases.

## Pass 18.2: runtime-authored trajectory state-machine composition

- [ ] Replace scenario-specific generator assembly with one generic trajectory
  generator driven by a runtime-authored sequence of Guidance/Autopilot/plant
  states. Preserve the existing working concrete profiles as behavioral
  references while migrating; do not introduce the abstraction into the
  already-qualified Phase 7 implementation merely to anticipate this pass.
- [ ] Define a narrow simulation-only virtual state interface for mode entry,
  command production, transition evaluation, and exit. Keep Guidance,
  Autopilot, vehicle-response, and truth-integration ownership distinct rather
  than turning the state object into another application orchestrator.
- [ ] Let runtime JSON select each state's Guidance mode and its mode-owned
  parameters, enable/disable Guidance and Autopilot explicitly, and specify
  typed transition criteria such as elapsed time, altitude, speed, waypoint
  acceptance, or terminal impact.
- [ ] Parse the full graph before execution, instantiate the ordered state
  sequence, and reject unknown modes, invalid transition payloads, unreachable
  or orphaned states, ambiguous transitions, missing terminal behavior, and
  cycles that are not explicitly permitted.
- [ ] Document deterministic transition priority, exact transition epoch,
  state-entry initialization, held-command behavior between subsystem ticks,
  and validation diagnostics. Add JSON examples that reconstruct the existing
  ballistic, constant-altitude, calibration, and waypoint profiles without
  changing their truth contracts.

## Pass 18.3: controlled-attitude point-mass intermediary

- [ ] Add a point-mass translational model with commanded acceleration and quaternion attitude response before full rigid-body physics.
- [ ] Express response dynamics in body angular-rate axes `p`, `q`, and `r`, with independently configurable response time constants and optional body-rate/body-angular-acceleration limits. Do not describe these as roll/pitch/yaw time constants.
- [ ] Use quaternion/rotation-vector control error dynamics; do not independently low-pass Euler angles.
- [ ] Add coordinated-turn behavior that derives bank from lateral guidance acceleration while vertical response counters gravity for the selected constant-altitude assumption.
- [ ] Replace first-below-ground sample clamping with bracketed ground-impact
  root localization against the selected geodetic surface, producing a
  deterministic impact timestamp/state and covering low-energy departure
  cases.
- [ ] Shape waypoint course/bank transitions with continuous bounded
  command-rate behavior so waypoint acceptance cannot create a high body-rate
  impulse; retain configurable bank/rate/acceleration limits and regression
  plots.

## Pass 18.4: full six-DOF vehicle dynamics

- [ ] Add genuine rigid-body translation and rotation only with explicit mass, center-of-gravity, inertia, thrust, aerodynamic-force, moment, actuator, and atmosphere contracts.
- [ ] Integrate the full state with documented numerical methods and validation evidence; retain separate ECI/ECEF and body/local-level output transformations.
- [ ] Preserve a clean repository boundary so a dedicated guidance, control, or trajectory-modeling implementation can provide equivalent truth through `TrajectorySource` without making NavKit depend on it.
