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

## Pass 18.2: controlled-attitude point-mass intermediary

- [ ] Add a point-mass translational model with commanded acceleration and quaternion attitude response before full rigid-body physics.
- [ ] Express response dynamics in body angular-rate axes `p`, `q`, and `r`, with independently configurable response time constants and optional body-rate/body-angular-acceleration limits. Do not describe these as roll/pitch/yaw time constants.
- [ ] Use quaternion/rotation-vector control error dynamics; do not independently low-pass Euler angles.
- [ ] Add coordinated-turn behavior that derives bank from lateral guidance acceleration while vertical response counters gravity for the selected constant-altitude assumption.

## Pass 18.3: full six-DOF vehicle dynamics

- [ ] Add genuine rigid-body translation and rotation only with explicit mass, center-of-gravity, inertia, thrust, aerodynamic-force, moment, actuator, and atmosphere contracts.
- [ ] Integrate the full state with documented numerical methods and validation evidence; retain separate ECI/ECEF and body/local-level output transformations.
- [ ] Preserve a clean repository boundary so a dedicated guidance, control, or trajectory-modeling implementation can provide equivalent truth through `TrajectorySource` without making NavKit depend on it.
