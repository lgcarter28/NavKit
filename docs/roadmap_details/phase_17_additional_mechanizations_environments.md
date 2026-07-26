# Phase 17 - Additional Mechanizations and Environments

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase captures long-range navigation/environment expansion after the default ECEF INS/GNSS path, validation pipeline, sensor cleanup, latency handling, transfer alignment, profiling, embedded hardening, and advanced aiding phases are stable.

## Pass 17.1: additional mechanizations

- [ ] Add PCI/ECI mechanization support with a complete algorithm document before implementation.
- [ ] Add local-level and wander-azimuth mechanizations once the ECEF implementation and validation infrastructure are mature enough to compare behavior cleanly.
- [ ] Keep mechanization state definitions, frame conventions, and propagation-policy contracts explicit rather than hiding them behind vague runtime switches.

## Pass 17.2: expanded environment models

- [ ] Add atmosphere, magnetic-field, Earth-orientation, geoid, terrain, and aero/vehicle-dynamics policies driven by concrete use cases.
- [ ] Reuse existing planet, gravity, frames, and units infrastructure where it remains clear and zero-overhead.
- [ ] Add validation scenarios for each environment model before treating it as a supported product capability.

## Pass 17.3: multi-planet scenarios

- [ ] Extend multi-planet scenarios using the existing planet-policy direction.
- [ ] Keep planet-specific constants, gravity, rotation, frames, and scenario assumptions explicit in configs and documentation.

## Pass 17.4: propagation configuration and process-noise seams

- [ ] Revisit propagation configuration once a second concrete propagation model exists. Consider collapsing long propagation template parameter lists into a `PropagationConfigPolicy` selected by the product config, but keep model-equation construction owned by the concrete propagation implementation unless multiple implementations reveal a stable reusable process-model boundary.
- [ ] Keep the likely ownership split explicit: configuration selects physical constants, tuning values, and algorithm options; concrete propagation policies own `build_f_matrix(...)`, `build_g_matrix(...)`, `build_phi(...)`, `build_qd(...)`, and nominal-state propagation math.
- [ ] Avoid a generic "state transition matrix config" until it protects a real multi-model boundary. `F`, `G`, `Phi`, and `Qd` are model equations, not arbitrary user-facing tuning matrices.
- [ ] Revisit process-noise configuration as a generic driving-noise covariance over an explicit noise definition, rather than the current IMU-focused payload. The v1 model supports gyro/accelerometer white noise and gyro/accelerometer bias-drive noise; generalize when additional process-noise channels appear.
- [ ] Introduce a dedicated driving-noise definition when the implementation needs it, similar in spirit to state definitions but representing the noise vector that `G` maps into the error-state space:

  ```cpp
  struct EcefInsNoiseDef
  {
      using GyroWhite = Segment<0, 3>;
      using AccelWhite = Segment<3, 3>;
      using GyroBiasDrive = Segment<6, 3>;
      using AccelBiasDrive = Segment<9, 3>;

      static constexpr int N = 12;
  };
  ```

- [ ] Add a generic fixed-size process-noise covariance alias and config policy once the driving-noise dimension is explicit:

  ```cpp
  template<NoiseDefPolicy NoiseDef>
  using ProcessNoiseCov = Eigen::Matrix<Scalar_t, NoiseDef::N, NoiseDef::N>;

  template<typename Candidate, typename NoiseDef>
  concept ProcessNoiseConfigPolicy = requires {
      typename Candidate::Qc_t;
      { Candidate::process_noise } -> std::same_as<const typename Candidate::Qc_t&>;
  };
  ```

- [ ] Keep the distinction clear: `StateDef::Error::N` is the filter error-state dimension, while `NoiseDef::N` is the driving-noise dimension. `G` maps from `NoiseDef` space into `StateDef::Error` space, so those dimensions should not be assumed equal.
- [ ] Support diagonal and full process-noise covariance initialization using the same compile-time/static and runtime-override patterns used for initial covariance and covariance floors, once the generic shape is warranted.
