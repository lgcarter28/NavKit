# Algorithm Specifications

This directory contains focused implementation-oriented algorithm specs.

- [`navigator_ecef_v1/`](navigator_ecef_v1/) defines the first concrete ECEF
  INS/GNSS navigator contract before implementation.
- [`imu_emulator_v1/`](imu_emulator_v1/) defines the first concrete IMU
  emulator/error-model contract used to generate raw increment inputs for the
  ECEF navigator.
- [`trajectory_generation_v1/`](trajectory_generation_v1/) defines the
  implemented trajectory frame, attitude-input, source-agnostic Guidance,
  Autopilot, Vehicle-response, explicit ballistic-coast, exact scheduling,
  ECI-plant, and diagnostics contracts used by simulation scenarios.

These documents are narrower than the broader mathematical reference in
[`../navigation_reference/`](../navigation_reference/). They should answer:

- what algorithm are we implementing first;
- what conventions and approximations are intentionally selected;
- what behavior must be tested before the implementation is trusted;
- what generalizations are deliberately deferred.
