# Phase 4 - Navigator Propagation Seam

**Status:** complete and superseded by the working ECEF INS/GNSS implementation.

## Pass 4.1: Navigator propagation seam

- [x] Added a propagation seam to `Navigator`.
- [x] Added propagation policy vocabulary.
- [x] Preserved existing measurement-update behavior while the propagation seam was introduced.
- [x] Established app-facing API direction for pushing IMU/sensor data and letting `Navigator` orchestrate processing.
- [x] Moved from a no-op seam to the later ECEF INS propagation implementation.

## Phase 4 follow-forward

Potential future `NavigatorPolicy` work remains intentionally deferred until a real external consumer needs a stable Navigator-as-a-policy boundary. Current follow-up work is tracked in the active roadmap under robust status/error handling, latent measurement replay, sensor scheduling, and documentation/API teaching material.
