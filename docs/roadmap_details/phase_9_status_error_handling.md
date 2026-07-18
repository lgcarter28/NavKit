# Phase 9 - Status and Error Handling

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase moves robust status/error handling earlier than the broader embedded-hardening work because later phases add enough complexity that bool returns and ad hoc failure paths will become risky.

## Pass 9.1: status/error design

- [ ] Define explicit status/error handling for numerical and data-quality failures in embedded-facing code.
- [ ] Decide where exceptions are acceptable in desktop app/sim code and where status-return APIs are required.
- [ ] Define the small status/result vocabulary needed by Navigator, filter, propagation, sensor, simulator, and runtime-validation seams without overbuilding a generic framework.

## Pass 9.2: estimator and measurement robustness

- [ ] Add innovation gating, fault detection, and measurement rejection policies.
- [ ] Add covariance health checks where practical, including symmetry and positive-semidefinite diagnostics.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.

## Pass 9.3: tests and diagnostics

- [ ] Add focused tests for failed/invalid input paths at core seams.
- [ ] Ensure status/error paths are visible in logs or runtime summaries where useful.
