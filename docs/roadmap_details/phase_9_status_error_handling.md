# Phase 9 - Status and Error Handling

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase moves robust status/error handling earlier than the broader embedded-hardening work because later phases add enough complexity that bool returns and ad hoc failure paths will become risky.

## Pass 9.1: status/error design

- [ ] Define explicit status/error handling for numerical and data-quality failures in embedded-facing code.
- [ ] Decide where exceptions are acceptable in desktop app/sim code and where status-return APIs are required.
- [ ] Define the small status/result vocabulary needed by Navigator, filter, propagation, sensor, simulator, and runtime-validation seams without overbuilding a generic framework.
- [ ] Migrate remaining silent value-return frame conveniences, including ECEF/LLA and ECEF/NED helpers that currently substitute zero or identity on failure, to explicit status propagation once logging, initialization, and simulator callers can surface those failures coherently.

## Pass 9.2: estimator and measurement robustness

- [ ] Extend the existing per-sensor GNSS chi-square innovation gate into
  reusable fault-detection and rejection policies for additional measurement
  models and richer fault modes.
- [ ] Connect the existing covariance symmetry and positive-semidefinite health
  diagnostics to explicit runtime status and fault handling where practical.
- [ ] Revisit attitude covariance reset mapping and covariance health diagnostics once richer attitude/error-state tests are in place.

## Pass 9.3: tests and diagnostics

- [ ] Add focused tests for failed/invalid input paths at core seams.
- [ ] Ensure status/error paths are visible in logs or runtime summaries where useful.
