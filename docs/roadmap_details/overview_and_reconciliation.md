# Roadmap Reconciliation Notes

This file preserves the high-level reconciliation decisions from the original long-form roadmap. It is history, not the active backlog. Use `docs/ROADMAP.md` for current work.

## Current reconciliation outcome

- [x] The roadmap now reflects the current repository reality: NavKit has a working desktop GPS/GNSS-aided ECEF INS simulation path.
- [x] The active roadmap was shortened to a verified baseline, active passes, and grouped future backlog.
- [x] Detailed completed-phase history was moved out of the active roadmap into `docs/roadmap_details/`.
- [x] Unfinished items from old phases were moved into the active roadmap or removed when obsolete.
- [x] Phase-detail files for completed phases now contain completed bullets only.

## Preserved architectural reconciliation decisions

- [x] The old task to move WGS-84 constants into `Earth.hpp` is superseded by the generic `navkit::core::environment::Wgs84` policy direction.
- [x] Environment-policy work established planet/gravity concepts, CRTP bases, frame tags, WGS-84, Moon, Mars, spherical gravity, J2 gravity, and environment tests.
- [x] Estimator policy boundaries now exist for state definitions, injection, reset, measurement models, noise, filters, filter-sensor interaction, sensor collections, update policies, navigator updates, propagation, logging, initialization, and app configuration where currently useful.
- [x] Public headers are organized by product boundary first, then engineering domain.
- [x] CMake targets separate reusable product core, simulator support, IO support, app support, and executables.
- [x] Runtime scenario files are app inputs, not embedded product-core configuration.
- [x] Compile-time app configs select one NavKit product graph per build tree.
- [x] Debug and Release build folders are config-rooted through the Python tooling.
- [x] The supported language standard is C++23.
- [x] Desktop timing, profiling, and simulation-analysis artifacts are available for current workflows.

## Active backlog handoff

The active backlog is intentionally not duplicated here. The following kinds of old unchecked items were consolidated into `docs/ROADMAP.md`:

- runtime hygiene and IMU error randomization;
- advanced analysis/restart initialization;
- trajectory-source expansion;
- transfer alignment and startup aiding;
- sensor model expansion;
- estimator validation and consistency;
- navigation math, frame convention, and covariance hardening;
- Monte Carlo and analysis automation;
- latent measurement replay;
- robust estimation and embedded readiness;
- explicit-type audit and documentation work.
