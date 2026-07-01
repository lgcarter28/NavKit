# Changelog

All notable changes to NavKit will be documented in this file.

The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

This project follows
[Semantic Versioning](https://semver.org/).

---

## [Unreleased] - YYYY-MM-DD

### Added

- A canonical current-state handoff and master roadmap.
- Repository-wide agent guidance and documentation indexes.
- Cross-platform environment bootstrap tooling and Linux/Windows GitHub Actions CI.
- Candidate-first `InjectionPolicy` and `ResetPolicy` concepts with positive and negative compile-time tests.
- Candidate-first `MeasurementPolicy` concept with positive coverage for GNSS position, GNSS velocity, and barometer models plus negative compile-time tests.
- Candidate-first `NoisePolicy` concept with positive and negative compile-time tests.
- First-pass `FilterPolicy`, `SensorCollectionPolicy`, and `UpdatePolicy` concepts for Navigator orchestration boundaries.
- Measurement-statistics regression tests for accepted and rejected measurement updates.

### Changed

- Reconciled README and setup documentation with the current implementation and build configuration.
- Updated the configured C++ language standard from C++20 to C++23.
- Registered the StateDef policy tests in the configured test executable.
- Ordered source mutation/checks before build and test verification.
- Consolidated superseded TODO lists and early core design notes into the canonical roadmap before removing them.
- Constrained `KalmanFilter` on `StateDefPolicy`, injection policy, and reset policy boundaries.
- Constrained `KalmanFilter` observation-update and measurement-statistics methods on measurement-model policy compatibility.
- Constrained `Sensor<Model, BufferSize, NoisePolicy>` on noise-policy compatibility while preserving fixed-capacity buffering.
- Completed the Phase 2 estimator-boundary refactor scope and explicitly deferred `SensorPolicy` until a Navigator-facing capability boundary exists.
- Constrained `Navigator` on current filter, sensor-collection, and update-policy capabilities.
- Clarified ADR-003 and agent guidance around valid C++ concept-definition syntax versus constrained template-parameter syntax.
- Reorganized public headers from the generic flat `core` bucket into structured product-core domain folders.
- Reorganized public headers under `include/navkit/core` as the reusable product-core boundary, with estimation domains under `core/estimation`, environment under `core/environment`, and simulation/IO kept outside core.
- Split the monolithic CMake library into `navkit_core`/`navkit::core` for reusable product-core code, `navkit_sim`/`navkit::sim` for simulator support, and `navkit_io`/`navkit::io` for desktop logging/file/JSON support, while keeping runnable executables under `apps/`.
- Split root CMake orchestration from product-boundary target definitions, moved header-only/interface target definitions under `cmake/targets`, kept compiled simulator target metadata beside simulator sources, and removed the dummy source file by modeling header-only core code as an `INTERFACE` target.
- Updated the documented development workflow to include changelog updates and README/SETUP reconciliation for user-facing behavior, layout, tooling, or workflow changes.
- Added a current architecture document and moved detailed target-boundary, namespace, source-layout, and target-kind rationale out of setup-oriented documentation.
- Aligned public namespaces with the product-core folder structure through the stable domain level: `navkit::core::estimation`, `navkit::core::environment`, `navkit::core::frames`, `navkit::core::models`, `navkit::core::units`, and `navkit::core::containers`.
- Elevated compile-time configuration cleanup, Release/Debug compiler-flag hardening, static-analysis posture, runtime profiling/resource evidence, and intentional coverage strategy into the next immediate roadmap phase.
- Clarified the roadmap distinction between product-core compile-time configuration and runtime app input bundles such as future `inputs/navkit_sim/...` scenario files.

### Fixed

- Removed stale feature-status claims and resolved the C++20/C++23 documentation mismatch.

---

## [0.1.0] - 2026-XX-XX

Initial pre-employment release establishing NavKit as an independently
developed software platform.

### Added

#### Repository

- Initial repository structure
- CMake build system
- Conan package management
- Cross-platform Python tooling
- VS Code development environment
- Clang-format configuration
- Clang-tidy configuration

#### Core

- Generic error-state Kalman filter
- Compile-time StateDef architecture
- Segment abstraction
- Ring buffer
- Generic measurement framework
- Generic sensor abstraction
- Policy-based filter architecture

#### Navigation

- Earth model
- Gravity model
- Coordinate frame utilities
- Unit framework (initial)

#### Simulation

- Trajectory generator
- GNSS simulator
- Truth generation

#### Analysis

- CSV logging framework
- Run logger
- Measurement statistics
- Covariance plots
- Innovation plots
- NIS analysis
- p-value analysis
- Innovation histograms

#### Testing

- Initial unit test framework
- Ring buffer tests
- Segment tests
- Navigation compile tests
- Measurement model tests

#### Documentation

- README
- SETUP
- Naming conventions
- Repository organization

### Changed

- Numerous architectural refinements during initial development.

### Fixed

- Build system compatibility
- Conan integration
- Template metaprogramming issues
- Logging architecture
- Plotting infrastructure
