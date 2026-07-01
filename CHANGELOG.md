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

### Changed

- Reconciled README and setup documentation with the current implementation and build configuration.
- Consolidated superseded TODO lists and early core design notes into the canonical roadmap before removing them.

### Fixed

- Removed stale feature-status claims and clarified the current C++20 versus target C++23 mismatch.

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
