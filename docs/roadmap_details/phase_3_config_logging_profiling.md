# Phase 3 - Configuration, Logging, Compiler Flags, Tests, and Profiling

**Status:** complete for the implemented configuration/logging/profiling foundation. Remaining cleanup and expansion items were moved to the active roadmap.

## Pass 3.1: configuration architecture

- [x] Introduced explicit core configuration vocabulary under `include/navkit/core/config`.
- [x] Moved domain-specific configuration concepts beside their consuming domains.
- [x] Added `docs/CONFIGURATION.md`.
- [x] Documented compile-time selected configuration, runtime JSON separation, and one-config-per-build-tree behavior.
- [x] Split reusable NavKit product configs from app composition configs under `config/compiletime`.
- [x] Added selected-config CMake/Python workflow support.
- [x] Added runtime config validation for app scenarios.
- [x] Added app-support configuration policy concepts.
- [x] Added standalone policy headers where the boundary is public or reused.
- [x] Reworked emulator binding/runtime vocabulary around clearer policy concepts.
- [x] Removed stale parallel IDs in favor of scoped sensor/emulator IDs where appropriate.
- [x] Added reusable initialization vocabulary and later split runtime startup into `pva_initialization` and `filter_initialization`.
- [x] Added PVA initialization support for random error, explicit error, no-error, and direct-value examples.
- [x] Added filter initial covariance support for compile-time defaults and runtime overrides.

## Pass 3.2: logging and analysis architecture

- [x] Added CSV schema utilities and log-product policy concepts.
- [x] Split concrete log products and payloads into focused headers.
- [x] Reworked `RunLogger` toward a generic product-composition facade.
- [x] Added logger policy concepts and tests.
- [x] Split runtime logging and plotting outputs into `data` and `figures`.
- [x] Added runtime-configurable log rates and scenario-specific output directories.
- [x] Added GNSS, IMU, filter-correction, nominal-estimate, truth, covariance, innovation, NIS, and dashboard plot support used by the current ECEF INS/GNSS scenarios.

## Pass 3.3: build, tooling, and workflow

- [x] Made Ninja the default local build generator through Python tooling.
- [x] Removed generator nesting from normal build directories.
- [x] Added config-rooted Debug/Release build directories.
- [x] Updated VS Code/debugging assumptions around prebuilt Debug trees.
- [x] Added `tools/run_scenario.py` as the one-liner sim-plus-plot workflow.
- [x] Removed obsolete/confusing simulation runner duplication.
- [x] Updated documentation for the current tooling shape.

## Pass 3.4: profiling and resource vocabulary

- [x] Added embedded-oriented profiling vocabulary.
- [x] Added compile-time profiling metadata and run manifest output.
- [x] Added timing/resource artifacts for the desktop simulation path.
- [x] Documented local clang-tidy as an explicit/CI-oriented workflow rather than normal agentic iteration.

## Phase 3 follow-forward

The remaining Phase 3 residue now lives in active roadmap passes for runtime hygiene, compile-time config decomposition, covariance floors, explicit-type audit, profiling/resource hardening, and documentation/API teaching material.
