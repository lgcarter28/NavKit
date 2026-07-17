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
- Candidate-first `MeasurementModelPolicy` concept with positive coverage for GNSS position, GNSS velocity, and barometer models plus negative compile-time tests.
- Candidate-first `NoisePolicy` concept with positive and negative compile-time tests.
- First-pass `FilterPolicy`, `SensorCollectionPolicy`, `UpdatePolicy`, and `NavigatorUpdatePolicy` concepts for Navigator orchestration boundaries.
- Measurement-statistics regression tests for accepted and rejected measurement updates.
- Design-intent testing guide plus focused coverage for ring-buffer overflow policies, sensor FIFO/noise behavior, CSV writer output/failure behavior, and stationary trajectory semantics.
- Linux-oriented coverage reporting through `tools/coverage.py` and a CI coverage artifact.
- Lightweight timing artifacts for stationary simulation and analysis runs, plus coarse Debug/Release executable/library size reports through `tools/resource_report.py`.
- Human-readable timing summaries through `tools/timing_report.py` and documented `navkit.timing.v1` artifact schema fields.
- Product-core embedded profiling vocabulary with enum profile points, fixed timing records, visualization metadata fields, clock/sink/profiler concepts, `NullProfiler`, `ScopedProfiler`, and deterministic concept/runtime tests.
- Coarse embedded profiling integration points for `KalmanFilter::observation_update` and `Navigator::process_measurements`, both defaulting to `NullProfiler`.
- Runtime-input validation for the selected stationary GNSS app composition, including required scenario sections, unsupported sensor/emulator sections, and numeric/vector shape checks.
- Generic `SimulationApp<Config>` support with app-configured sensor bindings, unsigned sensor IDs, emulator tuples, and tuple-derived runtime validation.
- Public `include/navkit/api/config` contracts for user-facing product config graphs, including `NavKitProductConfigPolicy`.
- App-side navigation initialization and transfer-alignment provider seams, including typed PVA/TXA startup and alignment vocabulary, deterministic and random PVA initialization providers, transfer-alignment sample validity flags, and validation for the selected stationary GNSS app config.
- Focused ECEF navigator v1 algorithm specification under `docs/algorithms/navigator_ecef_v1`, covering the first one-IMU quaternion mechanization, coning/sculling baseline, analytical covariance prediction contract, GNSS position/velocity antenna-lever-arm observations, runtime API contract, and validation gate.
- Focused IMU emulator v1 algorithm specification and first working simulator implementation, including a product-core IMU increment sample, ECEF-truth-derived ideal increments, deterministic gyro/accelerometer error-model parameters, seeded noise, bias random walk, quantization, runtime config parsing helpers, and equation-shaped tests.
- First ECEF INS propagation policy with rotation-vector/quaternion helpers, two-sample coning/sculling compensation, ECEF PVA nominal propagation, first-order `F_k`/`G_k` covariance prediction, symmetry-preserving covariance tests, and stationary truth/IMU closed-loop tests.
- Navigator typed IMU ingestion through `push_imu(...)`, fixed-capacity IMU buffering, propagation success reporting, and explicit stage-method coverage.
- Simulation app IMU runtime validation and app-configured IMU simulator selection for feeding generated IMU increments into the selected Navigator.
- Shared runtime cadence parsing for `rate_hz` or `dt_s`, plus a small IMU runtime helper that keeps IMU simulator initialization/generation details out of the central simulation loop.
- Compile-time medium-rate covariance cadence and bounded covariance-step history configuration for Navigator propagation products.
- Runtime-configurable console, truth, nav-estimate, and measurement-statistics logging cadences so high-rate simulation no longer forces high-rate file or console output.
- A `tools/run_scenario.py` workflow that runs a runtime scenario, can override output location/run name without mutating checked-in JSON, writes an effective runtime config, and then runs the standard analysis plots.

### Changed

- Reconciled README and setup documentation with the current implementation and build configuration.
- Updated the configured C++ language standard from C++20 to C++23.
- Registered the StateDef policy tests in the configured test executable.
- Ordered source mutation/checks before build and test verification.
- Refined estimator policy concepts by splitting standalone filter, filter-correction, and sensor-filter compatibility contracts; renamed measurement-model vocabulary to `MeasurementModelPolicy` and `Sensor::MeasurementModel_t`.
- Consolidated superseded TODO lists and early core design notes into the canonical roadmap before removing them.
- Constrained `KalmanFilter` on `StateDefPolicy`, injection policy, and reset policy boundaries.
- Constrained `KalmanFilter` observation-update and measurement-statistics methods on measurement-model policy compatibility.
- Constrained `Sensor<Id, Model, BufferSize, NoisePolicy>` on noise-policy compatibility while preserving fixed-capacity buffering.
- Completed the Phase 2 estimator-boundary refactor scope and explicitly deferred `SensorPolicy` until a Navigator-facing capability boundary exists.
- Constrained `Navigator` on current filter, sensor-collection, and update-policy capabilities.
- Clarified ADR-003 and agent guidance around valid C++ concept-definition syntax versus constrained template-parameter syntax.
- Reorganized public headers from the generic flat `core` bucket into structured product-core domain folders.
- Reorganized public headers under `include/navkit/core` as the reusable product-core boundary, with estimation domains under `core/estimation`, environment under `core/environment`, and simulation/IO kept outside core.
- Split the monolithic CMake library into `navkit_core`/`navkit::core` for reusable product-core code, `navkit_sim`/`navkit::sim` for simulator support, and `navkit_io`/`navkit::io` for desktop logging/file/JSON support, while keeping runnable executables under `apps/`.
- Split root CMake orchestration from product-boundary target definitions, moved header-only/interface target definitions under `cmake/targets`, kept compiled simulator target metadata beside simulator sources, and removed the dummy source file by modeling header-only core code as an `INTERFACE` target.
- Updated the documented development workflow to include changelog updates and README/SETUP reconciliation for user-facing behavior, layout, tooling, or workflow changes.
- Added a current architecture document and moved detailed target-boundary, namespace, source-layout, and target-kind rationale out of setup-oriented documentation.
- Added a dedicated configuration guide covering domain config concepts, concrete config slices, example config contracts, static-assert wiring, runtime-input separation, and the `NAVKIT_CONFIG` selection model.
- Aligned public namespaces with the product-core folder structure through the stable domain level: `navkit::core::estimation`, `navkit::core::environment`, `navkit::core::frames`, `navkit::core::models`, `navkit::core::units`, and `navkit::core::containers`.
- Elevated compile-time configuration cleanup, Release/Debug compiler-flag hardening, static-analysis posture, runtime profiling/resource evidence, and intentional coverage strategy into the next immediate roadmap phase.
- Clarified the roadmap distinction between product-core compile-time configuration and runtime app input bundles such as `config/runtime/navkit_sim/...` scenario files.
- Replaced vague `core/common` configuration with explicit `core/config` headers for foundational types, narrow configuration capability concepts, and default configuration slices.
- Moved estimator-specific configuration concepts for sensor buffer capacity and measurement-statistics availability beside the estimation domain while keeping `core/config` focused on shared scalar/time configuration vocabulary.
- Moved concrete app/product compile-time configuration examples out of public NavKit headers and into `config/compiletime`.
- Added `NAVKIT_CONFIG` CMake selection with a generated `navkit/SelectedConfig.hpp` alias and `tools/build.py --navkit-config` forwarding.
- Added `tools/build.py --build-dir`, selected-config CMake presets, and stricter `NAVKIT_CONFIG` validation for multi-config development.
- Added centralized NavKit-owned target warning profiles, CI warnings-as-errors, embedded-oriented Release optimization settings, Release CI build verification, and `tools/build.py` compile-check switches.
- Added Linux Debug `clang-tidy` static analysis to CI and made the local tidy wrapper require a valid compilation database instead of silently running without build flags.
- Clarified that clang-tidy is intentionally a CI gate and not part of the normal local agentic development loop.
- Preserved stationary GNSS timing and resource reports as CI artifacts without making wall-clock timing a brittle pass/fail gate.
- Default-enabled timing artifact updates for build and test wrappers, with opt-out flags for quiet or artifact-free commands.
- Made build, test, simulation, and analysis wrappers print concise timing summaries by default after updating `timing.json`.
- Made build and resource-report wrappers display coarse executable/library size summaries by default after writing resource artifacts.
- Moved `navkit_sim` runtime JSON inputs from `apps/navkit_sim/configs` to `config/runtime/navkit_sim`.
- Removed stale root example placeholder directories and documented that future architecture domains should not be represented by empty folders.
- Split compile-time configs into reusable NavKit library configs under `config/compiletime/navkit` and app composition configs under `config/compiletime/apps`, with a generic selected-app launcher for `navkit_sim`.
- Moved reusable NavKit product configs under `config/compiletime/navkit/products` with product-local namespaces and role-based internal type names.
- Expanded `ConfigApi.hpp` into the shared product-config include for common core graph machinery.
- Clarified that same-named NavKit and app compile-time config files are expected when separated by ownership directories, and documented how runtime JSON links to the selected app/NavKit composition.
- Replaced the bespoke stationary GNSS app runner with the generic simulation app loop while preserving stationary GNSS log/profile behavior.
- Moved runnable NavKit product graph aliases into reusable NavKit configs and collapsed app configs to `NavKit` plus explicit `EmulatorBindings`.
- Renamed the profiled reusable NavKit GNSS config to `ProfiledEcefInsGnss.hpp` to match the app-level selected config name.
- Replaced app-facing sensor-index wiring with configured `Sensor::Id` values, emulator-owned stream IDs, explicit `(Emulator, Sensor)` bindings, and tuple helpers for ID-based lookup.
- Replaced derived `MeasurementModels` config aliases with explicit `MeasurementStatisticsTuple` aliases keyed by configured sensor types.
- Split app-support policy concepts into standalone headers and constrained simulation app configs, emulator bindings, emulator runtime plumbing, runtime validation, measurement models, and Navigator sensor processing on their real concept boundaries.
- Reorganized `include/navkit/app_support` into ownership-oriented subdirectories for app config, emulation, runtime input, initialization, logging, profiling, and trajectory support.
- Split concrete IO log products and reusable log payload wrappers into focused `log_products` and `log_payloads` headers, and removed the unused `RunLogProducts.hpp` umbrella.
- Changed the default build directory convention so Python wrappers and presets derive build trees from the selected compile-time config header, preventing different `NAVKIT_CONFIG` builds from sharing one generated selected-config tree.
- Added a `KalmanFilter::MeasurementStatisticsTuple_t` class-level alias for consistency with the other filter type aliases.
- Split profiling, sensor-tuple, emulator-binding, product-config, and runtime-config-validation headers so public contracts stay separate from helper/trait machinery.
- Removed unused profiling and sensor-tuple umbrella headers after replacing internal users with narrower includes.
- Documented the config API include boundary in agent and architecture guidance, keeping `ConfigApi.hpp` focused on shared product graph vocabulary and exposed defaults.
- Tightened `SensorCollectionPolicy` around real NavKit sensors and moved ID/tuple lookup helpers out of public config headers.
- Moved Navigator policy compatibility checks to a dedicated header and simplified KalmanFilter measurement-statistics storage naming.
- Replaced Navigator's update-policy template-template parameter with an explicit concrete `NavigatorUpdate` policy alias in reusable NavKit configs.
- Moved app-side emulator binding vocabulary to `EmulatorBinding.hpp`, added focused trajectory-provider and measurement-statistics logging helpers, and slimmed `SimulationApp` orchestration.
- Extracted run settings, filter initialization, and emulator runtime processing out of `SimulationApp`, replaced dummy-object statistics dispatch with type-level logging, and collapsed one-field GNSS buffer config wrappers in reusable product configs.
- Renamed compile-time config constants such as GNSS sensor IDs and buffer sizes to snake_case while keeping type aliases in PascalCase.
- Refactored stationary simulation logging so `RunLogger` coordinates composable log-product adapters while app compile-time configs explicitly select the logger type.
- Added payload-specific log-product concepts and CSV schema helpers so logging adapters expose explicit serialization boundaries.
- Reworked `RunLogger` into a compile-time log-product tuple faÃ§ade with typed payload dispatch, selected-product metadata emission, and generic app-support measurement-statistics logging.
- Moved logger composition into app compile-time configs, added logger lifecycle/payload/product-access concepts, and made `RunLogger<...>` the generic log-product tuple faÃ§ade.
- Removed public `MeasurementStatisticsTuple` aliases from reusable product configs; `KalmanFilter` now derives filter-owned diagnostics storage from the configured `Sensors` tuple and exposes `measurement_statistics_available<Sensor>()`.
- Added sensor diagnostics configuration and disabled-statistics coverage while keeping measurement statistics keyed by configured sensor type.
- Installed Ninja through bootstrap as a local compile-database convenience for optional Windows clang-tidy diagnostics.
- Exposed sensor diagnostics aliases in reusable product configs and moved profiling clock metadata onto the selected clock type to avoid config metadata drift.
- Made Ninja the default generator for Python build/test/sim/tidy wrappers and rooted default build directories by generator, build type, and selected compile-time config.
- Parameterized the GNSS position-update log product on its selected measurement-statistics stream so matrix dimensions and metadata come from the configured payload instead of hard-coded schema constants.
- Split the Navigator propagation seam into explicit strapdown-integration and covariance-prediction hooks, added `Navigator::update()` as the normal orchestration call, and moved the simulation app loop to that API while preserving current GNSS-only `NoOpPropagation` behavior.
- Corrected the truth-log metadata convention for `q_eb` so it matches the ECEF-to-body attitude used by trajectory truth and IMU derivation.
- Tightened the IMU emulator API and truth schema: `TruthSample` now carries only trajectory truth, truth logs no longer include IMU-derived acceleration/angular-rate fields, reusable quaternion/skew helpers live under core math, and IMU generation uses explicit `bool`/out-parameter failure handling instead of exceptions or `std::optional`.
- Selected stationary GNSS product configs now use the first ECEF INS propagation policy instead of `NoOpPropagation`, while `NoOpPropagation` remains available for measurement-only products.
- Split the selected ECEF INS state-space definition into explicit nominal and error layouts: `DefaultInsStateDef` now aggregates `DefaultInsNominalStateDef` with `AttQuat` and `DefaultInsErrorStateDef` with `AttRotVec`, so propagation, covariance, Jacobians, and injection use the correct vector dimensions at their boundaries.
- Extended MSVC NavKit-owned compile options with `/bigobj` so template-heavy selected-config application translation units compile reliably on Windows.
- Corrected the propagation/filter ownership boundary: propagation policies now operate on configured state definitions, build discrete covariance inputs, and leave `KalmanFilter` responsible for applying covariance propagation.
- Reduced the v1 INS state definitions to bias-only PVA/IMU-error states by removing stale gyro and accelerometer scale-factor segments.
- Moved reusable coning/sculling, planet-rate, gravity-gradient, quaternion/RPY, and simulation random-draw helpers toward their owning domains instead of keeping them inside the first ECEF INS and IMU simulator implementations.
- Moved filter initial covariance into a selected immutable NavKit product config value using `InitialCovariance<StateDef>` and `diagonal_initial_covariance<StateDef>()`, tuned the current examples to variance values corresponding to 50 milli-deg/s gyro bias and 100 micro-g accelerometer bias, swept app-support runtime parsing to remove hidden scenario fallback defaults, and added runtime JSON override validation for configured initial covariance, including raw full-state forms and a frame-aware PVA plus remaining-error-state diagonal form.
- Renamed the repo-level simulation wrapper from `tools/run_first_sim.py` to `tools/run_sim.py`, clarified that `--navkit-config` locates a compile-time-configured build tree rather than changing runtime behavior, and factored runtime JSON merge/output override handling into reusable tool helpers.

### Removed

- Removed the public `SensorGraphConfigPolicy` helper and the `MeasurementModelsFromSensors_t` derivation path.

- Placeholder `imu_gnss_straight_line.json` runtime config until the corresponding simulation path is real and validated.
- Removed the obsolete `python/navkit_analysis/run_sim.py` package helper; simulation launching now belongs to repo-level tools.

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

