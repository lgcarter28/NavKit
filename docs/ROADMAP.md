# NavKit Master Roadmap

This document is the canonical current-state handoff and working roadmap. It was reconciled from earlier planning notes and verified against the repository as it exists today.

The roadmap separates near-term, dependency-ordered engineering from longer-term product ideas. Checked items are verified in the current repository, not merely claimed by an older TODO.

## How to use this roadmap

- Work in order within the near-term phases unless a task is explicitly independent.
- Keep every phase compiling and testable. Preserve the stationary GNSS baseline unless a phase intentionally changes its behavior.
- Treat ADR-001, ADR-002, and ADR-003 as proposed architectural direction. Update their status or content when the implementation decision is actually accepted.
- Before starting a phase, turn it into a small implementation plan with named files, tests, and acceptance evidence.
- Mark an item complete only after its tests and documentation are part of the configured build or workflow.
- Treat configuration clarity, compiler/toolchain hardening, performance instrumentation, and intentional test coverage as first-class architecture work, not cleanup to defer until late embedded qualification.
- Keep owner-only legal, employment, account, release, and backup actions separate from engineering work.

## Reconciliation decisions

These decisions record conflicts and stale assumptions resolved during roadmap consolidation:

- The task to move WGS-84 constants into `Earth.hpp` is superseded. The implemented direction is the generic `navkit::core::environment::Wgs84` policy under `include/navkit/core/environment/planet`, consistent with ADR-002.
- Environment-policy Pass 1 is substantially implemented: planet/gravity concepts, CRTP bases, frame tags, WGS-84, Moon, Mars, spherical gravity, J2 gravity, and environment tests exist.
- The estimator policy refactor is partially complete. `StateDefPolicy`, `InjectionPolicy`, `ResetPolicy`, `MeasurementPolicy`, `NoisePolicy`, `FilterPolicy`, `SensorCollectionPolicy`, `UpdatePolicy`, and `NavigatorUpdatePolicy` exist. `KalmanFilter` is constrained on state, injection, reset, and measurement-model boundaries; `Sensor` is constrained on noise-policy compatibility; and `Navigator` is constrained on current filter, sensor-collection, and update-policy boundaries. Propagation remains future work.
- Public headers are organized by product boundary first, then engineering domain. `include/navkit/core` is the reusable product core, with estimation/navigation domains under `core/estimation`, environment models under `core/environment`, and reusable support domains such as `config`, `containers`, `frames`, `units`, and `models` also under `core`. Desktop simulation support remains under `include/navkit/sim`; desktop logging/file/JSON support remains under `include/navkit/io`.
- CMake targets now separate product boundaries: `navkit_core`/`navkit::core` is the reusable product-core interface library, `navkit_sim`/`navkit::sim` is the compiled simulator support library, `navkit_io`/`navkit::io` is the desktop logging/file/JSON interface library, `navkit_app_support`/`navkit::app_support` owns reusable selected-config/profile-export app plumbing, and runnable executables own their concrete application flow under `apps/`.
- `core/config` contains shared product-core compile-time configuration vocabulary such as foundational scalar/time aliases and `NumericConfigPolicy`. Domain-specific configuration concepts live beside the domains that consume them, following the general pattern `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`; estimator buffer and measurement-statistics configuration concepts are the first concrete examples.
- Runtime scenario files for applications are treated as app inputs, not core configuration. Repository-provided configuration now lives under the root `config/` tree: `config/compiletime/navkit/products/...` for reusable NavKit product configs, `config/compiletime/apps/...` for top-level executable composition configs, and `config/runtime/...` for JSON or other runtime inputs.
- App composition configs intentionally live in a separate tree from reusable NavKit library configs, so the same descriptive file name can exist in both places without ambiguity. A selected app config owns the link between `using NavKit = ...` and `using App = ...`; runtime JSON is then validated by the app-support layer against that compiled composition before the executable runs.
- Debug and Release build flags are now treated as explicit engineering products for NavKit-owned targets. CI enables warnings-as-errors, Release uses an embedded-oriented optimization profile, Linux Debug CI runs clang-tidy against the compilation database, and local agentic workflows intentionally do not run clang-tidy unless diagnosing that CI lane. Target-specific embedded toolchain flags remain future work until a target profile is selected.
- `GnssPosModel`, `GnssVelModel`, and `BaroAltModel` exist, but only GNSS position is integrated into the current simulation. The barometer model currently selects the third position component; it is not yet a general ECEF-to-local-vertical altitude model.
- The current trajectory generator supports only a simplistic stationary ECEF trajectory. It sets body rate to zero and therefore does not model Earth rotation correctly for stationary IMU truth.
- `ImuSimulator` and `BaroSimulator` are empty shells, and the IMU process model is a placeholder.
- Analysis already provides position error/covariance, innovation, NIS, p-value, mean p-value, and innovation histogram plots. More formal statistics and consistency tests remain future work.
- Desktop timing and coarse binary-size artifacts now exist for the stationary simulation/analysis workflow and CI artifact upload. Product-core embedded profiling vocabulary now exists under `include/navkit/core/profiling`, and the first coarse `Navigator` and `KalmanFilter` integration points are instrumented. Profile export/visualization and memory/resource budgets remain future work.
- The documented and configured language standard is C++23.
- `tests/test_state_def_policy.cpp` is included in the configured test target, so its positive and negative concept assertions compile in local and CI builds.

## Current verified baseline

- [x] CMake and Conan build with Eigen, nlohmann-json, and doctest.
- [x] Python wrappers support build, test, stationary simulation, analysis, formatting, and copyright checks.
- [x] Fixed-capacity `RingBuffer`, fixed-size state/covariance aliases, and named state segments exist.
- [x] Generic measurement update, Joseph-form covariance update, injection/reset hooks, sensor queues, Navigator orchestration, and measurement statistics exist.
- [x] Public headers use structured product-boundary/domain organization with no flat `include/navkit/core` catch-all.
- [x] Reusable product-core, simulator support, IO support, and app executable targets are separated as `navkit::core`, `navkit::sim`, `navkit::io`, and `apps/*`; header-only/template-heavy targets are modeled as `INTERFACE`.
- [x] Stationary GNSS-position simulation writes truth, measurement, navigation, metadata, manifest, and update-statistics logs.
- [x] Offline plotting is separated from the embedded C++ library.
- [x] Planet, gravity, frame, and basic unit/frame infrastructure exists.
- [x] ADRs document the proposed compile-time policy architecture.
- [x] ADR-003 documents the C++ syntax distinction between unconstrained concept definitions and constrained public template parameters.
- [x] Current architecture overview documents implemented product boundaries, namespaces, target kinds, and current data flow.
- [x] Repository setup, naming, founding, licensing, changelog, and copyright documentation exists.

---

# Phase 0 — Provenance and owner-controlled safeguards

**Goal:** Preserve clear independent provenance and recovery points. These actions are owner-managed and are not prerequisites for ordinary local refactoring unless timing or legal advice makes them so.

- [ ] Complete any required pre-existing-IP disclosure and retain the resulting records.
- [ ] Verify that the proprietary license and copyright notices express the intended ownership model.
- [ ] Create a documented pre-employment/pre-major-refactor release snapshot.
  - [ ] Choose the release/version name.
  - [ ] Create the tag and private release.
  - [ ] Archive the source snapshot and relevant documentation.
- [ ] Maintain independent backups beyond hosted Git remotes.
- [ ] Reserve desired project domains if still useful.

**Exit evidence:** dated release/tag, archived snapshot, verified backups, and owner-retained disclosure records where applicable.

---

# Phase 1 — Baseline integrity and documentation alignment

**Goal:** Make the existing baseline trustworthy before expanding the architecture.

## Build and test integrity

- [x] Add `tests/test_state_def_policy.cpp` to `navkit_tests` and verify its positive and negative assertions compile.
- [x] Audit test sources against `tests/CMakeLists.txt` so every intended test is built.
- [ ] Add a regression command/check for the stationary GNSS simulation and analysis pipeline.
- [x] Configure C++23 consistently in CMake, README, setup, and contributor guidance; verify the supported Debug toolchain.
- [ ] Establish a numerical baseline for stationary GNSS output or selected metrics so refactors can demonstrate unchanged behavior.

## Architecture records

- [x] Write a concise current architecture overview that distinguishes implemented code from target architecture.
- [ ] Review ADR-001 through ADR-003 and either accept them, revise them, or keep them Proposed with explicit unresolved questions.
- [ ] Reconcile README, `SETUP.md`, and this roadmap after those decisions.

## Automation

- [x] Add ordered Linux and Windows CI for source checks, C++23 Debug builds, tests, simulation, and headless analysis.
- [ ] Confirm the first hosted GitHub Actions run passes on both platforms.
- [x] Add clang-tidy selectively after the baseline build is stable.
- [x] Add basic coverage reporting after the test target accurately represents the suite; Phase 3 will turn coverage into a design-intent standard.

**Exit criteria:** all intended tests are configured, the baseline build/test/simulation workflow is reproducible, language-standard intent is explicit, and architecture documents no longer overstate implementation status.

---

# Phase 2 — Estimator policy boundaries

**Goal:** Complete the next estimator policy refactor pass without changing GNSS-only runtime behavior.

## Injection and reset

- [x] Define candidate-first `InjectionPolicy<Candidate, StateDef>`.
- [x] Define candidate-first `ResetPolicy<Candidate, StateDef>`.
- [x] Constrain `KalmanFilter` on `StateDefPolicy`, injection, and reset policies.
- [x] Preserve the current INS additive-injection sign convention and zero-error reset behavior.
- [x] Keep covariance reset explicitly a no-op until attitude-aware reset is designed.
- [x] Add valid and invalid compile-time policy tests.

## Measurement models

- [x] Define `MeasurementPolicy<Candidate, StateDef>` around dimension, fixed-size matrix types, noise context, observation, Jacobian, covariance, and Kalman-gain operations.
- [x] Decide whether `MeasurementModelBase` provides enough shared implementation to justify retaining the CRTP base.
- [x] Constrain `KalmanFilter::observation_update` and measurement-statistics storage at the public policy boundary.
- [x] Verify GNSS position, GNSS velocity, and barometer model conformance.
- [x] Add negative compile-time tests for missing types and operations.

## Sensors and noise

- [x] Define a noise-policy compatibility concept for a measurement model and measurement sample.
- [x] Constrain `Sensor<Id, Model, BufferSize, NoisePolicy>`.
- [x] Defer `SensorPolicy` until Navigator or another generic consumer has a real capability boundary that needs it.
- [x] Verify fixed-capacity, allocation-free behavior remains intact.

## Diagnostics

- [x] Decide whether `MeasurementStatistics` needs `StateDef` explicitly or can remain model-derived.
- [x] Preserve innovation, innovation covariance, measurement covariance, Jacobian, gain, NIS, timestamp, validity, and acceptance logging.
- [x] Add runtime regression tests for accepted/rejected update behavior and statistics.

**Exit evidence:** estimator templates are constrained at meaningful current public boundaries: state definition, injection, reset, measurement model, and sensor noise compatibility. Positive and negative concept tests are part of the configured `navkit_tests` target. Accepted and rejected measurement-statistics behavior has runtime regression coverage. `SensorPolicy` is intentionally deferred because no current generic consumer needs a dedicated sensor capability concept; Phase 4 will define the actual Navigator-facing sensor collection boundary. The stationary GNSS Debug build, tests, simulation, logs, and headless analysis were verified during the Phase 2 passes.

**Status:** complete for the current estimator-boundary refactor scope. Remaining propagation and Navigator orchestration work moves to Phase 4 after the configuration, compiler-flag, test-coverage, and profiling foundation in Phase 3.

---

# Phase 3 — Configuration, compiler flags, tests, and runtime profiling

**Goal:** Make the product-core configuration model, compiler/tooling posture, coverage strategy, and performance evidence explicit before adding more navigation physics and orchestration complexity.

## Compile-time configuration architecture

### Pass 3.1a — Initial core configuration vocabulary

- [x] Rename `include/navkit/core/common` to the explicit `include/navkit/core/config` product-core configuration domain.
- [x] Introduce foundational scalar/time aliases separately from named configuration policies.
- [x] Define the first narrow configuration concepts and a temporary `DefaultConfig` composition bundle for scalar/time aliases, sensor-buffer capacities, and measurement-statistics availability.
- [x] Relocate `navkit_sim` runtime JSON files into an explicit app-input location, separate from product-core configuration.
- [x] Add compile-time tests that valid configuration slices satisfy the intended concepts and intentionally invalid slices fail those concepts.

### Pass 3.1b — Configuration ownership cleanup

- [x] Keep `include/navkit/core/config` focused on configuration vocabulary shared broadly by product-core code, such as scalar/time types and any future truly cross-cutting core options.
- [x] Move domain-specific configuration concepts beside the domains that consume them once the domain boundary is clear, following the general pattern `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`; current examples include sensor buffer configuration near `core/estimation/sensor` and measurement-statistics configuration near `core/estimation/filter`.
- [x] Remove or demote library-owned concrete `DefaultConfig` as the universal product configuration once repository-provided app/product configurations exist outside public NavKit headers.
- [x] Document that concept policies distributed in domain folders define required capabilities, while concrete compile-time configurations are selected by applications or product builds.

### Pass 3.1c — Configuration guide and example-contract documentation

- [x] Add `docs/CONFIGURATION.md` as the human-facing map for the concept/policy configuration architecture.
- [x] Explain the core mental model: domain config concepts define local requirements; concrete compile-time configs compose named slices; aggregate product checks validate runnable graphs; CMake selects one config per build tree; runtime inputs remain separate.
- [x] Document the generic domain-specific policy location pattern, such as `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`, rather than implying config concepts only live under estimation.
- [x] Document that built-in concrete examples are the user-facing teaching surface, not a universal base class or internal default product configuration.
- [x] Describe `MinimalConfig` as a deliberately small example that shows required composition shape without implying it is a production target.
- [x] Document that runnable/product configs should avoid noisy duplicate slice-level `static_assert` blocks and usually end with one aggregate product-config check, while teaching configs and focused tests carry detailed concept examples.
- [x] Point readers to `tests/test_config_policy.cpp` for rigorous positive and negative examples, including invalid cases expressed as `static_assert(!Concept<Bad>)`.
- [x] Link `docs/CONFIGURATION.md` from `README.md`, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, `docs/README.md`, and `AGENTS.md`.

### Pass 3.1d — Root configuration tree

- [x] Create a root `config/` tree as the obvious place for repository-provided selectable configuration.
- [x] Add `config/compiletime/...` for C++ compile-time configuration headers, split into reusable NavKit library configs and app composition configs.
- [x] Add `config/runtime/...` for JSON or other runtime inputs consumed by desktop applications, simulators, and demos.
- [x] Move `navkit_sim` runtime JSON files to `config/runtime/navkit_sim/...` and update tools, apps, docs, and tests.
- [x] Add at least one plug-and-play app compile-time configuration for `navkit_sim`, such as `config/compiletime/apps/navkit_sim/StationaryGnss.hpp`, so a fresh clone can build and run without user-authored configuration.
- [x] Add README files or examples that show where new desktop, embedded-target, simulation, and test-fixture configurations should live.

### Pass 3.1e — CMake selected-config path

- [x] Add a CMake cache variable named `NAVKIT_CONFIG` for selecting exactly one compile-time configuration header per build tree.
- [x] Provide a good default `-DNAVKIT_CONFIG` value, for example `apps/navkit_sim/StationaryGnss.hpp` relative to `config/compiletime`, so ordinary clone/build/test workflows work without extra flags.
- [x] Keep `NAVKIT_CONFIG` orthogonal to `CMAKE_BUILD_TYPE`; Debug/Release chooses compiler mode, while `NAVKIT_CONFIG` chooses the top-level compile-time build configuration.
- [x] Generate a build-local selected-config header, for example `build/generated/navkit/SelectedConfig.hpp`, from a CMake template so generic applications can include one stable header.
- [x] Expose the selected compile-time configuration through a stable alias such as `navkit::selected_config::Config` or an equivalent clearly documented name.
- [x] Ensure selected-config include paths are applied to app or product targets that need them, not forced into `navkit::core` as a dependency on repository app configuration.
- [x] Update `navkit_sim` to remain config-agnostic in source and consume only the generated selected-config alias.

### Pass 3.1f — Multiple configurations and developer UX

- [x] Document the primary rule: one build tree selects one `NAVKIT_CONFIG`.
- [x] Support multiple configurations by using multiple build directories or CMake presets, not by making one executable dynamically switch among compile-time configurations.
- [x] Add CMake presets or documented wrapper examples that pair common build types and selected configs for convenience while keeping those axes independent.
- [x] Add a `tools/build.py` option such as `--navkit-config apps/navkit_sim/StationaryGnss.hpp` that forwards to `-DNAVKIT_CONFIG=...`.
- [x] Document how to add a new compile-time config header, how to select it with CMake or the build wrapper, and how to pair it with a runtime JSON input when an application needs one.
- [x] Reconcile `README.md`, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, and `AGENTS.md` so the default selected config, root config tree, and one-config-per-build-tree rule stay discoverable.
- [x] Keep reusable NavKit library configs and app composition configs in dedicated directories so app and library configs can share descriptive names without coupling their ownership.
- [x] Add runtime-input validation at the app-support boundary so missing scenario sections, unsupported sensor/emulator sections, and malformed JSON inputs fail early with clear diagnostics against the selected compile-time composition.

### Pass 3.1g — Generic simulation-app composition

- [x] Replace the bespoke `StationaryGnssApp::run()` shape with a generic `SimulationApp<Config>::run()` that owns the common application loop: load runtime input, validate it against the selected compile-time app/NavKit composition, create the runtime trajectory, construct configured emulators, push generated measurements into the matching NavKit sensors, process the Navigator, log outputs, and export profiling artifacts.
- [x] Move app sensor/emulator capability selection into app compile-time config tuples, now represented as `EmulatorBindings`, while keeping numeric values such as noise, covariance, seeds, rates, output paths, and run names in runtime JSON.
- [x] Replace the current `RuntimeConfigValidation` shape with generic app-support validation that is derived from the selected app compile-time tuples. Validation must support arbitrary combinations of emulators/loggers and must not be hard-coded to stationary GNSS.
- [x] Define a small `SensorEmulatorPolicy`/adapter contract that connects each app-side emulator to an explicit configured NavKit sensor alias plus a stable app/runtime sensor ID. Do not rely solely on model-type lookup because realistic configurations may include multiple sensors with the same model type, such as two GNSS receivers, dual barometers, or redundant IMUs.
- [x] Represent each configured NavKit sensor and app emulator stream with an unsigned `SensorId`, optionally named by config-local constants such as `primary_gnss_sensor_id = 0U`. App bindings explicitly state `(Id, Emulator, Sensor)`, and compile-time checks prove IDs are unique, every emulator target sensor exists in the selected NavKit sensor graph, and binding IDs match the configured `Sensor::Id`.
- [x] Remove stale placeholder runtime configs, including placeholder future-scenario JSON files, during this refactor unless they are converted into real validated examples.
- [x] Add compile-time tests for valid/invalid app composition concepts and runtime tests for missing emulator sections, extra unsupported sections, and app/NavKit capability mismatches.

### Pass 3.1h — Public config API surface and product graph aliases

- [x] Add an explicit public config API directory, for example `include/navkit/api/config`, as the front door for compile-time configuration contracts intended for end users.
- [x] Define `NavKitProductConfigPolicy` in the public config API. It should explicitly state the required aliases for a runnable/product NavKit configuration, including at least `StateDef`, `Sensors`, `Profiler`, `Filter`, `NavigatorUpdate`, and `Navigator`.
- [x] Define or expose additional public config API concepts only when users are expected to satisfy or assert them in concrete compile-time configs. Domain implementation concepts stay beside their consuming domain unless they graduate into the user-facing config API.
- [x] Keep the public API concepts as contracts and documentation for config authors; do not move every low-level policy concept into the API folder merely because it exists.
- [x] Move product graph aliases into reusable NavKit configs: state definition, sensor model aliases, concrete sensor aliases, `Sensors`, `Profiler`, `Filter`, and `Navigator`.
- [x] Decide whether `MeasurementModels` should be public, derived, or removed. The current direction is to remove it and require explicit `MeasurementStatisticsTuple` aliases keyed by configured sensors.
- [x] Collapse app configs so they no longer reconstruct NavKit sensors. App configs should select `NavKit`, define app-local unsigned sensor IDs, define `EmulatorBindings`, and select `SimulationApp<Config>`.
- [x] Simplify emulator binding machinery so the ID is the app/runtime key and the sensor target is an explicit NavKit sensor alias. `SimulationApp` derives the tuple index from `NavKit::Sensors` by `Sensor::Id`, so app configs avoid raw indices and do not search sensor bindings by potentially duplicated model type.
- [x] Update configuration docs and tests so users can find the public config concepts, see which aliases are required, and understand which aliases are local helper wiring rather than the public config contract.

### Pass 3.1i — Simplify diagnostics ownership and config graph policy boundaries

- [x] Remove the current public `SensorGraphConfigPolicy` shape. The public config API should not force derived aliases such as `MeasurementModelsFromSensors_t<Sensors>` or expose tuple-search/count machinery as user-facing architecture. Keep only public contracts that config authors are expected to satisfy directly.
- [x] Replace `MeasurementModels` with an explicit `MeasurementStatisticsTuple` alias in concrete NavKit configs. The tuple must be manually authored, for example `using MeasurementStatisticsTuple = std::tuple<MeasurementStatistics<PrimaryGnssSensor>>;`, so config readers can see exactly which configured sensor streams produce stored diagnostics.
- [x] Key `MeasurementStatistics` directly on the configured `Sensor` type, not on `SensorModel` and not through a new `MeasurementStatisticsConfig<Sensor>` wrapper. The sensor already carries `Sensor::Id` and `Sensor::Model_t`, which should be enough identity for duplicate same-model sensors.
- [x] Delete derived helpers such as `MeasurementModelsFromSensors_t` or any future `MeasurementStatisticsTupleFromSensors_t` unless a compelling, demonstrated need exists. Prefer explicit config tuples over metaprogramming convenience that hides product behavior.
- [x] Remove dead/aspirational config leaves such as `EnableMeasurementStatistics`. Diagnostic storage is configured by the explicit `MeasurementStatisticsTuple`; no separate boolean gate or filter diagnostics/settings bundle is needed until a concrete memory/performance requirement justifies it.
- [x] Slim `include/navkit/core/estimation/sensor/SensorId.hpp` and app-support identity headers so they define identity types and simple binding structs only. Move lookup helpers such as `SensorIndexFromId_v`, `SensorFromId_t`, `BindingIndexFromId_v`, and `EmulatorFromId_t` into a clearly named internal tuple/lookup utility header if they remain necessary.
- [x] Move generic tuple helper machinery out of domain headers such as `MeasurementStatistics.hpp`, `SensorGraphConfigPolicy.hpp`, and `Navigator.hpp`. Helpers like `tuple_contains_v`, `tuple_index_v`, and `NavigatorPolicyCompatibility` should live in small internal/detail headers or be removed by simplifying ownership.
- [x] Define a real `SensorPolicy` concept that proves a type is a NavKit sensor: configured ID, model alias, measurement alias, noise context, and queue operations. Then strengthen `SensorCollectionPolicy` so it means "tuple of sensors", not merely "tuple-like type".
- [x] Make the `Sensor` -> `SensorPolicy` -> `SensorCollectionPolicy` -> `Navigator` chain obvious in headers and tests. A reader should not need to reverse-engineer why `std::tuple<int, double>` is not a valid sensor collection.

### Pass 3.1j — Finish app orchestration cleanup and naming polish

- [x] Introduce a trajectory-provider seam so `SimulationApp` is not hard-coded to `StationaryTrajectoryConfig` or `TrajectoryGenerator::stationary`. Runtime trajectory selection should remain easy for stationary, straight-line, turning, scripted, and future Monte Carlo scenarios.
- [x] Move measurement-statistics logging out of `SimulationApp`. The app loop should not iterate `NavKit::MeasurementStatisticsTuple` or probe logger methods such as `log_gnss_pos_statistics`; logging adapters should own model-specific CSV/schema details.
- [x] Clarify the tuple ownership chain: `Sensors` are configured product-core sensor instances; sensor models own measurement math; the filter owns update math and diagnostic storage; logging adapters own serialization. Parallel tuples are acceptable when they are part of the explicit API configuration, manually authored, clearly named for their owner/purpose, and not silently derived through helper machinery.
- [x] Keep internal tuple helper machinery minimal and local. If a tuple relationship can be made obvious by an explicit config alias, prefer that over adding public `_v`/`_t` metaprogramming helpers.
- [x] Move `NavigatorPolicyCompatibility` and `navigator_policy_compatible_v` out of `Navigator.hpp` into a dedicated compatibility/helper header so the Navigator API header stays focused on orchestration.
- [x] Replace the current Navigator template-template update-policy parameter with an explicit concrete update-policy alias in NavKit configs, for example `using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;` followed by `using Navigator = core::estimation::Navigator<Filter, Sensors, NavigatorUpdate, Profiler>;`. Keep `UpdatePolicy` as the per-sensor update concept, and add a tuple-wide `NavigatorUpdatePolicy` concept that proves the selected update policy is valid for the configured filter and sensor collection.
- [x] Keep the new Navigator update boundary as a deliberate, useful abstraction: it should clean up `Navigator.hpp`, preserve the lower-level per-sensor `UpdatePolicy` checks, and avoid narrow one-off helper machinery leaking into concrete configs.
- [x] Simplify `KalmanFilter` statistics type aliases. Rename the public config alias from `MeasurementStatisticsConfigs` to `MeasurementStatisticsTuple` for plain-language clarity, expose `MeasurementStatisticsTuple_t` at the class level to match the other aliases, then remove unnecessary internal layers such as `MeasurementStatisticsConfigs_t` when they only restate the template parameter.
- [x] Rename `include/navkit/app_support/SensorId.hpp` to an emulator-binding-focused header, such as `EmulatorBinding.hpp`, because the file now owns app-side emulator/sensor binding vocabulary rather than the core sensor ID definition.
- [x] Trim concrete NavKit config static assertions to the reusable aggregate product-config check. The final alias names, such as `StateDef`, `PrimaryGnssSensor`, `Sensors`, `MeasurementStatisticsTuple`, `Profiler`, `Filter`, `NavigatorUpdate`, and `Navigator`, should stay clear enough that the config reads as the product graph rather than as a concept-test file.
- [x] Move reusable NavKit product configs under `config/compiletime/navkit/products`, including `MinimalConfig.hpp`, `StationaryGnss.hpp`, and `ProfiledStationaryGnss.hpp`. Product configs should use product-local namespaces so scenario identity comes from the file path and namespace, while local types use role-based names such as `NumericConfig` and `ProductConfig`. Export stable scenario-specific aliases from `navkit::config::navkit`, such as `StationaryGnssConfig` and `ProfiledStationaryGnssConfig`. Future reusable component configs, such as profilers, sensor bundles, filters, or navigator update choices, should live in their own component folders only after multiple real options exist, and their exported names should be descriptive because users select them directly.

### Pass 3.1k — Header boundary and helper split cleanup

- [x] Add durable config API inclusion guidelines to `AGENTS.md` and, if the rule is architecture-level enough, `docs/ARCHITECTURE.md`. `ConfigApi.hpp` should include shared public configuration vocabulary and defaults exposed by primary core template boundaries; concrete product configs should include `ConfigApi.hpp` plus the specific selected model, profiler, target, or component headers they use. Do not turn `ConfigApi.hpp` into a grab bag of every possible concrete component choice.
- [x] Split profiling policy vocabulary so the primary concepts and public types are easy to find. Separate clock, sink, scope, profiler, and sink-record trait/helper concerns, then retire the unused `ProfilePolicy.hpp` umbrella include.
- [x] Split `ScopedProfiler.hpp` so the scoped RAII profile record type and the profiler facade have obvious ownership and neither is buried under unrelated helpers.
- [x] Clarify the old `SensorTuple.hpp` umbrella. Split the public sensor-tuple concept/policy from tuple lookup/count traits into `SensorTuplePolicy.hpp` and `SensorTupleTraits.hpp`, then retire the unused umbrella include.
- [x] Clarify app emulator-binding headers. If `EmulatorBindingPolicy.hpp` mostly owns binding structs, tuple uniqueness, lookup, and validation helpers rather than an obvious `EmulatorBindingPolicy` concept/type, split policy/contract vocabulary from traits/helpers.
- [x] Split `NavKitProductConfigPolicy.hpp` so the public product-config concept stays readable as the API contract, while supporting detail checks live in a small detail/traits header.
- [x] Decompose `RuntimeConfigValidation.hpp` into smaller app-support headers for runtime key/schema definitions, JSON value parsing helpers, emulator runtime-key derivation, and validation orchestration. Keep the top-level validation entry point obvious.

### Pass 3.1l — Follow-on app/config simplification

- [x] Reduce `SimulationApp<Config>` back to orchestration only: load validated runtime input, obtain truth samples from a trajectory provider, run configured emulators, process the configured Navigator, and delegate logging/export details.
- [x] Replace dummy-object type dispatch such as `typename NavKit::MeasurementStatisticsTuple{}` with explicit type-level helper APIs, for example `log_measurement_statistics<typename NavKit::MeasurementStatisticsTuple>(logger, filter)`, so `SimulationApp` does not appear to pass empty runtime statistics state.
- [x] Keep trajectory category and trajectory parameters runtime-configurable. Swapping stationary, straight-line, turn, and future scripted trajectory families from JSON is intentionally useful and is an acceptable non-hot-path runtime polymorphism exception outside product-core embedded algorithms.
- [x] Collapse one-field public concrete config slices into aggregate config settings where they do not improve readability. Keep internal domain policy concepts available where useful, but do not force end users through standalone one-field types unless those types are independently reusable.
- [x] Refactor `RunLogger` toward composable log products/adapters while preserving existing stationary GNSS filenames, schemas, manifests, and downstream analysis compatibility during a focused IO cleanup pass.
- [x] Treat logging as a mixed compile-time/runtime concern at the first seam: app compile-time config selects the logger adapter type, while runtime JSON continues to select run name and output directory. Richer optional log-product enable/disable and verbosity selection remains a Phase 8 logging task.
- [x] Preserve existing stationary GNSS runtime behavior, file names, manifest contents, profile export behavior, and analysis compatibility while simplifying the app loop and logging boundary.
- [x] At the end of the pass, run the normal format/copyright/build/test workflow, then run local clang-tidy explicitly with `python tools/format.py --check --tidy --tidy-warnings-as-errors` and let it run long enough to collect full findings before deciding whether to fix or defer issues.

### Pass 3.2 — Log product concepts and payload boundaries

- [x] Add a narrow `LogProductPolicy<Candidate, Payload>` concept under `include/navkit/io` that validates the shared log-product lifecycle and the concrete payload-specific `log(payload)` operation. Do not force every log product into one fake common `log(...)` signature.
- [x] Introduce explicit log payload wrapper types for products whose natural inputs are more than one argument, such as nav-estimate logging and measurement-statistics logging. Payload wrappers should name the serialized boundary clearly instead of leaking helper argument lists through `RunLogger`.
- [x] Update existing concrete log products to satisfy `LogProductPolicy` with their actual payloads: truth samples, GNSS position measurements, nav-estimate payloads, and GNSS position update-statistics payloads.
- [x] Move generic CSV schema helpers such as matrix-header and matrix-value flattening out of `RunLogProducts.hpp` into a focused helper header such as `include/navkit/io/CsvSchemaUtils.hpp`.
- [x] Add compile-time tests proving each current concrete log product satisfies its intended `LogProductPolicy<Candidate, Payload>` instance, plus negative concept tests for missing lifecycle/schema/log operations.
- [x] Keep `RunLogger` as the coordinating façade and manifest owner for now. It should compose log products through typed payloads, preserve current stationary GNSS filenames/schemas/manifests, and avoid becoming a second logging framework.
- [x] Keep richer runtime optional log-product enable/disable, verbosity, schema migration, and model/state-derived generic logging in the later Phase 8 logging scope unless this pass uncovers a tiny prerequisite.

#### Pass 3.2a — Log product header organization

- [ ] Split concrete log products out of `RunLogProducts.hpp` into focused headers under `include/navkit/io/log_products/`: `TruthLogProduct.hpp`, `GnssPositionLogProduct.hpp`, `NavEstimateLogProduct.hpp`, and `GnssPositionUpdateLogProduct.hpp`.
- [ ] Split reusable payload wrappers into `include/navkit/io/log_payloads/`: `NavEstimateLogPayload.hpp` and `MeasurementStatisticsLogPayload.hpp`. Keep payloads beside concrete products only if they are truly private to one product; otherwise keep the dedicated payload folder so callers can name payload boundaries clearly.
- [ ] Keep `RunLogProducts.hpp` only as an optional narrow convenience umbrella if it has real value; otherwise replace includes with the specific product/payload headers and delete the umbrella.
- [ ] Preserve the existing namespaces and public include compatibility where practical, but prefer narrow includes in production code and tests so dependencies stay obvious.
- [ ] Add or update compile-time tests if needed so the split headers still expose all current concrete log-product policy checks.

#### Pass 3.2b — Generic compile-time RunLogger composition

- [ ] Replace the current hard-coded `RunLogger` member list with a generic `RunLogger<LogProducts...>` or equivalent tuple-based composition selected by app compile-time config. The default stationary GNSS logger should remain available through a clear alias so existing app configs stay readable.
- [ ] Keep the implementation simple and explicit: avoid broad tuple metaprogramming machinery, type-erasure, virtual dispatch, or a runtime registry. Use small local helpers only where they directly dispatch a typed payload to the matching product.
- [ ] Route logging by explicit payload type and `LogProductPolicy<Product, Payload>` conformance. If multiple products can consume the same payload, require an explicit decision instead of silently logging to all or guessing.
- [ ] Keep `RunLogger` responsible for output directory setup, product open/flush/close orchestration, run manifest ownership, and metadata file emission. Concrete log products own their CSV/schema details.
- [ ] Preserve stationary GNSS filenames, metadata schemas, run-manifest shape, profile export behavior, and downstream Python analysis compatibility.
- [ ] Keep runtime JSON responsible for run name and output directory in this pass. Optional runtime log-product enable/disable and verbosity remain Phase 8 work unless a tiny prerequisite falls out naturally.
- [ ] Add compile-time tests for generic logger composition and at least one negative case where a selected product cannot consume the requested payload.
- [ ] Add runtime or integration evidence by running the stationary GNSS sim and analysis pipeline after the generic logger refactor.

### Pass 3.3 — Navigation initialization and transfer-alignment boundary

- [ ] Replace direct truth/error-style filter initialization in app support with an explicit initialization boundary. Product-core navigation code must not know about truth, injected errors, or simulator-only perturbations.
- [ ] Introduce a core-facing `NavInitialization`-style message that represents only the initial navigation solution, not the full Kalman/filter state. It should carry PVA content such as position, velocity, attitude, timestamp, and `CovariancePva`; the filter/product maps that message into its internal state layout.
- [ ] Add an app-side `NavInitializationProvider` concept/config seam. It should produce the required one-time startup initialization message from runtime input, simulated truth, a saved state, external data, or future embedded inputs without changing Navigator construction.
- [ ] Rename current runtime initialization inputs away from filter-error language such as `initial_position_offset_m`. Prefer explicit initializer types such as `explicit_pva` for JSON-provided PVA/covariance and `truth_perturbed_pva` for simulator-only noisy initialization generated from truth.
- [ ] Keep Navigator construction separate from initialization. Construction wires the compile-time product graph; every runnable config calls an `initialize_navigator(...)`-style path with a `NavInitialization` message before normal updates.
- [ ] Add a separate optional transfer-alignment boundary. Transfer alignment is not construction and not the required initial PVA message; it is a timestamped aiding stream that active configurations may call through a `transfer_align(...)`-style path.
- [ ] Introduce a `TransferAlignmentProvider` concept/config seam for optional alignment aiding samples. Do not make the interface GNSS-specific: GNSS may be one aiding source, but transfer alignment should be framed around source-independent PVA aiding plus optional angular-rate and translational-acceleration aiding.
- [ ] Define a `TransferAlignmentSample` shape with PVA, timestamp, optional angular-rate/specific-force aiding fields where useful, and a matching `CovarianceTxa` representation. Keep optional TXA covariance handling simple and explicit first; only introduce compile-time capability flags if runtime optional fields become unclear or too dynamic for embedded-facing paths.
- [ ] Extend runtime JSON validation so initialization and transfer-alignment sections are checked against the selected compile-time app configuration. Missing required initialization inputs, unsupported initializer types, and disabled/unsupported transfer-alignment sections should produce clear errors.
- [ ] Keep truth/noise/error-distribution logic entirely in app/sim providers. Product-core code consumes typed initialization and transfer-alignment messages only.
- [ ] Preserve the current stationary GNSS demo behavior through a simple default initialization provider while migrating the naming and boundaries.
- [ ] Add focused tests for runtime JSON validation, required initialization presence, unsupported initializer/transfer-alignment types, and the happy-path stationary GNSS initialization provider.
- [ ] Document the lifecycle rule: construction is compile-time product wiring, initialization is required runtime nav-data input, transfer alignment is optional timestamped aiding input, and simulator truth/error/noise models stay outside product-core embedded algorithms.

## Compiler flags and static-analysis hardening

- [x] Audit current Debug, Release, and CI compiler flags for MSVC, Clang, and GCC where supported.
- [x] Define strict Debug diagnostics that catch likely correctness issues early without creating unreviewable warning noise.
- [x] Define Release optimization flags deliberately and document which optimization profile is used for performance evidence.
- [x] Add a Release build verification path to the normal workflow or CI once flags are stable.
- [x] Decide how clang-tidy, compiler warnings-as-errors, sanitizers, and platform-specific analysis tools fit into local and CI workflows. Clang-tidy is a CI gate, not a default local agentic workflow step.
- [x] Keep target-specific embedded flags separate from desktop development flags until an embedded toolchain/profile is selected.

## Coverage and design-intent tests

- [x] Review current test coverage by domain and identify meaningful gaps, not just line-count gaps.
- [x] Add coverage reporting once the configured test target accurately represents the intended suite.
- [x] Add tests that demonstrate intended extension usage for configuration, policies, models, sensors, filters, and Navigator orchestration.
- [ ] Continue positive and negative compile-time concept tests for each public policy boundary.
- [x] Add runtime tests for important incorrect-input or rejected-operation behavior where failure is expected and should be stable.
- [x] Document the testing standard: completeness, clarity, and design intent matter more than tests for tests' sake.

## Runtime profiling and resource evidence

### Pass 3.4a — Desktop workflow timing artifacts

- [x] Add simple script-level timing around build/test/demo/analysis commands without forcing desktop-only dependencies into `navkit::core`.
- [x] Record stationary simulation wall time and analysis wall time in a machine-readable artifact, such as `data/logs/<run_name>/timing.json`.
- [x] Include metadata needed for trend review: schema version, run name, selected `NAVKIT_CONFIG`, build type, command names, start/end timestamps or elapsed seconds, and tool version where practical.
- [x] Add executable/library size reporting for Debug and Release artifacts as an early coarse resource signal.
- [x] Add CI or tool-wrapper hooks that preserve timing/resource artifacts as uploaded artifacts without making stochastic or machine-dependent values brittle pass/fail gates.
- [x] Keep these desktop timing artifacts useful for future Monte Carlo runtime summaries and batch trade studies.

### Pass 3.4b — Embedded-ready profiling policy architecture

- [x] Add a zero-overhead-by-default profiling vocabulary under the reusable product-core boundary, likely `include/navkit/core/profiling`.
- [x] Define stable enum-based profile points instead of string-owned scope names for embedded-facing instrumentation, for example `ProfilePoint::NavigatorProcessMeasurements`, `ProfilePoint::KalmanObservationUpdate`, and future propagation/mechanization points.
- [x] Provide a `NullProfiler`/`NullProfileScope` that is the default and is expected to optimize away completely in production builds.
- [x] Define a profiler capability concept around static scope creation or scope-recording behavior so algorithms can be constrained without runtime polymorphism.
- [x] Define clock-policy expectations: target clocks provide a fixed `Tick` type and `now()` function. Future concrete clocks may include desktop steady clock, deterministic fake clock for tests, Cortex-M DWT cycle counter, and hardware timer counters.
- [x] Define sink-policy expectations: sinks record fixed-size timing records without allocation, exceptions, virtual dispatch, or string ownership. Future concrete sinks may include `NullSink`, fixed ring-buffer sink, memory-dump sink, SWO/UART/telemetry adapters, and desktop JSON/CSV adapters outside core.
- [x] Use fixed-size records such as `{ProfilePoint point, Tick start_tick, Tick elapsed_ticks}` or an equivalent representation that can be exported later without formatting in the hot path.
- [x] Expand profile records with visualization-friendly metadata such as sequence, parent sequence, depth, and flags while leaving ordering/nesting ownership to future profiler/sink policies.
- [x] Add unit tests with a deterministic fake clock and fake sink to prove scope entry/exit records elapsed ticks correctly.
- [x] Add compile-time tests that valid profiler/clock/sink policies satisfy their concepts and invalid policies fail them.

### Pass 3.4c — First algorithm integration points

- [x] Introduce profiler policy parameters only at coarse, stable hot-path seams first: `Navigator::process_measurements`, `KalmanFilter::observation_update`, and future propagation/mechanization update.
- [x] Keep the default selected configuration on `NullProfiler`; profiling must not affect default runtime behavior, allocation behavior, or public numerical results.
- [x] Avoid pervasive tiny-scope instrumentation until the first INS/mechanization hot path exists and measurement overhead is understood.
- [x] Keep desktop report formatting and file output outside `navkit::core`; embedded-facing code should only emit fixed records through policy sinks.

### Pass 3.4d - Profile sinks, export, and visualization path

- [x] Keep the embedded hot-path record format native, compact, fixed-size, and allocation-free; do not emit strings, JSON, file paths, or visualization-specific structures from `navkit::core`.
- [x] Add the first concrete core sink policies only after the algorithm hooks prove the record flow. Start with simple single-producer, non-thread-safe sinks: `NullProfileSink` and `RingBufferProfileSink` with explicit dropped-record or overwrite accounting.
- [x] Prefer reusing `navkit::core::containers::RingBuffer` for the first ring-buffer sink if its reject/overwrite semantics match profiling needs. If profiling needs diverge, add a narrow profiling-specific fixed buffer rather than complicating the generic container.
- [x] Keep RTOS, ISR-safe, multi-producer, lock-free, or atomic sinks out of the first implementation. Add them later as separate sink policies when a target profile and concurrency model require them.
- [x] Define an initial logical `navkit.profile.v1` export schema outside the hot path for desktop/offline use. Prefer JSON or CSV first so the schema can evolve before any binary ICD is frozen.
- [x] Put C++ outer-layer export adapters in `navkit::io`, not `navkit::core`. The first adapter is `ProfileCsvWriter`; Python handles Chrome Trace / Perfetto JSON conversion for now.
- [x] Add a fully configured profiled stationary GNSS sim selection that emits `profile.csv` through an embedded-style clock, scoped profiler, and fixed-capacity ring-buffer sink.
- [x] Add Python tooling that consumes exported profile records plus metadata and can emit a standards-friendly trace format, with Chrome Trace / Perfetto-compatible JSON as the preferred first visualization target.
- [x] Add Python summary views for count, total time, min/max/mean, p95/p99 where sample counts are meaningful, and percent-of-profiled-time by `ProfilePoint`.
- [x] Keep future NavKit-native plots optional; prefer exporting to established trace viewers before hand-building complex timeline/flame-chart UI.
- [x] Document required metadata for export: schema version, profile-point mapping, clock source, tick frequency or tick-to-time conversion, build/config identity, run name, dropped-record count, and record flags.
- [x] Defer binary profile dump and formal ICD work until real algorithm records and export metadata are proven by at least one integration pass.

### Pass 3.4e - Resource evidence and philosophy

- [ ] Decide how memory/resource evidence should be collected on desktop now and mapped to embedded targets later.
- [ ] Add allocation/resource checks where practical for fixed-capacity core paths, especially sensor queues and estimator update operations.
- [ ] Document the performance philosophy: track trends early, use enum/profile-policy hooks for embedded readiness, and set hard embedded budgets only when target profiles and clocks exist.

**Exit criteria:** configuration extension points are named and concept-tested; Debug/Release compiler flags and static-analysis expectations are documented and exercised by repository tooling; the test suite has an explicit coverage/design-intent baseline; and desktop timing/resource evidence is generated by repository tooling before the next major propagation/mechanization expansion.

---

# Phase 4 — Navigator and propagation seam

**Goal:** Introduce propagation as an orchestration capability without implementing full INS mechanization yet.

- [x] Define the minimum `FilterPolicy` actually required by Navigator.
- [x] Define `SensorCollectionPolicy` only around operations Navigator truly uses.
- [x] Formalize the existing update-policy capability.
- [ ] Define candidate-first `PropagationPolicy` using concrete prediction inputs rather than speculative APIs.
- [ ] Implement `NoOpPropagation` to preserve current GNSS-only behavior.
- [ ] Refactor Navigator to orchestrate prediction, sensor processing, and update policy application.
- [ ] Keep Navigator unaware of planet, gravity, and frame types; those belong inside propagation/mechanization configuration.
- [ ] Add valid and invalid compile-time tests for filter, sensor collection, propagation, and update boundaries.
- [ ] Verify the stationary GNSS numerical baseline remains unchanged with `NoOpPropagation`.

**Exit criteria:** Navigator accepts a propagation policy, the no-op configuration reproduces current behavior, and the public orchestration boundary is concept-tested.

---

# Phase 5 — Navigation physics and simulation contracts

**Goal:** Establish correct truth and sensor contracts before trusting an INS implementation.

## Frames, coordinates, and environment

- [ ] Define the minimum coordinate operations needed by PCPF/ECEF mechanization and local-vertical measurements.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code and documentation.
- [ ] Integrate `environment::Wgs84` and the selected gravity policy into physics code without adding an Earth-specific framework layer.
- [ ] Add tested helpers for required ECEF/geodetic/local-vertical conversions.

## Truth generation

- [ ] Correct stationary ECEF truth, including Earth rotation and nonzero stationary gyro truth.
- [ ] Define acceleration versus specific-force semantics in `TruthSample`.
- [ ] Add straight-line truth.
- [ ] Add constant-rate-turn truth.
- [ ] Add circular motion only if it adds distinct validation value.

## Sensor contracts

- [ ] Define a timestamped IMU sample type with documented increment/rate semantics.
- [ ] Implement an ideal IMU simulator first.
- [ ] Add IMU white noise, bias, bias random walk, scale factor, misalignment, and quantization incrementally with deterministic seeds.
- [ ] Implement the barometer simulator and a physically meaningful altitude model/Jacobian.
- [ ] Integrate the existing GNSS velocity model into simulation.
- [ ] Add explicit sensor scheduling and multi-rate behavior.

**Exit criteria:** ideal stationary and moving truth cases have physics-based tests; ideal sensors reproduce expected measurements; frame and unit conventions are documented at their APIs.

---

# Phase 6 — First complete PCPF/ECEF INS

**Goal:** Cross from a measurement-only estimator to a validated strapdown INS/GNSS navigation system.

## Nominal mechanization

- [ ] Implement a PCPF/ECEF mechanization policy configured by planet and gravity policies.
- [ ] Propagate quaternion attitude with normalization and documented convention.
- [ ] Propagate velocity with gravity, Coriolis, and specific force.
- [ ] Propagate position consistently with the chosen integration scheme.
- [ ] Keep PCI/ECI and local-level mechanizations deferred until the PCPF interface is proven.

## Error-state filter prediction

- [ ] Define the continuous-time error dynamics and process-noise mapping.
- [ ] Implement state-transition and process-noise discretization with documented approximation order.
- [ ] Add covariance prediction and symmetry/positive-semidefinite checks where practical.
- [ ] Connect prediction to the Navigator propagation seam.

## Attitude error handling

- [ ] Implement quaternion/attitude-error injection.
- [ ] Implement the corresponding covariance reset mapping.
- [ ] Verify attitude sign, perturbation side, and frame conventions analytically and numerically.

## Aiding integration

- [ ] Validate GNSS position aiding with the propagating filter.
- [ ] Validate GNSS velocity aiding.
- [ ] Validate barometric altitude aiding after its local-vertical model is corrected.

## Scenarios

- [ ] Stationary Earth alignment/hold test.
- [ ] Straight-line IMU/GNSS test.
- [ ] Constant-turn test.
- [ ] Short GNSS-outage drift test.

**Exit criteria:** ideal-data mechanization tests pass; noisy aided scenarios remain bounded; covariance and innovation outputs are available; the full simulation and analysis workflow runs from repository tools.

---

# Phase 7 — Estimator validation and consistency

**Goal:** Turn plots into repeatable engineering evidence.

## Near-term analysis improvements

- [ ] Add a skip-first-N or time-window option for transient exclusion.
- [ ] Add innovation summary statistics: mean, variance, standard deviation, RMS, and sample count.
- [ ] Add NIS summary statistics: mean, expected mean, 95%/99% exceedance rates, and mean p-value.
- [ ] Overlay expected Gaussian distributions on innovation histograms.
- [ ] Add whitened-innovation time histories and autocorrelation checks.
- [ ] Add a Kolmogorov-Smirnov or other justified uniformity test for p-values, documenting its limitations.

## State consistency

- [ ] Add NEES for scenarios with known truth.
- [ ] Add confidence bounds and exceedance statistics.
- [ ] Produce a machine-readable consistency summary.
- [ ] Generate a human-readable automatic validation report.
- [ ] Define pass/fail thresholds carefully; avoid treating a single stochastic run as proof of consistency.

**Exit criteria:** selected scenarios generate reproducible metrics and reports, statistical assumptions are documented, and failures produce actionable diagnostics.

---

# Phase 8 — Monte Carlo, logging, and performance

**Goal:** Support trade studies and quantify estimator behavior across randomized runs.

## Monte Carlo

- [ ] Add a seeded batch/Monte Carlo driver.
- [ ] Parallelize independent runs without changing determinism.
- [ ] Aggregate RMSE, NEES, NIS, failure counts, and runtime.
- [ ] Generate comparison tables and reports across configurations.

## Logging

- [ ] Refactor the current monolithic `RunLogger` into composable logging adapters after `SimulationApp<Config>` exists. `RunLogger` currently mixes truth logging, GNSS measurement logging, nav-state/error logging, GNSS update-statistics logging, metadata schemas, manifests, and app-specific dimensions in one large header.
- [ ] Move log-family selection into compile-time app logging config while keeping run-specific choices runtime-configurable. Compile-time logging config should define available log products, schema writers, and required compile-time dimensions; runtime JSON should select output directory, run name, optional enabled/disabled products where safe, and detail/verbosity.
- [ ] Replace hard-coded logging assumptions such as GNSS-only files, `StateDef::Pos` position extraction, and fixed `H`/`K` matrix dimensions with model/state-derived schema helpers or logger policies.
- [ ] Keep IO/logging adapters outside `navkit::core`; they may depend on `navkit::sim`, `navkit::io`, and app-support types, but product-core embedded algorithms must continue to emit only typed states, measurements, statistics, and profile records.
- [ ] Preserve existing stationary GNSS file names and schema compatibility until downstream analysis is intentionally migrated.
- [ ] Add log/schema version metadata and compatibility checks.
- [ ] Extend the Phase 3 timing/resource artifacts into Monte Carlo aggregate reports.
- [ ] Add memory/high-water statistics suitable for embedded evaluation once target/resource APIs are defined.
- [x] Keep compile-time profiling metadata in `navkit_build_manifest.json`, including clock source, tick scale, selected config, sink capacity, overflow policy, schema version, and profile-point mapping.
- [x] Add `profile_run_manifest.json` beside profile exports with runtime-only profile output facts such as CSV path, run name, record count, and dropped-record count.
- [ ] Extend the build and run manifest contracts with richer build identity once the selected-config and logging metadata stabilize.
- [ ] Improve Chrome Trace / Perfetto export readability with process/thread metadata, clearer display names, and stable category naming.
- [ ] Use profile record `sequence`, `parent_sequence`, and `depth` to represent cleaner nested timing once propagation, mechanization, and multi-sensor update paths make nesting informative.
- [ ] Add profile points around propagation, mechanization, sensor queue processing, update policies, and Monte Carlo/app-level loops after the first INS path exists.
- [ ] Evaluate whether Perfetto/Chrome trace remains sufficient before building NavKit-native timeline or flame-style visualization.
- [ ] Consider an optional binary backend only after CSV/JSON throughput becomes a demonstrated bottleneck.

**Exit criteria:** a repeatable batch command produces deterministic aggregate results and records enough version/configuration metadata to reproduce them.

---

# Phase 9 — Robust estimator and embedded readiness

**Goal:** Mature the framework after the first INS is demonstrably correct.

## Time and robustness

- [ ] Define explicit status/error handling for numerical and data-quality failures.
- [ ] Add innovation gating, fault detection, and measurement rejection policy.
- [ ] Add state history and latency handling.
- [ ] Add delayed-measurement replay.
- [ ] Add RTS smoothing after forward propagation/history interfaces stabilize.

## Type and frame safety

- [ ] Extend the existing unit/frame types based on observed misuse risks.
- [ ] Add compile-time DCM composition/result-frame checks.
- [ ] Add tested unit conversions and arithmetic where they improve safety without obscuring Eigen interoperability.
- [ ] Keep frame/type abstractions zero-overhead and verify generated/runtime behavior where important.

## Embedded deployment

- [ ] Define allocation, exception, RTTI, logging, and timing constraints for supported embedded profiles.
- [x] Separate desktop simulation and IO source from the reusable product-core boundary with `navkit::core`, `navkit::sim`, and `navkit::io` targets.
- [ ] Add embedded toolchain profiles and a hardware abstraction boundary when a target is selected.
- [ ] Add runtime and memory budgets to CI or target qualification tests.

## Documentation

- [ ] Add Doxygen/API documentation.
- [ ] Grow the navigation theory/reference manual alongside implemented equations.
- [ ] Add measurement-model and mechanization tutorials.
- [ ] Add end-to-end example walkthroughs and a developer architecture guide.

**Exit criteria:** supported embedded constraints are explicit and tested; robustness features have scenario coverage; public extension points are documented.

---

# Long-term product horizon

These are not scheduled until the first INS, validation pipeline, and extension boundaries are proven.

## Additional environments and mechanizations

- [ ] PCI/ECI mechanization.
- [ ] Local-level and wander-azimuth mechanizations.
- [ ] Atmosphere, magnetic-field, Earth-orientation, geoid, and terrain policies driven by concrete use cases.
- [ ] Multi-planet scenarios using the existing planet-policy direction.

## Additional aiding and navigation modes

- [ ] Magnetometer, radar, lidar, camera, visual odometry, star tracker, and celestial aiding.
- [ ] GPS-denied navigation demonstrations.
- [ ] Integrity monitoring and fault detection/exclusion.
- [ ] Multi-hypothesis and robust estimation.

## Alternative estimators

- [ ] Sliding-window optimization.
- [ ] Factor-graph backend and incremental smoothing.
- [ ] Shared measurement/environment interfaces only where EKF and graph implementations genuinely benefit from them.

## Simulation platform

- [ ] Multi-vehicle simulation.
- [ ] Hardware-in-the-loop integration.
- [ ] Production-grade scenario management and automatic qualification reports.

---

# Milestone summary

1. **Trusted baseline:** configured tests, CI, numerical baseline, aligned documentation.
2. **Constrained estimator:** estimator and diagnostic policy boundaries implemented and tested.
3. **Configurable, hardened, measurable core:** clear compile-time configuration, deliberate compiler/static-analysis posture, early timing/resource evidence, and intentional coverage standards.
4. **Propagation-ready Navigator:** no-op propagation preserves GNSS-only behavior.
5. **Physics-ready simulation:** correct truth, IMU contracts, and multi-rate ideal sensors.
6. **Complete first INS:** PCPF/ECEF mechanization, covariance prediction, and GNSS/barometer aiding.
7. **Professional validation:** consistency metrics, Monte Carlo, and reproducible reports.
8. **Embedded-ready toolkit:** robustness, type safety, resource budgets, and target profiles.
9. **Advanced navigation platform:** new aiding modes, mechanizations, and estimator backends.
