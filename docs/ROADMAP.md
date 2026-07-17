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
- The estimator policy refactor is partially complete. `StateDefPolicy`, `InjectionPolicy`, `ResetPolicy`, `MeasurementModelPolicy`, `NoisePolicy`, `FilterPolicy`, `FilterSensorPolicy`, `SensorCollectionPolicy`, `UpdatePolicy`, `NavigatorUpdatePolicy`, and `PropagationPolicy` exist. `KalmanFilter` is constrained on state, injection, reset, measurement-model, and sensor-dependent filter boundaries; `Sensor` is constrained on noise-policy and diagnostics compatibility; and `Navigator` is constrained on current filter, sensor-collection, propagation, and update-policy boundaries. The first ECEF INS propagation policy is wired into the stationary GNSS app with nominal quaternion attitude storage, while richer aiding and latency/replay handling remain future work.
- Public headers are organized by product boundary first, then engineering domain. `include/navkit/core` is the reusable product core, with estimation/navigation domains under `core/estimation`, environment models under `core/environment`, and reusable support domains such as `config`, `containers`, `frames`, `units`, and `models` also under `core`. Desktop simulation support remains under `include/navkit/sim`; desktop logging/file/JSON support remains under `include/navkit/io`.
- CMake targets now separate product boundaries: `navkit_core`/`navkit::core` is the reusable product-core interface library, `navkit_sim`/`navkit::sim` is the compiled simulator support library, `navkit_io`/`navkit::io` is the desktop logging/file/JSON interface library, `navkit_app_support`/`navkit::app_support` owns reusable selected-config/profile-export app plumbing, and runnable executables own their concrete application flow under `apps/`.
- `core/config` contains shared product-core compile-time configuration vocabulary such as foundational scalar/time aliases and `NumericConfigPolicy`. Domain-specific configuration concepts live beside the domains that consume them, following the general pattern `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`; estimator buffer and measurement-statistics configuration concepts are the first concrete examples.
- Runtime scenario files for applications are treated as app inputs, not core configuration. Repository-provided configuration now lives under the root `config/` tree: `config/compiletime/navkit/products/...` for reusable NavKit product configs, `config/compiletime/apps/...` for top-level executable composition configs, and `config/runtime/...` for JSON or other runtime inputs.
- App composition configs intentionally live in a separate tree from reusable NavKit library configs, so the same descriptive file name can exist in both places without ambiguity. A selected app config owns the link between `using NavKit = ...` and `using App = ...`; runtime JSON is then validated by the app-support layer against that compiled composition before the executable runs.
- Debug and Release build flags are now treated as explicit engineering products for NavKit-owned targets. CI enables warnings-as-errors, Release uses an embedded-oriented optimization profile, Linux Debug CI runs clang-tidy against the compilation database, and local agentic workflows intentionally do not run clang-tidy unless diagnosing that CI lane. Target-specific embedded toolchain flags remain future work until a target profile is selected.
- `GnssPosModel`, `GnssVelModel`, and `BaroAltModel` exist, but only GNSS position is integrated into the current simulation. The barometer model currently selects the third position component; it is not yet a general ECEF-to-local-vertical altitude model.
- The current trajectory generator supports only a simplistic stationary ECEF trajectory with timestamp, ECEF position, ECEF velocity, and body-to-ECEF attitude truth. Derived IMU quantities are produced by the IMU simulator rather than stored in `TruthSample`.
- `ImuSimulator` can generate deterministic raw IMU increments from consecutive ECEF truth samples, including Earth-rate gyro truth, specific force, bias, bias random walk, white noise, scale factor, misalignment, non-orthogonality, and quantization. The selected stationary GNSS simulation app now validates IMU runtime config, generates IMU increments, and feeds them into `Navigator::push_imu(...)`. `BaroSimulator` remains an empty shell.
- Analysis already provides position error/covariance, innovation, NIS, p-value, mean p-value, and innovation histogram plots. More formal statistics and consistency tests remain future work.
- Desktop timing and coarse binary-size artifacts now exist for the stationary simulation/analysis workflow and CI artifact upload. Product-core embedded profiling vocabulary now exists under `include/navkit/core/profiling`, and the first coarse `Navigator` and `KalmanFilter` integration points are instrumented. Profile export/visualization and memory/resource budgets remain future work.
- The documented and configured language standard is C++23.
- `tests/test_state_def_policy.cpp` is included in the configured test target, so its positive and negative concept assertions compile in local and CI builds.

## Current verified baseline

- [x] CMake and Conan build with Eigen, nlohmann-json, and doctest.
- [x] Python wrappers support build, test, stationary simulation, analysis, formatting, and copyright checks.
- [x] Fixed-capacity `RingBuffer`, fixed-size state/covariance aliases, and named state segments exist.
- [x] Generic measurement update, Joseph-form covariance update, injection/reset hooks, sensor queues, Navigator orchestration, and measurement statistics exist.
- [x] The selected stationary GNSS simulation now runs ECEF INS propagation from generated IMU increments before GNSS-position measurement updates.
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

# Phase 0 â€” Provenance and owner-controlled safeguards

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

# Phase 1 â€” Baseline integrity and documentation alignment

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

# Phase 2 â€” Estimator policy boundaries

**Goal:** Complete the next estimator policy refactor pass without changing GNSS-only runtime behavior.

## Injection and reset

- [x] Define candidate-first `InjectionPolicy<Candidate, StateDef>`.
- [x] Define candidate-first `ResetPolicy<Candidate, StateDef>`.
- [x] Constrain `KalmanFilter` on `StateDefPolicy`, injection, and reset policies.
- [x] Preserve the current INS additive-injection sign convention and zero-error reset behavior.
- [x] Keep covariance reset explicitly a no-op until attitude-aware reset is designed.
- [x] Add valid and invalid compile-time policy tests.

## Measurement models

- [x] Define `MeasurementModelPolicy<Candidate, StateDef>` around dimension, fixed-size matrix types, noise context, observation, Jacobian, covariance, and Kalman-gain operations.
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

# Phase 3 â€” Configuration, compiler flags, tests, and runtime profiling

**Goal:** Make the product-core configuration model, compiler/tooling posture, coverage strategy, and performance evidence explicit before adding more navigation physics and orchestration complexity.

## Compile-time configuration architecture

### Pass 3.1a â€” Initial core configuration vocabulary

- [x] Rename `include/navkit/core/common` to the explicit `include/navkit/core/config` product-core configuration domain.
- [x] Introduce foundational scalar/time aliases separately from named configuration policies.
- [x] Define the first narrow configuration concepts and a temporary `DefaultConfig` composition bundle for scalar/time aliases, sensor-buffer capacities, and measurement-statistics availability.
- [x] Relocate `navkit_sim` runtime JSON files into an explicit app-input location, separate from product-core configuration.
- [x] Add compile-time tests that valid configuration slices satisfy the intended concepts and intentionally invalid slices fail those concepts.

### Pass 3.1b â€” Configuration ownership cleanup

- [x] Keep `include/navkit/core/config` focused on configuration vocabulary shared broadly by product-core code, such as scalar/time types and any future truly cross-cutting core options.
- [x] Move domain-specific configuration concepts beside the domains that consume them once the domain boundary is clear, following the general pattern `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`; current examples include sensor buffer configuration near `core/estimation/sensor` and measurement-statistics configuration near `core/estimation/filter`.
- [x] Remove or demote library-owned concrete `DefaultConfig` as the universal product configuration once repository-provided app/product configurations exist outside public NavKit headers.
- [x] Document that concept policies distributed in domain folders define required capabilities, while concrete compile-time configurations are selected by applications or product builds.

### Pass 3.1c â€” Configuration guide and example-contract documentation

- [x] Add `docs/CONFIGURATION.md` as the human-facing map for the concept/policy configuration architecture.
- [x] Explain the core mental model: domain config concepts define local requirements; concrete compile-time configs compose named slices; aggregate product checks validate runnable graphs; CMake selects one config per build tree; runtime inputs remain separate.
- [x] Document the generic domain-specific policy location pattern, such as `include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp`, rather than implying config concepts only live under estimation.
- [x] Document that built-in concrete examples are the user-facing teaching surface, not a universal base class or internal default product configuration.
- [x] Describe `MinimalConfig` as a deliberately small example that shows required composition shape without implying it is a production target.
- [x] Document that runnable/product configs should avoid noisy duplicate slice-level `static_assert` blocks and usually end with one aggregate product-config check, while teaching configs and focused tests carry detailed concept examples.
- [x] Point readers to `tests/test_config_policy.cpp` for rigorous positive and negative examples, including invalid cases expressed as `static_assert(!Concept<Bad>)`.
- [x] Link `docs/CONFIGURATION.md` from `README.md`, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, `docs/README.md`, and `AGENTS.md`.

### Pass 3.1d â€” Root configuration tree

- [x] Create a root `config/` tree as the obvious place for repository-provided selectable configuration.
- [x] Add `config/compiletime/...` for C++ compile-time configuration headers, split into reusable NavKit library configs and app composition configs.
- [x] Add `config/runtime/...` for JSON or other runtime inputs consumed by desktop applications, simulators, and demos.
- [x] Move `navkit_sim` runtime JSON files to `config/runtime/navkit_sim/...` and update tools, apps, docs, and tests.
- [x] Add at least one plug-and-play app compile-time configuration for `navkit_sim`, such as `config/compiletime/apps/navkit_sim/EcefInsGnss.hpp`, so a fresh clone can build and run without user-authored configuration.
- [x] Add README files or examples that show where new desktop, embedded-target, simulation, and test-fixture configurations should live.

### Pass 3.1e â€” CMake selected-config path

- [x] Add a CMake cache variable named `NAVKIT_CONFIG` for selecting exactly one compile-time configuration header per build tree.
- [x] Provide a good default `-DNAVKIT_CONFIG` value, for example `apps/navkit_sim/EcefInsGnss.hpp` relative to `config/compiletime`, so ordinary clone/build/test workflows work without extra flags.
- [x] Keep `NAVKIT_CONFIG` orthogonal to `CMAKE_BUILD_TYPE`; Debug/Release chooses compiler mode, while `NAVKIT_CONFIG` chooses the top-level compile-time build configuration.
- [x] Generate a build-local selected-config header, for example `build/generated/navkit/SelectedConfig.hpp`, from a CMake template so generic applications can include one stable header.
- [x] Expose the selected compile-time configuration through a stable alias such as `navkit::selected_config::Config` or an equivalent clearly documented name.
- [x] Ensure selected-config include paths are applied to app or product targets that need them, not forced into `navkit::core` as a dependency on repository app configuration.
- [x] Update `navkit_sim` to remain config-agnostic in source and consume only the generated selected-config alias.

### Pass 3.1f â€” Multiple configurations and developer UX

- [x] Document the primary rule: one build tree selects one `NAVKIT_CONFIG`.
- [x] Support multiple configurations by using multiple build directories or CMake presets, not by making one executable dynamically switch among compile-time configurations.
- [x] Add CMake presets or documented wrapper examples that pair common build types and selected configs for convenience while keeping those axes independent.
- [x] Add a `tools/build.py` option such as `--navkit-config apps/navkit_sim/EcefInsGnss.hpp` that forwards to `-DNAVKIT_CONFIG=...`.
- [x] Make the default build directory derive from the selected compile-time config header, for example `apps/navkit_sim/EcefInsGnss.hpp` maps to `build/debug/apps/navkit_sim/EcefInsGnss`, so switching configs does not overwrite another build tree's generated selected-config header.
- [x] Document how to add a new compile-time config header, how to select it with CMake or the build wrapper, and how to pair it with a runtime JSON input when an application needs one.
- [x] Reconcile `README.md`, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, and `AGENTS.md` so the default selected config, root config tree, and one-config-per-build-tree rule stay discoverable.
- [x] Keep reusable NavKit library configs and app composition configs in dedicated directories so app and library configs can share descriptive names without coupling their ownership.
- [x] Add runtime-input validation at the app-support boundary so missing scenario sections, unsupported sensor/emulator sections, and malformed JSON inputs fail early with clear diagnostics against the selected compile-time composition.

### Pass 3.1g â€” Generic simulation-app composition

- [x] Replace the bespoke `StationaryGnssApp::run()` shape with a generic `SimulationApp<Config>::run()` that owns the common application loop: load runtime input, validate it against the selected compile-time app/NavKit composition, create the runtime trajectory, construct configured emulators, push generated measurements into the matching NavKit sensors, process the Navigator, log outputs, and export profiling artifacts.
- [x] Move app sensor/emulator capability selection into app compile-time config tuples, now represented as `EmulatorBindings`, while keeping numeric values such as noise, covariance, seeds, rates, output paths, and run names in runtime JSON.
- [x] Replace the current `RuntimeConfigValidation` shape with generic app-support validation that is derived from the selected app compile-time tuples. Validation must support arbitrary combinations of emulators/loggers and must not be hard-coded to stationary GNSS.
- [x] Define a small `SensorEmulatorPolicy`/adapter contract that connects each app-side emulator to an explicit configured NavKit sensor alias plus a stable app/runtime sensor ID. Do not rely solely on model-type lookup because realistic configurations may include multiple sensors with the same model type, such as two GNSS receivers, dual barometers, or redundant IMUs.
- [x] Represent each configured NavKit sensor and app emulator stream with an unsigned `SensorId`, optionally named by config-local constants such as `primary_gnss_sensor_id = 0U`. Configured emulator types now carry the app/runtime stream ID, app bindings explicitly state `(Emulator, Sensor)`, and compile-time checks prove IDs are unique, every emulator target sensor exists in the selected NavKit sensor graph, and binding IDs match the configured `Sensor::Id`.
- [x] Remove stale placeholder runtime configs, including placeholder future-scenario JSON files, during this refactor unless they are converted into real validated examples.
- [x] Add compile-time tests for valid/invalid app composition concepts and runtime tests for missing emulator sections, extra unsupported sections, and app/NavKit capability mismatches.

### Pass 3.1h â€” Public config API surface and product graph aliases

- [x] Add an explicit public config API directory, for example `include/navkit/api/config`, as the front door for compile-time configuration contracts intended for end users.
- [x] Define `NavKitProductConfigPolicy` in the public config API. It should explicitly state the required aliases for a runnable/product NavKit configuration, including at least `StateDef`, `Sensors`, `Profiler`, `Filter`, `NavigatorUpdate`, and `Navigator`.
- [x] Define or expose additional public config API concepts only when users are expected to satisfy or assert them in concrete compile-time configs. Domain implementation concepts stay beside their consuming domain unless they graduate into the user-facing config API.
- [x] Keep the public API concepts as contracts and documentation for config authors; do not move every low-level policy concept into the API folder merely because it exists.
- [x] Move product graph aliases into reusable NavKit configs: state definition, sensor model aliases, concrete sensor aliases, `Sensors`, `Profiler`, `Filter`, and `Navigator`.
- [x] Decide whether `MeasurementModels` should be public, derived, or removed. The current direction is to remove it and require explicit `MeasurementStatisticsTuple` aliases keyed by configured sensors.
- [x] Collapse app configs so they no longer reconstruct NavKit sensors. App configs should select `NavKit`, define configured emulator aliases that carry stream IDs, define `EmulatorBindings`, and select `SimulationApp<Config>`.
- [x] Simplify emulator binding machinery so the ID is the app/runtime key and the sensor target is an explicit NavKit sensor alias. `SimulationApp` derives the tuple index from `NavKit::Sensors` by `Sensor::Id`, so app configs avoid raw indices and do not search sensor bindings by potentially duplicated model type.
- [x] Update configuration docs and tests so users can find the public config concepts, see which aliases are required, and understand which aliases are local helper wiring rather than the public config contract.

### Pass 3.1i â€” Simplify diagnostics ownership and config graph policy boundaries

- [x] Remove the current public `SensorGraphConfigPolicy` shape. The public config API should not force derived aliases such as `MeasurementModelsFromSensors_t<Sensors>` or expose tuple-search/count machinery as user-facing architecture. Keep only public contracts that config authors are expected to satisfy directly.
- [x] Replace `MeasurementModels` with an explicit `MeasurementStatisticsTuple` alias in concrete NavKit configs. The tuple must be manually authored, for example `using MeasurementStatisticsTuple = std::tuple<MeasurementStatistics<PrimaryGnssSensor>>;`, so config readers can see exactly which configured sensor streams produce stored diagnostics.
- [x] Key `MeasurementStatistics` directly on the configured `Sensor` type, not on a detached measurement model and not through a new `MeasurementStatisticsConfig<Sensor>` wrapper. The sensor already carries `Sensor::Id` and `Sensor::MeasurementModel_t`, which should be enough identity for duplicate same-model sensors.
- [x] Delete derived helpers such as `MeasurementModelsFromSensors_t` or any future `MeasurementStatisticsTupleFromSensors_t` unless a compelling, demonstrated need exists. Prefer explicit config tuples over metaprogramming convenience that hides product behavior.
- [x] Remove dead/aspirational config leaves such as `EnableMeasurementStatistics`. Diagnostic storage is configured by the explicit `MeasurementStatisticsTuple`; no separate boolean gate or filter diagnostics/settings bundle is needed until a concrete memory/performance requirement justifies it.
- [x] Slim `include/navkit/core/estimation/sensor/SensorId.hpp` and app-support identity headers so they define identity types and simple binding structs only. Move lookup helpers such as `SensorIndexFromId_v`, `SensorFromId_t`, `BindingIndexFromId_v`, and `EmulatorFromId_t` into a clearly named internal tuple/lookup utility header if they remain necessary.
- [x] Move generic tuple helper machinery out of domain headers such as `MeasurementStatistics.hpp`, `SensorGraphConfigPolicy.hpp`, and `Navigator.hpp`. Helpers like `tuple_contains_v`, `tuple_index_v`, and `NavigatorPolicyCompatibility` should live in small internal/detail headers or be removed by simplifying ownership.
- [x] Define a real `SensorPolicy` concept that proves a type is a NavKit sensor: configured ID, model alias, measurement alias, noise context, and queue operations. Then strengthen `SensorCollectionPolicy` so it means "tuple of sensors", not merely "tuple-like type".
- [x] Make the `Sensor` -> `SensorPolicy` -> `SensorCollectionPolicy` -> `Navigator` chain obvious in headers and tests. A reader should not need to reverse-engineer why `std::tuple<int, double>` is not a valid sensor collection.

### Pass 3.1j â€” Finish app orchestration cleanup and naming polish

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
- [x] Move reusable NavKit product configs under `config/compiletime/navkit/products`, including `MinimalConfig.hpp`, `EcefInsGnss.hpp`, and `ProfiledEcefInsGnss.hpp`. Product configs should use product-local namespaces so scenario identity comes from the file path and namespace, while local types use role-based names such as `NumericConfig` and `ProductConfig`. Export stable scenario-specific aliases from `navkit::config::navkit`, such as `EcefInsGnssConfig` and `ProfiledEcefInsGnssConfig`. Future reusable component configs, such as profilers, sensor bundles, filters, or navigator update choices, should live in their own component folders only after multiple real options exist, and their exported names should be descriptive because users select them directly.

### Pass 3.1k â€” Header boundary and helper split cleanup

- [x] Add durable config API inclusion guidelines to `AGENTS.md` and, if the rule is architecture-level enough, `docs/ARCHITECTURE.md`. `ConfigApi.hpp` should include shared public configuration vocabulary and defaults exposed by primary core template boundaries; concrete product configs should include `ConfigApi.hpp` plus the specific selected model, profiler, target, or component headers they use. Do not turn `ConfigApi.hpp` into a grab bag of every possible concrete component choice.
- [x] Split profiling policy vocabulary so the primary concepts and public types are easy to find. Separate clock, sink, scope, profiler, and sink-record trait/helper concerns, then retire the unused `ProfilePolicy.hpp` umbrella include.
- [x] Split `ScopedProfiler.hpp` so the scoped RAII profile record type and the profiler facade have obvious ownership and neither is buried under unrelated helpers.
- [x] Clarify the old `SensorTuple.hpp` umbrella. Split the public sensor-tuple concept/policy from tuple lookup/count traits into `SensorTuplePolicy.hpp` and `SensorTupleTraits.hpp`, then retire the unused umbrella include.
- [x] Clarify app emulator-binding headers. If `EmulatorBindingPolicy.hpp` mostly owns binding structs, tuple uniqueness, lookup, and validation helpers rather than an obvious `EmulatorBindingPolicy` concept/type, split policy/contract vocabulary from traits/helpers.
- [x] Split `NavKitProductConfigPolicy.hpp` so the public product-config concept stays readable as the API contract, while supporting detail checks live in a small detail/traits header.
- [x] Decompose `RuntimeConfigValidation.hpp` into smaller app-support headers for runtime key/schema definitions, JSON value parsing helpers, emulator runtime-key derivation, and validation orchestration. Keep the top-level validation entry point obvious.

### Pass 3.1l â€” Follow-on app/config simplification

- [x] Reduce `SimulationApp<Config>` back to orchestration only: load validated runtime input, obtain truth samples from a trajectory provider, run configured emulators, process the configured Navigator, and delegate logging/export details.
- [x] Replace dummy-object type dispatch such as `typename NavKit::MeasurementStatisticsTuple{}` with explicit type-level helper APIs, for example `log_measurement_statistics<typename NavKit::MeasurementStatisticsTuple>(logger, filter)`, so `SimulationApp` does not appear to pass empty runtime statistics state.
- [x] Keep trajectory category and trajectory parameters runtime-configurable. Swapping stationary, straight-line, turn, and future scripted trajectory families from JSON is intentionally useful and is an acceptable non-hot-path runtime polymorphism exception outside product-core embedded algorithms.
- [x] Collapse one-field public concrete config slices into aggregate config settings where they do not improve readability. Keep internal domain policy concepts available where useful, but do not force end users through standalone one-field types unless those types are independently reusable.
- [x] Refactor `RunLogger` toward composable log products/adapters while preserving existing stationary GNSS filenames, schemas, manifests, and downstream analysis compatibility during a focused IO cleanup pass.
- [x] Treat logging as a mixed compile-time/runtime concern at the first seam: app compile-time config selects the logger adapter type, while runtime JSON continues to select run name and output directory. Richer optional log-product enable/disable and verbosity selection remains a Phase 8 logging task.
- [x] Preserve existing stationary GNSS runtime behavior, file names, manifest contents, profile export behavior, and analysis compatibility while simplifying the app loop and logging boundary.
- [x] At the end of the pass, run the normal format/copyright/build/test workflow, then run local clang-tidy explicitly with `python tools/format.py --check --tidy --tidy-warnings-as-errors` and let it run long enough to collect full findings before deciding whether to fix or defer issues.

### Pass 3.1m â€” App config type-aggregation cleanup

- [x] Refactor app compile-time configs to use the same readable type-aggregation style as NavKit product configs: local role aliases, configured emulator aliases, explicit binding aliases, aggregate `EmulatorBindings`, selected `Logger`, and final `App`. App configs may reference exported NavKit product aliases but must not reconstruct product-core internals.
- [x] Treat `EmulatorBindings` as the owned app graph. Do not add a separate `Emulators` tuple unless a real app consumer needs emulator-only iteration; each binding already carries the emulator, target sensor, and runtime stream relationship.
- [x] Embed the runtime stream `Id` into configured emulator types, mirroring `Sensor::Id`. `EmulatorBinding<Emulator, Sensor>` should derive `Binding::Id` from `Emulator::Id` and `static_assert(Emulator::Id == Sensor::Id)`, instead of duplicating the same ID as a third binding template parameter.
- [x] Preserve current selected-config behavior, stationary GNSS runtime behavior, file names, profiling export, and analysis compatibility.

### Pass 3.1n â€” App-support concept-policy boundary cleanup

- [x] Move `SimulationAppConfigPolicy` out of `SimulationApp.hpp` into a focused standalone policy header. Keep `SimulationApp.hpp` focused on the application orchestration loop.
- [x] Establish the hard default rule that policy concepts live in clear standalone `*Policy.hpp` headers. Exceptions should be rare and deliberate: tiny private implementation concepts may stay local only when moving them would make ownership less clear.
- [x] Replace raw emulator-binding trait checks inside `SimulationAppConfigPolicy` with a named binding-tuple concept. The simulation-app config concept should read in terms of product config plus emulator-binding compatibility, not raw `_v` helper plumbing.
- [x] Split emulator binding vocabulary into an individual `EmulatorBindingPolicy` for one binding and an `EmulatorBindingTuplePolicy` for the tuple relationship against the selected NavKit sensor tuple. Keep lookup/count helpers in traits headers, not in the policy headers.
- [x] Constrain `EmulatorRuntime<NavKit, EmulatorBindings>` on `NavKitProductConfigPolicy<NavKit>` and the emulator-binding tuple policy so runtime plumbing cannot be instantiated with unrelated tuple-like types.
- [x] Constrain `validate_runtime_config<Config>` on the simulation-app config concept after the concept lives in a standalone header, so runtime validation shares the same compile-time app/product contract as `SimulationApp`.
- [x] Add an `EmulatorPolicy` concept for app-side emulators. It should capture the real static interface used by `EmulatorRuntime`: runtime construction, runtime-key/JSON validation, sensor configuration, measurement generation, and measurement logging. Keep it narrow and based on the current emulator boundary; do not introduce a generic simulation framework.
- [x] Constrain `MeasurementModelBase` and concrete measurement models such as GNSS position, GNSS velocity, and barometer on `StateDefPolicy` where the state definition is a real public template boundary.
- [x] Try constraining `Navigator::process_one_sensor` with `SensorPolicy` so the internal Navigator loop stays aligned with the public sensor-collection contract. Keep the change only if it improves diagnostics without fighting `std::apply` reference behavior.
- [x] Investigate, but do not force in this pass, whether `KalmanFilter::process_sensor` and the private direct-observation implementation should be split or further constrained. Avoid contorting the design merely to replace `typename` where the current mixed sensor/direct-model path is intentional.
- [x] Hold off on additional `RunLogger`/`RunLogProducts` concept work until the dedicated generic logger-composition pass. Note that logger constraints should be revisited there, after the product/payload split is stable.
- [x] Add focused compile-time tests for the new policy concepts and negative cases. Keep helper machinery minimal and local; the concepts should clarify ownership boundaries, not add another abstraction layer.

### Pass 3.1o â€” App-support directory organization

- [x] Reorganize `include/navkit/app_support` into ownership-oriented subdirectories while preserving behavior: top-level app entry/orchestration headers, `config/` for app compile-time config concepts and traits, `emulation/` for generic emulator/binding/runtime machinery, `runtime/` for JSON/runtime-input parsing and validation, `initialization/` for current startup initialization helpers, `logging/` for app-side logging adapters, `profiling/` for profile export adapters, and `trajectory/` for trajectory-provider helpers.
- [x] Keep generic emulation infrastructure directly under `app_support/emulation`, but move concrete emulators into a clearly named concrete location such as `app_support/emulation/concrete` unless a domain-specific subfolder like `gnss/` becomes immediately useful.
- [x] Update includes to prefer the new narrow paths. Add temporary umbrella headers only when they materially reduce churn or preserve useful public include compatibility; do not recreate a flat junk drawer through umbrellas.
- [x] Keep `SimulationApp.hpp` easy to read after the move: it should remain an orchestration loop that delegates runtime input, emulation, logging, profiling, trajectory, and initialization responsibilities to focused headers.
- [x] Add or update documentation in `docs/ARCHITECTURE.md` or `docs/CONFIGURATION.md` if the new layout changes how end users discover app compile-time config, runtime JSON validation, or simulator emulators.
- [x] Run format/copyright checks, Debug build/tests, and the stationary GNSS sim/analysis pipeline after the move to catch include-path and selected-config regressions.

### Pass 3.1p â€” Core policy-concept contract cleanup

- [x] Flesh out `FilterPolicy` so it names the stable standalone filter lifecycle contract consumed by Navigator, initialization, and app-support code. Do not include implementation-specific diagnostics or every `KalmanFilter` helper merely because they exist.
- [x] Add a separate sensor-dependent filter compatibility concept, `SensorFilterPolicy<Filter, Sensor>`, so tuple compatibility can say "this filter can consume this configured sensor" without forcing every filter consumer to know about sensors.
- [x] Use `FilterPolicy` directly in the `Navigator` template declaration, and keep tuple-wide sensor/filter/update compatibility checks in `NavigatorPolicyCompatibility`.
- [x] Update `NavigatorPolicyCompatibility` to read in terms of the real concepts it owns, `SensorFilterPolicy<Filter, Sensor>` and `UpdatePolicy<Update, Filter, Sensor>`, rather than overloading a broad filter concept with sensor-dependent meaning.
- [x] Add a narrow `FilterCorrectionPolicy` for inject/reset behavior and constrain concrete Navigator update policies, such as `UpdatePostFilter<Filter>` and `UpdateAfterEachSensor<Filter>`, on that smaller boundary instead of full sensor-processing behavior.
- [x] Constrain `KalmanFilter::process_sensor` with `SensorPolicy Sensor` while keeping the separate measurement-model policy check that proves the sensor's measurement model can update the configured state definition.
- [x] Lightly constrain current app-support initialization helpers with `StateDefPolicy` and the appropriate filter concept, while avoiding a deep polish of the existing truth/error-style initialization path. Phase 3.3 owns the real PVA initialization and transfer-alignment redesign.
- [x] Lightly constrain `MeasurementStatisticsLogger` with existing sensor/filter concepts where it improves clarity, but defer logger-specific constraints until the generic logger composition pass replaces GNSS-specific method probing with typed payload dispatch.
- [x] Rename the measurement-model concept vocabulary from `MeasurementPolicy` to `MeasurementModelPolicy`, aligning it with `MeasurementModelBase` and making clear that the policy constrains the model attached to a sensor, not a raw measurement sample.
- [x] Migrate public/config-facing sensor aliases from generic model names to measurement-model names. `Sensor::Model_t` is now `Sensor::MeasurementModel_t`, and downstream public aliases such as `MeasurementStatistics<Sensor>` use `MeasurementModel_t`.
- [x] Rename concrete config aliases such as `PrimaryGnssModel` to `PrimaryGnssMeasurementModel` where they are user-facing product graph nodes. Tiny local implementation aliases named `Model` remain acceptable where the surrounding function scope makes the meaning obvious.
- [x] Add positive and negative compile-time tests for the new and renamed concepts. Keep raw `typename` in private tuple expansion helpers where a concept would only add ceremony.

### Pass 3.2 â€” Log product concepts and payload boundaries

- [x] Add a narrow `LogProductPolicy<Candidate, Payload>` concept under `include/navkit/io` that validates the shared log-product lifecycle and the concrete payload-specific `log(payload)` operation. Do not force every log product into one fake common `log(...)` signature.
- [x] Introduce explicit log payload wrapper types for products whose natural inputs are more than one argument, such as nav-estimate logging and measurement-statistics logging. Payload wrappers should name the serialized boundary clearly instead of leaking helper argument lists through `RunLogger`.
- [x] Update existing concrete log products to satisfy `LogProductPolicy` with their actual payloads: truth samples, GNSS position measurements, nav-estimate payloads, and GNSS position update-statistics payloads.
- [x] Move generic CSV schema helpers such as matrix-header and matrix-value flattening out of `RunLogProducts.hpp` into a focused helper header such as `include/navkit/io/CsvSchemaUtils.hpp`.
- [x] Add compile-time tests proving each current concrete log product satisfies its intended `LogProductPolicy<Candidate, Payload>` instance, plus negative concept tests for missing lifecycle/schema/log operations.
- [x] Keep `RunLogger` as the coordinating faÃ§ade and manifest owner for now. It should compose log products through typed payloads, preserve current stationary GNSS filenames/schemas/manifests, and avoid becoming a second logging framework.
- [x] Keep richer runtime optional log-product enable/disable, verbosity, schema migration, and model/state-derived generic logging in the later Phase 8 logging scope unless this pass uncovers a tiny prerequisite.

#### Pass 3.2a â€” Log product header organization

- [x] Split concrete log products out of `RunLogProducts.hpp` into focused headers under `include/navkit/io/log_products/`: `TruthLogProduct.hpp`, `GnssPositionLogProduct.hpp`, `NavEstimateLogProduct.hpp`, and `GnssPositionUpdateLogProduct.hpp`.
- [x] Split reusable payload wrappers into `include/navkit/io/log_payloads/`: `NavEstimateLogPayload.hpp` and `MeasurementStatisticsLogPayload.hpp`. Keep payloads beside concrete products only if they are truly private to one product; otherwise keep the dedicated payload folder so callers can name payload boundaries clearly.
- [x] Delete the unused `RunLogProducts.hpp` umbrella after replacing production code and tests with specific product/payload headers.
- [x] Preserve the existing namespaces and public include compatibility where practical, but prefer narrow includes in production code and tests so dependencies stay obvious.
- [x] Add or update compile-time tests if needed so the split headers still expose all current concrete log-product policy checks.

#### Pass 3.2b â€” Generic compile-time RunLogger composition

- [x] Replace the current hard-coded `RunLogger` member list with a generic `RunLogger<LogProducts...>` or equivalent tuple-based composition selected by app compile-time config. The default stationary GNSS logger should remain available through a clear alias so existing app configs stay readable.
- [x] Keep the implementation simple and explicit: avoid broad tuple metaprogramming machinery, type-erasure, virtual dispatch, or a runtime registry. Use small local helpers only where they directly dispatch a typed payload to the matching product.
- [x] Route logging by explicit payload type and `LogProductPolicy<Product, Payload>` conformance. If multiple products can consume the same payload, require an explicit decision instead of silently logging to all or guessing.
- [x] Replace `MeasurementStatisticsLogger`'s GNSS-specific method probing, such as `log_gnss_pos_statistics(...)`, with typed payload dispatch through the selected logger/log products. App-support generic code must not hard-code GNSS-specific logger method names.
- [x] Move log-family selection into compile-time app logging config while keeping run-specific choices runtime-configurable. Compile-time logging config should define available log products, schema writers, and required compile-time dimensions; runtime JSON should continue to select output directory and run name in this pass.
- [x] Keep `RunLogger` responsible for output directory setup, product open/flush/close orchestration, run manifest ownership, and metadata file emission. Concrete log products own their CSV/schema details.
- [x] Preserve stationary GNSS filenames, metadata schemas, run-manifest shape, profile export behavior, and downstream Python analysis compatibility.
- [x] Keep IO/logging adapters outside `navkit::core`; they may depend on `navkit::sim`, `navkit::io`, and app-support types, but product-core embedded algorithms must continue to emit only typed states, measurements, statistics, and profile records.
- [x] Add compile-time tests for generic logger composition and at least one negative case where a selected product cannot consume the requested payload.
- [x] Add runtime or integration evidence by running the stationary GNSS sim and analysis pipeline after the generic logger refactor.

#### Pass 3.2c â€” RunLogger API and measurement-statistics ownership cleanup

- [x] Keep `include/navkit/io/RunLogger.hpp` focused on the generic compile-time logger faÃ§ade. Rename `BasicRunLogger` to `RunLogger<LogProducts...>` and move local helper machinery such as product matching counts and metadata-path derivation into `include/navkit/io/RunLoggerTraits.hpp`.
- [x] Move concrete stationary GNSS logger composition out of `RunLogger.hpp` and into app compile-time config. App configs should explicitly alias their selected logger product set, e.g. `using Logger = navkit::io::RunLogger<TruthLogProduct, ...>;`.
- [x] Use value-style naming for value helpers such as `matching_product_count_v`; reserve PascalCase for types and concepts.
- [x] Add focused `RunLogger` tests with small fake log products and a temporary output directory to prove product open/flush/metadata/manifest ownership, positive payload dispatch, zero-matching-product failure, and ambiguous multi-product payload failure.
- [x] Add `LoggerPolicy`, `LoggerPayloadPolicy<Logger, Payload>`, and narrow `LoggerProductAccessPolicy<Logger, Product>` concepts for logger lifecycle, payload dispatch, and selected-product access. Keep `LoggerProductAccessPolicy` distinct from `LogProductPolicy`: the former proves a logger composition exposes a concrete product for setup, while the latter proves the product can serialize a payload.
- [x] Rename `SensorFilterPolicy` to `FilterSensorPolicy<Filter, Sensor>` so policy names follow candidate-first/context-second ordering consistently with `LoggerPayloadPolicy<Logger, Payload>` and `LoggerProductAccessPolicy<Logger, Product>`.
- [x] Add `SensorDiagnosticsPolicy` and a default diagnostics config type. Use `SensorDiagnosticsPolicy` directly in the `Sensor` template parameter list, expose `Sensor::Diagnostics_t`, and keep disabled measurement statistics as no-op behavior rather than removing storage slots from tuples.
- [x] Remove public/user-facing `MeasurementStatisticsTuple` aliases from reusable NavKit product configs. Product configs should expose `Sensors`; filter-owned diagnostic storage should be derived internally from that explicit sensor graph.
- [x] Keep `MeasurementStatistics<Sensor>` templated on `Sensor` for now so duplicate sensors using the same measurement model remain unambiguous.
- [x] Add `include/navkit/core/estimation/filter/MeasurementStatisticsStorage.hpp` with a small mechanical `MeasurementStatisticsStorage_t<Sensors>` helper that expands a sensor tuple into `std::tuple<MeasurementStatistics<Sensor>...>`. Treat it as filter-storage plumbing, not a public config helper.
- [x] Change `KalmanFilter` to take the configured `Sensors` tuple instead of a public `MeasurementStatisticsTuple`, derive `MeasurementStatisticsTuple_t` internally through `MeasurementStatisticsStorage_t<Sensors>`, and keep the filter as the owner of filter-produced diagnostics.
- [x] Rename `has_measurement_statistics<Sensor>()` to `measurement_statistics_available<Sensor>()`. It should return false when diagnostics are disabled for the sensor or before a valid latest update has been recorded, so loggers do not emit default-zero diagnostic rows.
- [x] Update `MeasurementStatisticsLogger` to iterate `NavKit::Sensors`, construct `MeasurementStatisticsLogPayload<MeasurementStatistics<Sensor>>` explicitly, and constrain call sites with `FilterSensorPolicy` and `LoggerPayloadPolicy` where that improves diagnostics.
- [x] Document Windows clang-tidy expectations: Linux CI remains canonical; local Windows clang-tidy requires a compile-database-capable generator such as Ninja. Add Ninja to the Python/bootstrap or setup guidance as a local-tidy prerequisite without making local tidy part of the normal agentic loop.

#### Pass 3.2d â€” Log schema generalization and remaining product-specific assumptions

- [x] Make Ninja the default generator for repository Python build wrappers while keeping a deliberate generator override behind explicit `--build-dir`. Pass Ninja through Conan/CMake configuration, keep selected-config build directories isolated by build type/config, document the one-time clean rebuild expectation when replacing existing MSBuild trees, and preserve CMake presets as Ninja-based examples. Treat faster incremental builds as a bonus; the main goal is consistent compile-database generation for optional local clang-tidy and CI-like diagnostics.
- [x] Begin removing hard-coded logging assumptions such as GNSS-only update-statistics plumbing, `StateDef::Pos` position extraction, and fixed `H`/`K` matrix dimensions where the selected product/log payload can own the schema more clearly. Current scope keeps the GNSS position-update product GNSS-specific, but templates it on the selected statistics stream so state and matrix dimensions come from the configured payload.
- [x] Decide how optional runtime log-product enable/disable, verbosity, schema migration, and richer compatibility checks fit into the longer-term logging architecture. Keep them in Phase 8 unless a tiny prerequisite falls out naturally.
- [x] Revisit whether measurement-statistics logging should become product/model-specific log products beyond the current GNSS position update product once multiple measurement models are active. Current decision: keep the explicit GNSS update product for now and add additional typed products when additional measurement models are actually logged.
- [x] Preserve stationary GNSS filenames, metadata schemas, run-manifest shape, profile export behavior, and downstream Python analysis compatibility while generalizing schemas.

### Pass 3.3 â€” Navigation initialization and transfer-alignment boundary

- [x] Replace direct truth/error-style filter initialization in app support with an explicit initialization boundary. Product-core navigation code must not know about truth, injected errors, or simulator-only perturbations.
- [x] Introduce a core-facing `NavInitialization`-style message that represents only the initial navigation solution, not the full Kalman/filter state. It should carry PVA content such as position, velocity, attitude, timestamp, and PVA covariance; the filter/product maps that message into its internal state layout.
- [x] Add an app-side `NavInitializationProvider` concept/config seam. It should produce the required one-time startup initialization message from runtime input, simulated truth, a saved state, external data, or future embedded inputs without changing Navigator construction.
- [x] Rename current runtime initialization inputs away from filter-error language such as `initial_position_offset_m`. Prefer explicit initializer types such as `pva_error` for deterministic JSON-provided PVA error/covariance and `pva_random` for simulator-side random PVA error generated from covariance.
- [x] Keep Navigator construction separate from initialization. Construction wires the compile-time product graph; every runnable config calls an `initialize_navigator(...)`-style path with a `NavInitialization` message before normal updates.
- [x] Add a separate optional transfer-alignment boundary. Transfer alignment is not construction and not the required initial PVA message; it is a timestamped aiding stream that active configurations may call through a `transfer_align(...)`-style path.
- [x] Introduce a `TransferAlignmentProvider` concept/config seam for optional alignment aiding samples. Do not make the interface GNSS-specific: GNSS may be one aiding source, but transfer alignment should be framed around source-independent PVA aiding plus optional angular-rate and translational-acceleration aiding.
- [x] Define a `TransferAlignmentSample` shape with PVA, timestamp, optional angular-rate/specific-force aiding fields where useful, and matching TXA covariance representation. Keep optional TXA covariance handling simple and explicit first; only introduce compile-time capability flags if runtime optional fields become unclear or too dynamic for embedded-facing paths.
- [x] Extend runtime JSON validation so initialization and transfer-alignment sections are checked against the selected compile-time app configuration. Missing required initialization inputs, unsupported initializer types, and disabled/unsupported transfer-alignment sections should produce clear errors.
- [x] Keep truth/noise/error-distribution logic entirely in app/sim providers. Product-core code consumes typed initialization and transfer-alignment messages only.
- [x] Preserve the current stationary GNSS demo behavior through a simple default initialization provider while migrating the naming and boundaries.
- [x] Add focused tests for runtime JSON validation, required initialization presence, unsupported initializer/transfer-alignment types, and the happy-path stationary GNSS initialization provider.
- [x] Document the lifecycle rule: construction is compile-time product wiring, initialization is required runtime nav-data input, transfer alignment is optional timestamped aiding input, and simulator truth/error/noise models stay outside product-core embedded algorithms.

### Pass 3.3a â€” PVA initialization and transfer-alignment polish

- [x] Add a reusable `Vec3` alias as `using Vec3 = Eigen::Matrix<core::Scalar_t, 3, 1>;` under `include/navkit/core/math/Types.hpp`. Keep it as a convenience alias for common 3D vectors without forcing every API to use it.
- [x] Add `PvaStateDef` with `Pos`, `Vel`, and attitude-error-vector segments and `N = 9`, and represent PVA covariance as `StateCov<PvaStateDef>` so initialization covariance preserves position/velocity/attitude cross-correlation terms and uses existing segment/block utilities.
- [x] Store `NavInitialization` PVA content as a packed `PvaState` plus `PvaCovariance`, mirroring transfer-alignment `TxaState`/`TxaCovariance` storage. Centralize physical interpretation through accessors such as `pos_e_m`, `vel_e_mps`, and `rpy_b2e_rad` instead of repeating segmented vectors directly on initialization messages.
- [x] Move stable PVA/TXA state vocabulary under `include/navkit/core/estimation/navigator`, while keeping app-side provider files under `include/navkit/app_support/initialization`.
- [x] Rename PVA fields for clarity and unit consistency, including staged `attitude_rad`/`att_rad` vocabulary to `rpy_b2e_rad`. Align covariance field names with the PVA state vocabulary and units while avoiding one-off 3x3 covariance block members that lose cross-correlation terms.
- [x] Replace the staged `truth_perturbed_pva` runtime/provider naming with clearer initializer choices: `PvaExplicitInitializationProvider` for explicit deterministic PVA error input and `PvaRandomInitializationProvider` for random PVA error draws from configured covariance.
- [x] Restructure runtime initialization JSON around explicit PVA sections. Use `pva_error` with frame/unit-bearing fields such as `p_e_m`, `v_e_mps`, and `rotvec_b2e_rad`, or local-level equivalents such as `p_n_m`, `v_n_mps`, and `rotvec_b2n_rad`; use `pva_cov` with either `diag` containing 9 diagonal covariance values or `full` containing 81 row-major covariance values in the same implied convention. Drop sigma-based initialization inputs from this path.
- [x] Add runtime validation that rejects missing/ambiguous `pva_cov` shapes, wrong `diag`/`full` lengths, unsupported initializer types, and malformed PVA vectors with clear messages.
- [x] Clean provider control flow so it does not use raw pointers, `nullptr`, or repeated null comparisons to branch on optional truth samples; use explicit empty/non-empty trajectory logic instead.
- [x] Add `PvaRandomInitializationProvider` using the configured full PVA covariance to generate colored random PVA error draws with an explicit runtime seed, while keeping randomness in app/sim support and outside product-core code.
- [x] Add `TxaStateDef`, `TxaState`, and `TxaCovariance` for transfer-alignment aiding/covariance cross-correlation terms, and add transfer-alignment validity flags directly on `TransferAlignmentSample`: `pos_valid`, `vel_valid`, `rpy_valid`, `angular_rate_valid`, and `specific_force_valid`. Keep time always valid and defer bitset/packed-flag optimization until a target memory constraint exists.
- [x] Split concrete initialization providers into `include/navkit/app_support/initialization/concrete` and keep shared PVA JSON parsing/sampling helpers separate from provider classes.
- [x] Update tests and docs for the new JSON shape, full/diagonal covariance handling, deterministic explicit initializer, random covariance-sampled initializer, and transfer-alignment validity flags.

### Pass 3.3b â€” Build, install, and generated-output layout cleanup

- [x] Make Ninja the required generator for repository Python wrappers and CI. Keep raw CMake generator overrides possible only through explicit user-selected build directories; do not optimize the official wrapper layout around Visual Studio/Ninja generator swapping.
- [x] Remove the generator segment from default wrapper build directories. Use the simpler convention `build/<build-type-lower>/apps/navkit_sim/<ConfigStem>`, for example `build/debug/apps/navkit_sim/EcefInsGnss`, while preserving `--build-dir` for advanced/manual layouts.
- [x] Update Python build helpers, CMake presets, VS Code launch/docs, CI paths, timing/resource helpers, and setup/configuration documentation away from `build/Ninja/<BuildType>/...` toward the new `build/debug/...` and `build/release/...` convention.
- [x] Add a deliberate CMake install/staging path rooted at `install/<build-type-lower>/apps/navkit_sim/<ConfigStem>` for local package validation. Install public headers, exported CMake targets, compiled libraries, and runnable app executables; do not install source-tree runtime JSON examples by default.
- [x] Keep unit tests primarily build-tree based for local development speed. Add install-tree smoke validation separately, such as running the installed `navkit_sim` executable against the source-tree runtime JSON and/or compiling a tiny downstream `find_package(NavKit CONFIG REQUIRED)` consumer against the staged install.
- [x] Reserve `artifacts/` for CI/CD bundles, uploaded reports, packages, and release-style outputs rather than normal local build trees or local simulation logs.
- [x] Rename generated run-output root from `data/` to `output/`, especially `data/logs/<run_name>` to `output/logs/<run_name>`, and update runtime defaults, checked-in runtime JSON, Python tools, CI artifact paths, docs, and tests accordingly.
- [x] Sweep compile-time value constants that are currently PascalCase. For log-product/filter dimensions such as `MeasurementDimension` and `StateDimension`, use Kalman-standard notation `M` for measurement dimension and `N` where helpful.
- [x] Preserve the conceptual split: `build/` is for developer build trees, `install/` is for staged installed software, `output/` is for local/generated run outputs, `artifacts/` is for CI/CD packaged evidence, and `config/` is for input configuration.

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
- [x] Add runtime tests for important incorrect-input or rejected-operation behavior where failure is expected and should be stable.
- [x] Document the testing standard: completeness, clarity, and design intent matter more than tests for tests' sake.

## Runtime profiling and resource evidence

### Pass 3.4a â€” Desktop workflow timing artifacts

- [x] Add simple script-level timing around build/test/demo/analysis commands without forcing desktop-only dependencies into `navkit::core`.
- [x] Record stationary simulation wall time and analysis wall time in a machine-readable artifact, such as `output/logs/<run_name>/timing.json`.
- [x] Include metadata needed for trend review: schema version, run name, selected `NAVKIT_CONFIG`, build type, command names, start/end timestamps or elapsed seconds, and tool version where practical.
- [x] Add executable/library size reporting for Debug and Release artifacts as an early coarse resource signal.
- [x] Add CI or tool-wrapper hooks that preserve timing/resource artifacts as uploaded artifacts without making stochastic or machine-dependent values brittle pass/fail gates.
- [x] Keep these desktop timing artifacts useful for future Monte Carlo runtime summaries and batch trade studies.

### Pass 3.4b â€” Embedded-ready profiling policy architecture

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

### Pass 3.4c â€” First algorithm integration points

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

### Pass 3.4e - Resource evidence and hot-path profiling discipline

- [x] Document the resource-evidence philosophy: desktop timing, binary-size, and trace summaries are useful trend signals, not embedded guarantees. Hard budgets require a selected target profile, compiler, clock source, memory map, and concurrency model.
- [x] Keep current product-core hot paths honest by construction: no dynamic allocation, string/file/JSON ownership, runtime polymorphism, or desktop IO in embedded-facing profiling loops.
- [x] Add lightweight profiling contract tests that the no-op profiler path is empty/trivial/noexcept and that active profile records/sinks remain fixed-size, non-polymorphic, and capacity-bounded.
- [x] Document that assembly inspection, active-vs-no-op cycle comparisons, high-water memory checks, and hardware-specific benchmarks belong in a future target-profile pass rather than brittle desktop unit-test gates.

**Exit criteria:** configuration extension points are named and concept-tested; Debug/Release compiler flags and static-analysis expectations are documented and exercised by repository tooling; the test suite has an explicit coverage/design-intent baseline; and desktop timing/resource evidence is generated by repository tooling before the next major propagation/mechanization expansion.

---

# Phase 4 â€” Navigator and propagation seam

**Goal:** Introduce propagation as an orchestration capability without implementing full INS mechanization yet.

- [x] Define the minimum `FilterPolicy` actually required by Navigator.
- [x] Define `SensorCollectionPolicy` only around operations Navigator truly uses.
- [x] Formalize the existing update-policy capability.
- [x] Define candidate-first `PropagationPolicy` using the current concrete Navigator context, `Filter&` and `Sensors&`, rather than inventing IMU/mechanization input types before the first algorithm spec exists.
- [x] Implement `NoOpPropagation` to preserve current GNSS-only behavior.
- [x] Refactor Navigator to orchestrate propagation, sensor processing, and update policy application.
- [ ] Revisit whether a standalone `NavigatorPolicy` is useful once Navigator owns construction, initialization handoff, propagation, sensor processing, and update orchestration. Expected requirements would include aliases such as `Filter_t`, `Sensors_t`, `Update_t`, and `Profiler_t`, accessors for filter and sensors, and `process_measurements()` or its propagation-aware successor; do not implement this concept until a real consumer needs it.
- [x] Keep Navigator unaware of planet, gravity, and frame types; those belong inside propagation/mechanization configuration.
- [x] Add valid and invalid compile-time tests for filter, sensor collection, propagation, and update boundaries.
- [x] Verify the stationary GNSS numerical baseline remains unchanged with `NoOpPropagation`.

**Exit criteria:** Navigator accepts a propagation policy, the no-op configuration reproduces current behavior, and the public orchestration boundary is concept-tested.

---

# Phase 5 â€” Navigation physics and simulation contracts

**Goal:** Establish correct truth and sensor contracts before trusting an INS implementation.

## Pass 5a â€” Focused ECEF navigator algorithm document

- [x] Create a dedicated LaTeX algorithm-spec folder, `docs/algorithms/navigator_ecef_v1/`, with a `main.tex` entry point and separate section/chapter `.tex` files.
- [x] Scope the first mechanization narrowly: one IMU, body frame collocated with the IMU and navigation center, no IMU lever arm, no multiple-IMU fusion, no arbitrary center of navigation, and no runtime-polymorphic algorithm selection. Non-IMU aiding sensor lever arms, beginning with GNSS antenna position/velocity, are in scope.
- [x] Define the first concrete ECEF navigation algorithm contract before implementation: frames, state, IMU input semantics, timing/discretization assumptions, nominal propagation equations, error-state propagation expectations, covariance propagation, and normalization/reset conventions.
- [x] Explicitly mark deferred generalizations: multiple IMUs, IMU lever arms, estimated sensor lever-arm calibration states, non-collocated centers of navigation, alternate mechanization frames, higher-order integration beyond the first coning/sculling baseline, and target-specific numerical/performance budgets.
- [x] Link the spec back to the implementation passes that will consume it so real propagation policy work is driven by written math rather than aspirational generic APIs.
- [x] Treat this pass as a documentation/design gate. Do not implement the real ECEF mechanization, IMU buffering, covariance propagation, or public INS Navigator API until this algorithm document exists.

## Pass 5a.1 â€” Code-ready ECEF/GNSS math refinement

- [x] Refine the `navigator_ecef_v1` notation so formal LaTeX uses conventional DCM/quaternion frame notation such as `C_b^e` and `q_b^e`, while code remains free to use explicit `b2e`/`e2b` variable names where clearer.
- [x] Define truth, estimate, measured, and error-state notation explicitly, including the Groves-style ECEF-resolved small-angle attitude error convention and left-multiplicative quaternion correction.
- [x] Change the IMU contract to increment-based inputs with coning and sculling compensation in scope for the first implementation.
- [x] Show the full reduced continuous-time `F` and `G` matrices for the v1 15-state INS error model, including configured gravity-gradient/J2 expectations.
- [x] Replace the Van Loan implementation direction with an analytical discrete-transition/process-noise formulation suitable for code implementation.
- [x] Add loosely coupled GNSS position and velocity observation equations with configured antenna lever arm and explicit `H` matrix blocks.

## Pass 5b â€” IMU emulator/error-model algorithm document

- [x] Create a standalone LaTeX IMU emulator/error-model algorithm document under `docs/algorithms/`, similar in structure to `docs/algorithms/navigator_ecef_v1/`.
- [x] Use `docs/navigation_reference` section 2.2 as source material, but clean up the notation into a narrow code-ready implementation reference rather than copying the broader reference notation.
- [x] Define the v1 scope explicitly: one body-collocated IMU, body-frame truth inputs, no IMU lever arm, no multiple IMUs, and no online calibration-state estimation.
- [x] Define frames, units, truth inputs, raw triad outputs, calibrated quantities, and increment output fields with code-facing names.
- [x] Lay out the full forward triad error model for gyro and accelerometer independently: bias, optional bias random walk, diagonal scale factor, installation/mounting misalignment, internal non-orthogonality, white measurement noise, and optional quantization.
- [x] Specify the calibration/error-model ordering explicitly, including the inverse calibration form used by downstream navigator correction paths.
- [x] Include discrete-time algorithm requirements for software implementation: interval averaging, bias random-walk update, white-noise discretization, raw rate/specific-force generation, increment integration, optional quantization, timestamp convention, and deterministic seeded random draws.
- [x] Define runtime configuration fields and validation expectations for ideal and non-ideal IMU emulator modes.
- [x] Define required tests tied to equations: ideal output, constant bias, scale-factor response, cross-axis response from misalignment/non-orthogonality, seeded noise reproducibility, bias random walk statistics, quantization behavior, increment units, and sample timing.
- [x] Include all equations needed for software implementation; defer full appendix-style line-by-line derivations until later.

## Pass 5c â€” IMU increment contract and emulator implementation

- [x] Define the product-core IMU increment sample type consumed by `navigator_ecef_v1`, including timestamp, sample interval, incremental angle `delta_theta_ib_b_rad`, incremental velocity `delta_v_ib_b_mps`, and explicit pre/post-correction semantics.
- [x] Update truth generation enough to support ideal stationary ECEF IMU truth: trajectory providers should emit ECEF truth states only (`t`, `p_eb_e`, `v_eb_e`, and body-to-ECEF attitude), while the IMU emulator derives body-frame inertial angular increments and specific-force increments from successive truth samples with explicit Earth-rotation compensation.
- [x] Implement an ideal IMU emulator first: deterministic timestamped increments from truth with no bias, noise, scale factor, misalignment, non-orthogonality, or quantization.
- [x] Define an IMU triad error-model vocabulary for gyro and accelerometer independently. The model should preserve the substance of `docs/navigation_reference` section 2.2 without inheriting its notation:
  - [x] bias and optional bias random walk;
  - [x] diagonal scale factor;
  - [x] installation/mounting misalignment;
  - [x] internal non-orthogonality;
  - [x] white measurement noise;
  - [x] quantization/noise increment behavior when needed.
- [x] Document and test the calibration ordering explicitly, using clear code-facing names. A suitable first-order forward model is `raw = (I + scale) * nonorthogonality * misalignment * truth + bias + noise`, with separate gyro and accelerometer parameter sets.
- [x] Add deterministic runtime configuration for IMU emulator parameters and random seeds, with a reusable parser/validator ready for the selected app config that will first wire IMU into the simulation loop.
- [x] Add focused emulator tests before navigator integration: ideal output, constant bias, scale-factor response, cross-axis response from misalignment/non-orthogonality, deterministic seeded noise, increment units, and sample timing.
- [x] Tighten the embedded-facing simulator contract: keep `TruthSample` trajectory-only, move reusable quaternion/skew helpers out of the IMU implementation, prefer `bool`/out-parameter runtime failure handling over exceptions and `std::optional`, and rename `ideal_interval_from_truth_ecef()` with frame suffix ordering.
- [x] Keep IMU lever arms, multiple IMUs, online scale-factor/misalignment estimation, and latency/replay handling out of v1 unless a later pass deliberately expands scope.

## Pass 5d â€” Strapdown aided navigator implementation

- [x] Add the initial Navigator API seam from the completed `navigator_ecef_v1` algorithm document: `Navigator::update()` is the normal orchestration call, and the explicit stage methods are `process_strapdown_integration()`, `propagate_covariance()`, and `process_measurements()`.
- [x] Add the first propagation-policy seam while keeping `NoOpPropagation` behavior-preserving for measurement-only products. The first implementation proved the ECEF INS path, then the follow-up cleanup moved covariance ownership back to the filter: propagation policies own strapdown math plus `F_k`/`G_k`/`Phi_k`/`Q_d` construction, while `KalmanFilter` owns covariance propagation.
- [x] Move the selected simulation app loop to `Navigator::update()` so executable orchestration no longer couples itself to the temporary measurements-only path.
- [x] Add focused reusable implementation primitives before broad app wiring: rotation-vector/quaternion conversion helpers, two-sample coning/sculling helpers, ECEF PVA strapdown helpers, interval `F_k`/`G_k` builders, first-order `Phi_k`/`Q_d`, and symmetry-preserving covariance prediction tests.
- [x] Add typed IMU ingestion to `Navigator` with fixed-capacity internal buffering. The normal client contract is `push_imu(...)` followed by `update()`, with `process_strapdown_integration()`, `propagate_covariance()`, and `process_measurements()` available as explicit stage methods for tests and advanced users.
- [x] Implement the first ECEF v1 propagation policy against the current split-state reality: propagate nominal body-to-ECEF attitude as a unit quaternion while retaining the 3D `Att` segment as the small-angle attitude error state used by covariance, Jacobians, and injection.
- [x] Keep the v1 selected configs aligned with the algorithm document by using `DefaultInsStateDef`: a 15D PVA+gyro-bias+accelerometer-bias error/covariance layout with quaternion nominal attitude storage. Leave online scale-factor/misalignment estimation for a deliberately designed future pass.
- [x] Wire the selected stationary simulation loop to generate IMU increments from the new IMU emulator and feed the Navigator through the new typed ingestion path, while preserving existing GNSS-position logging and analysis behavior.
- [x] Implement the first real INS path from the `navigator_ecef_v1` equation-to-code map, using the Pass 5c IMU emulator as the primary data source:
  - [x] Mechanization primitives: rotation-vector/quaternion conversion, quaternion propagation sign tests, two-sample coning/sculling, ECEF PVA nominal propagation, and gravity/gravity-gradient policy calls.
  - [x] Covariance prediction primitives: build `F_k`, build `G_k`, first-order `Phi_k`, first-order `Q_d`, filter-owned covariance propagation, symmetry checks, and equation-block sanity tests.
  - [x] Navigator integration: typed IMU ingestion, fixed-capacity internal buffers, `update()` orchestration, and explicit stage-method tests.
  - [x] Aiding integration beyond the existing GNSS position path is deliberately moved to Pass 5e.4 after IMU-only propagation diagnosis is stable.
- [x] Define typed IMU data ingestion using the real IMU sample contract. Clients push timestamped IMU increments through `push_imu(...)`, then call `Navigator::update()`.
- [x] Correct the propagation/filter ownership boundary after the first working navigator pass: remove filter/sensor knowledge from propagation policies, move `P = Phi P Phi^T + Q_d` into `KalmanFilter::propagate_covariance(...)`, and keep `Navigator` as the owner of orchestration and stage sequencing.
- [x] Tighten the first IMU/runtime cleanup: expose 1024-sample IMU buffer capacity in product configs, support `rate_hz` or `dt_s` runtime rate input with exactly-one validation where applicable, default truth and IMU to 1000 Hz for simple v1 timing, and move IMU-specific simulator setup out of the central simulation loop.
- [x] Move reusable implementation pieces toward their owning domains: quaternion/RPY conversion helpers under math, coning/sculling helpers under Navigator IMU support, diagonal random draws under simulation utilities, and zero process-noise policy into its own propagation header.
- [ ] Define buffer ownership and timing rules for v1 beyond the current FIFO IMU buffer. Simple history buffers for inspection and future replay support are acceptable, but full latency handling, measurement replay, and delayed-measurement correction are explicitly out of scope.
- [x] Move INS observability/logging/analysis expansion to the dedicated Pass 5e.3 so this implementation pass can focus on stabilizing the single-IMU ECEF v1 path.
- [x] Move runtime scenario JSON splitting to Pass 5e.4 so this implementation pass can focus on stabilizing the single-IMU ECEF v1 path.
- [ ] Defer full delayed-measurement replay, multi-IMU support, IMU lever arms, estimated scale-factor/misalignment states, and richer timing/latency handling until the single-IMU ECEF v1 path is numerically stable.

## Pass 5e.1/5e.2 â€” Navigator stabilization, timing, and diagnosis cleanup

- [x] Add explicit medium-rate covariance propagation configuration to the selected NavKit product configs. Start with a compile-time default of 100 Hz because this rate informs fixed-size STM/process-noise history capacity and future delayed-measurement support.
- [x] Add a bounded covariance-step history buffer owned by `Navigator`, defaulting to 256 entries for the first implementation. Keep the current accumulated pending covariance step for the normal no-latency path, but retain enough recent `Phi`/`Q_d` history for inspection and future latent-measurement replay.
- [x] Add `Navigator::StateDef_t = typename Filter_t::StateDef_t` and use it to consolidate repeated state-definition spelling inside Navigator.
- [x] Normalize math helper names: use `quaternion_from_rpy_rad()` / `rpy_rad_from_quaternion()` for roll-pitch-yaw Euler conversions, and add `quaternion_from_rotvec_rad()` / `rotvec_rad_from_quaternion()` for small-angle or Rodrigues rotation-vector use cases. Keep nominal-state attitude and error-state perturbation vocabulary separate in names and tests.
- [x] Migrate from transitional RPY nominal attitude storage to the `navigator_ecef_v1` split-state design by making `DefaultInsStateDef` an aggregate selected state-space definition: `DefaultInsNominalStateDef` stores body-to-ECEF attitude in `AttQuat`, while `DefaultInsErrorStateDef` keeps the covariance/error attitude as the 3D small-angle `AttRotVec` perturbation.
- [x] Leave PVA sub-selection alone for now. Do not introduce PVA state views or copies unless a zero-runtime-cost view abstraction becomes clearly worthwhile.
- [x] Standardize function signatures in Navigator/propagation code: input parameters first as `const&`, in/out parameters second as non-const references, and output-only parameters last. Add this rule to the agent/design guidance before applying broad cleanup.
- [x] Add a tight INS diagnosis pass before expanding aiding: create an IMU-only stationary validation configuration or test harness, run ideal stationary truth with ideal IMU and no GNSS, and verify that pure strapdown propagation holds ECEF position/velocity/attitude within tight tolerances over a short run.
- [x] Add focused logging or assertions for the diagnosis pass around one or a few IMU intervals: `delta_theta_ib_b`, `delta_v_ib_b`, gravity, specific force, and resulting velocity/position increments. Use this to isolate ECEF/inertial rotation sign, quaternion/DCM direction, gravity/sign, and GNSS Jacobian/update sign issues before masking them with additional aiding.
- [x] Add runtime-configurable console/log output rates. Default the console heartbeat to 1 Hz and include time, position, velocity, and attitude; keep file logging rates separate from truth/IMU system rates so 1000 Hz simulation does not force 1000 Hz logs.
- [x] Close the roll/pitch attitude-drift regression after the quaternion convention debugging pass. The active v1 convention is body-to-ECEF nominal quaternion storage, vector transform `v_e = q_b2e * v_b`, left-multiplicative small-angle injection `C_true ~= (I + [delta theta]x) C_hat`, and `q_b2e+ = delta_q(delta theta) * q_b2e-`.

## Pass 5e.3 â€” INS observability, IMU debug logs, and full-state plotting

- [ ] Add queryable/interpolated truth sampling so consumers can request truth at arbitrary times without forcing all downstream processing to run at the trajectory generation rate. Keep truth generation/system rate separate from truth logging rate.
- [x] Keep runtime-configurable log rates independent for truth, navigation, measurement statistics, IMU logs, IMU debug logs, and console heartbeat. Default high-rate simulation to numerically simple rates such as 1000 Hz while allowing logs to default to lower inspection-friendly rates.
- [x] Add or extend truth trajectory logs for the complete v1 truth contract: time, ECEF position, ECEF velocity, and body-to-ECEF attitude. Preserve frame/unit suffix naming and schema metadata.
- [x] Add IMU increment log products for both truth/ideal increments and measured emulator increments:
  - [x] Log `delta_theta_ib_b_rad`, `delta_v_ib_b_mps`, timestamp, and interval `dt_s`.
  - [x] Include cumulative sums of truth/ideal and measured increments so bias, drift, quantization, and sign errors are visible without staring at noisy high-rate samples.
  - [x] Add Python plots for raw increment time histories and cumulative-sum comparisons, grouped by gyro/incremental-angle and accelerometer/incremental-velocity channels.
- [x] Add explicit compile-time-selected IMU simulator debug logging for implementation-internal interval quantities needed to debug the truth-to-IMU conversion:
  - [x] Include ECEF terms such as `p_bar_e_m`, `v_bar_e_mps`, `a_bar_e_mps2`, `gravity_e_mps2`, and `specific_force_e_mps2`.
  - [x] Include body/inertial terms such as `delta_theta_eb_b_rad`, `delta_theta_ib_b_rad`, `specific_force_ib_b_mps2`, and `delta_v_ib_b_mps`.
  - [x] Include time-varying emulator error states such as gyro and accelerometer biases when enabled.
  - [x] Keep this as an explicit debug log product or compile-time diagnostics path rather than leaking IMU internals into `SimulationApp`.
- [ ] Add dedicated IMU simulator runtime/config manifests beside the run logs so analysis can verify seed, rate, noise PSD/covariance, bias, scale factor, misalignment, non-orthogonality, quantization, and enabled debug-log settings. The current run manifest already stores the selected runtime config; this item tracks richer per-IMU manifest/schema detail.
- [x] Generalize Kalman filter/navigation error logging beyond ECEF position:
  - [x] Add a filter covariance logging mode that can record either covariance diagonal only or the full upper/lower triangular covariance matrix. Keep diagonal logging as the lightweight default, and enable triangular logging when downstream transforms require cross-correlation terms.
  - [x] Generate one log/schema path that records the full configured error-state correction vector and matching covariance data for every state in `StateDef::Error`.
  - [x] Keep the existing ECEF position error/covariance plot behavior as the first specialization/view, but make the same 1-sigma and 3-sigma bound plots available for every configured error-state component.
  - [x] Add correction-within-covariance plots that show filter error-state corrections against covariance bounds. These plots must not require truth, so they are valid for both real runs and simulations.
  - [x] Add simulation-only truth-error-within-covariance plots that interpolate truth to nav/log timestamps, calculate actual navigation errors, and plot those errors against covariance bounds.
  - [x] Include attitude-error, gyro-bias, and accelerometer-bias plots for `DefaultInsErrorStateDef`, with frame/unit labels taken from schema helpers or explicit state-def metadata where available.
  - [x] For IMU bias truth-error plots, expose/log the simulator's truth bias state as turn-on bias plus in-run bias/random-walk state, compare it against the estimated gyro/accelerometer bias states, and plot the resulting bias errors inside the corresponding covariance bounds.
  - [x] Add ECEF-to-local-level plots for position, velocity, and attitude errors/covariance. Use a clearly named sensitivity/rotation matrix from ECEF to NED, apply `P_ned = G P_ecef G^T` where the full covariance is available, and keep diagonal-only logging explicitly limited to component-wise ECEF plots.
  - [x] Add analysis plots for full-state correction histories, truth-error histories, covariance bounds, and transformed NED bounds where truth is available.
  - [ ] Add optional normalized error-ratio plots where truth is available.
  - [x] Preserve existing GNSS innovation, NIS, p-value, and histogram outputs while making the state-error plotting path independent of GNSS-specific assumptions.
- [ ] Add tests for the new log-product schemas and plotting input contracts. Favor small deterministic CSV/log fixtures where possible so plotting failures are caught without requiring long stochastic simulation runs.

## Pass 5e.4 â€” Logging contract cleanup, trajectory initialization, and `auto` audit

- [x] Update `AGENTS.md` with the stronger repository rule for `auto`: prohibit `auto` by default, except for narrow obvious cases such as iteration variables, lambda/closure types, unavoidable template abstraction, or immediately consumed expressions where the concrete type is irrelevant. Explicit concrete types are required for stored/reused Eigen expressions, math temporaries, domain values, dimensions, state vectors, matrices, units, and frame-bearing quantities.
- [ ] Continue the dedicated `auto` deep-dive audit across the codebase. The first pass replaced high-risk nontrivial `auto` usages in touched simulation, propagation, logging, and analysis-facing math code; a broader repository-wide audit remains.
- [x] Rename default log products/files to include `ecef` where the frame is part of the contract, now that local-level/NED products also exist.
- [x] Split runtime logs into fact logs and derived analysis artifacts:
  - [x] Keep a dedicated truth trajectory log with only timestamp, ECEF position, ECEF velocity, and body-to-ECEF attitude.
  - [x] Keep a dedicated nominal navigation/filter estimate log with timestamp, nominal state estimate, attitude representation, and covariance data. Support diagonal-only covariance as the default and full triangular covariance when downstream transforms need cross-correlation terms.
  - [x] Keep `filter_correction` as an event log that writes only when corrections actually occur; do not fill the file with zero rows on propagation-only timesteps.
  - [x] Move truth-error calculation out of primary C++ logging and into Python post-processing: load truth and estimate logs, interpolate truth to estimate timestamps, compute simulation-only errors, and optionally write derived truth-error CSV artifacts.
- [x] Redesign the monolithic truth-error/covariance dashboard while keeping the detailed broken-out plots available:
  - [x] Use one subplot per kinematic/error-state family with all axes on the same subplot.
  - [x] Use a two-column layout: left column position, velocity, attitude; right column gyro bias and accelerometer bias.
  - [x] Use consistent axis colors, such as x/roll red, y/pitch blue, and z/yaw green.
  - [x] Plot errors and matched 3-sigma bounds only; omit 1-sigma bounds from the dashboard to reduce clutter.
  - [x] Label attitude plots as roll, pitch, and yaw rather than north/east/down.
- [x] Add dedicated IMU error/bias plots outside the mega-dashboard. Compare simulator truth gyro/accelerometer bias states against estimated bias states, plot bias errors inside covariance bounds, and label axes using the configured body-axis convention.
- [ ] Split IMU logs by responsibility:
  - [x] nominal IMU simulator log for truth/ideal and measured increments plus cumulative sums;
  - [x] optional IMU debug log for implementation-internal interval quantities;
  - [ ] richer IMU runtime/config manifest that records enabled error sources, seed, rates, covariance/PSD settings, quantization, coning/sculling setting, and debug-log setting.
- [x] Extend trajectory initial-condition parsing for the current stationary/runtime trajectory path without introducing full 6-DOF dynamics yet:
  - [x] Support ECEF initial values such as `p_e_m`, `v_e_mps`, and one attitude form among `q_b2e`, `rpy_b2e_rad`, or `dcm_b2e`.
  - [x] Support local-level initial values such as `p_lla_deg_m`, `v_n_mps`, and one attitude form among `q_b2n`, `rpy_b2n_rad`, or `dcm_b2n`, converting internally to canonical ECEF truth state.
  - [x] Add initial angular-rate parsing for `w_ib_b_radps`, `w_eb_b_radps`, and `w_nb_b_radps`, with explicit validation that only one angular-rate convention is selected.
  - [x] Default stationary body axes to the aircraft convention x-forward/nose, y-right-wing, z-down by deriving the initial body frame from local-level/NED attitude when provided.
- [x] Add coning/sculling compensation configurability on both sides of the IMU contract:
  - [x] IMU simulator can emit raw/uncompensated or compensated increments.
  - [x] ECEF INS propagation can apply compensation or assume increments are already compensated.
  - [x] Add validation or clear manifest metadata to make double compensation or missing compensation obvious.
- [x] Keep full rigid-body 6-DOF trajectory dynamics out of this pass. Record the future direction separately: mass, CG, inertia, force/moment models, aerodynamic effects, and closed-loop guidance should become a dedicated trajectory-dynamics pass after the initial-condition and truth-provider contract is stable.

## Pass 5e.5 â€” Combined logging, config, IMU contract, and plotting cleanup

- [x] Keep the existing `ImuRuntime::process(...)` overload shape unless implementation work reveals a concrete ownership bug. Do not churn the API just because one call currently ignores a returned diagnostic sample.
- [x] Replace `IdealImuInterval` with clearer interval vocabulary:
  - [x] `ImuInterval` is the canonical simulation/math interval data: `time_s`, `dt_s`, `omega_ib_b_radps`, and `specific_force_ib_b_mps2`.
  - [x] `ImuIntervalDebug` is optional debug-only data: average position/velocity/acceleration, gravity, ECEF specific force, and intermediate ECI/body-frame terms needed for diagnosis.
  - [x] `ImuIncrement` remains the sampled IMU output contract: `time_s`, `dt_s`, `delta_theta_ib_b_rad`, and `delta_v_ib_b_mps`.
- [x] Remove IMU ECEF debug logging/plotting that is unused or hard-coded to zero. Keep a body/inertial IMU debug product, for example `imu_debug_body`, with ECI/body quantities resolved in the body frame.
- [x] Move reusable IMU calibration helpers out of simulator implementation code:
  - [x] Put scale-factor, misalignment, and non-orthogonality triad calibration helpers in `include/navkit/core/math/TriadCalibration.hpp`.
  - [ ] Move reusable Earth-rate, frame-transform, and trajectory-provider conversion helpers into their owning core math/frames/environment locations instead of leaving them buried in trajectory or simulator code.
- [x] Make coning/sculling compensation compile-time configured on both sides of the contract:
  - [x] IMU simulator emits either raw/uncompensated or compensated increments according to its selected compile-time config.
  - [x] ECEF INS propagation either applies coning/sculling compensation or assumes increments are already compensated.
  - [x] Let the app config set the simulator-side compensation flag from the NavKit propagation choice when appropriate, for example the logical opposite of the selected Navigator compensation setting, and record the choice in manifests.
- [x] Split runtime output under `output/logs/<run>/` into `data/` and `figures/`, and update Python analysis to read/write through that structure. Keep runtime-configurable log rates for each file family; high-rate truth/IMU generation must not imply high-rate logging.
- [x] Split logs by responsibility:
  - [x] dedicated truth trajectory log only: time, position, velocity, and attitude;
  - [x] dedicated nominal navigation/filter estimate log with estimate and covariance data;
  - [x] dedicated filter-correction event log that records corrections only and does not duplicate covariance owned by the estimate log;
  - [x] remove `nav.csv` unless it is deliberately replaced by a clearer product such as a PVA-specific estimate log.
- [x] Rename default frame-bearing log and figure products so the frame suffix is at the end, for example `error_covariance_position_ecef.png`, `error_covariance_position_ned.png`, and matching ECEF/NED CSV names.
- [x] Add ECEF plots for all position, velocity, and attitude states in addition to the existing NED views. Keep NED plots for local-level interpretability.
- [x] Update plot units and labels:
  - [x] attitude error plots in degrees, labeled roll/pitch/yaw rather than NED axes;
  - [x] accelerometer bias/error plots in micro-g;
  - [x] gyro bias/error plots in milli-degrees per second or another clearly documented small-angle-rate unit;
  - [x] individual IMU error covariance plots use black covariance bounds and red error series.
- [x] Redesign the roll-up dashboards:
  - [x] keep detailed broken-out plots in the existing per-state format;
  - [x] specialize `error_covariance_dashboard_*` with one subplot per state family and all axes overlaid;
  - [x] use a two-column layout: position/velocity/attitude on the left, gyro bias/accelerometer bias on the right;
  - [x] color-match x/roll, y/pitch, z/yaw errors and 3-sigma bounds; omit 1-sigma bounds from dashboards;
  - [x] consolidate the legend into a single vertical legend in the available blank panel area;
  - [x] update `filter_correction_covariance_*` to match the same dashboard layout, but only for actual correction events.
- [x] Increase default initial covariance examples to realistic first-cut values: 100 m position, 10 m/s velocity, 5 deg roll/pitch tilt, and 10 deg yaw. Prefer deriving these as scaled-up versions of initialization-error-generation covariance when that relationship is explicit.
- [x] Split PVA startup types by meaning: `PvaStateDef` stores position, velocity, and RPY startup state, while `PvaErrorStateDef` stores position, velocity, and small-angle attitude error. Keep `PvaState = State<PvaStateDef>` and `PvaCovariance = StateCov<PvaErrorStateDef>` so covariance cannot pretend to be an RPY state.
- [x] Add `pva_error_frame` for `pva_random` initialization so random covariance draws can be interpreted in ECEF or NED before conversion to the internal ECEF-resolved error convention.
- [x] Rename lingering profiled stationary compile-time configs to `ProfiledEcefInsGnss` so compile-time config names describe the compiled product graph rather than a runtime trajectory scenario.
- [x] Add explicit runtime log `enabled` flags with validation: enabled logs must provide `dt_s` or `rate_hz`, disabled logs may omit cadence.
- [x] Clean dashboard legends so ECEF dashboards use x/y/z error and 3-sigma pairs, while NED dashboards show local N/E/D and body x/y/z pairs in the blank dashboard panel.
- [x] Revisit gyro-bias initialization covariance relative to the configured deterministic IMU bias and adjust the default first-cut value to a more realistic milli-degree-per-second scale.
- [x] Add or update scenario configs for the current validation set:
  - [x] ideal truth reconstruction with GNSS and IMU errors disabled, expecting nearly zero reconstruction error and no discontinuities;
  - [x] modeled IMU turn-on bias, in-run bias/random walk, and Gaussian white noise enabled;
  - [x] additional unmodeled IMU error sources enabled to inspect estimator robustness;
  - [x] full IMU debug logging enabled for diagnosis.
- [x] Defer compile-time config decomposition into reusable component headers to a dedicated follow-up; runtime JSON decomposition is complete enough for the current scenario work.
- [x] Begin runtime JSON decomposition into reusable component JSON files referenced from a scenario master JSON. Linked sub-files are resolved relative to the master input JSON path, for example `./otherfile.json` or `imu/imu_config.json`.
- [x] Keep trajectory selection and profile details runtime-configurable. In this cleanup pass, support initial angular-rate parsing via `w_ib_b_radps`, `w_eb_b_radps`, and `w_nb_b_radps`; full force/moment 6-DOF dynamics remain a later trajectory-dynamics pass.
- [x] Add or refresh tests for log schemas, runtime JSON include/link resolution, plot input contracts, IMU interval naming, compensation flag compatibility, and decomposed config examples.

## Pass 5e.6 — Roadmap consolidation and bias covariance tuning

- [x] Reconcile the recent 5e.4/5e.5 logging/config/plotting passes with the next GNSS and runtime-hygiene passes so finished points are marked complete and unresolved points are not left hidden in completed passes.
- [x] Move broader follow-up work, including compile-time component-header decomposition, IMU 1-sigma randomization fields, IMU error-term naming, IMU limits, covariance floors, and repository-wide `auto` audit work, into a dedicated future hygiene pass.
- [x] Move filter initial covariance into a selected immutable NavKit product config value using `InitialCovariance<StateDef>` and `diagonal_initial_covariance<StateDef>()`; tune the current examples to variance values corresponding to 50 milli-deg/s gyro bias and 100 micro-g accelerometer bias. App-support initialization may consume an explicitly configured runtime JSON override, but must not own fallback tuning constants or state-specific covariance defaults.
- [x] Sweep app-support runtime parsing for hidden product/scenario fallback defaults. Require runtime-owned fields such as run name, output directory, logging cadences, simulator seeds/noise settings, trajectory duration, and trajectory cadence to come from runtime JSON instead of app-support helper literals.
- [x] Add focused test coverage for the tuned default IMU bias covariance values.

## Pass 5e.7 — GNSS lever arm, sensor debug products, and GNSS velocity aiding

- [ ] Add GNSS antenna lever-arm support before expanding velocity aiding:
  - [ ] configure the lever arm in the simulator/runtime path;
  - [ ] apply the lever arm in GNSS truth measurement generation;
  - [ ] include the lever-arm contribution in `GnssPosModel` and its `H` matrix;
  - [ ] add focused finite-difference Jacobian tests.
- [ ] Add typed sensor debug logs and plots for simulated sensors:
  - [ ] log the literal truth measurement and measured measurement in the measurement's own coordinates, such as truth/measured range-azimuth-elevation for a future radar or truth/measured ECEF position for GNSS position;
  - [ ] do not require explicit measurement-error columns when the truth and measured quantities are sufficient to derive errors in analysis;
  - [ ] plot truth and measured quantities overlaid, using black truth markers and green measured markers, with a lower error subplot when the error is derivable;
  - [ ] plot measurement covariance bounds when available.
- [ ] Add GNSS velocity aiding after lever-arm position support is stable. Wire the existing GNSS velocity model into the selected simulation/app config, include antenna lever-arm velocity terms, update analysis/logging products, and add finite-difference Jacobian tests.
- [ ] Revisit realistic GNSS error/noise defaults once position and velocity aiding are both active.

## Pass 5e.8 — Runtime hygiene, IMU config randomization, and covariance guardrails

- [ ] Finish compile-time config decomposition into reusable `components` headers plus scenario-level product/app headers now that trajectory behavior is runtime-selected rather than compile-time-selected.
- [ ] Extend IMU runtime error config with optional 1-sigma fields for runtime random draws. If a user provides a `*_1sig` field, validation should interpret the configured field as a one-sigma bound and populate the concrete error value from a deterministic seeded random draw.
- [ ] Revisit IMU error-term vocabulary in JSON and `ImuTriadErrorConfig`: consider standard names such as `bias_turnon`, `bias_inrun`, and clearly documented `random_walk`/white-noise semantics instead of the current mixed terminology.
- [ ] Add IMU acceleration and angular-rate limit support to the simulator/runtime path.
- [ ] Add configurable covariance floors per state-family/diagonal block to prevent covariance from becoming ill-conditioned or singular during idealized analysis runs.
- [ ] Continue the repository-wide `auto` audit using the strengthened AGENTS rule. Replace nontrivial `auto` in math, Eigen, state, frame/unit, simulator, and logging code with explicit types unless it falls into the narrow allowed cases.

## Frames, coordinates, and environment

- [ ] Define the minimum coordinate operations needed by PCPF/ECEF mechanization and local-vertical measurements.
- [ ] Confirm position, velocity, attitude, angular-rate, and specific-force frame conventions in code and documentation.
- [ ] Integrate `environment::Wgs84` and the selected gravity policy into physics code without adding an Earth-specific framework layer.
- [ ] Add tested helpers for required ECEF/geodetic/local-vertical conversions.

## Truth generation

- [x] Keep `TruthSample` trajectory-only for v1: timestamp, ECEF position, ECEF velocity, and body-to-ECEF attitude. Acceleration, angular-rate, and specific-force truth are derived by simulator/emulator components rather than stored as primary trajectory fields.
- [ ] Add a trajectory-source abstraction so generated trajectories and CSV/playback trajectories feed the same downstream hooks. Do not create a separate playback driver unless the shared trajectory-provider path cannot express the required replay behavior.
- [x] Priority 1: flesh out stationary ECEF trajectory/emulator validation with proper Earth rotation, nonzero stationary IMU gyro increments, and physically meaningful derived specific-force increments. This scenario supports future gyrocompassing, coarse alignment, and ZUPT validation work.
- [ ] Priority 2: add a simple ballistic trajectory: stationary launch-pad initialization, optional transfer-alignment window, initial heading/pitch definition, and a simple axial body-x boost profile before ballistic/coast behavior. Keep this intentionally simple before adding aero or guidance complexity.
- [ ] Priority 3: add a constant-altitude, constant-speed trajectory on the curved Earth rather than flat-Earth kinematics, suitable for longer-duration navigation validation.
- [ ] Add a calibration-maneuver trajectory after the first three truth modes are stable: horizontal S-turn, vertical S-turn, and bank-left/bank-right excitation for observability and calibration studies.
- [ ] Add a basic waypoint trajectory with simple bank-to-turn behavior once coordinate, attitude, and trajectory-source contracts are stable.
- [ ] Add straight-line, constant-rate-turn, or circular-motion helpers only when they provide distinct validation value beyond the prioritized scenarios above.

## Sensor contracts

- [x] Define a timestamped IMU sample type with documented increment/rate semantics.
- [x] Implement an ideal IMU simulator first.
- [x] Add IMU white noise, bias, bias random walk, scale factor, misalignment, non-orthogonality, and quantization incrementally with deterministic seeds.
- [ ] Implement the barometer simulator and a physically meaningful altitude model/Jacobian.
- [ ] Integrate the existing GNSS velocity model into simulation.
- [ ] Add explicit sensor scheduling and multi-rate behavior.

**Exit criteria:** ideal stationary and moving truth cases have physics-based tests; ideal sensors reproduce expected measurements; frame and unit conventions are documented at their APIs.

---

# Phase 6 â€” First complete PCPF/ECEF INS

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

# Phase 7 â€” Estimator validation and consistency

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

# Phase 8 â€” Monte Carlo, logging, and performance

**Goal:** Support trade studies and quantify estimator behavior across randomized runs.

## Monte Carlo

- [ ] Add a seeded batch/Monte Carlo driver.
- [ ] Parallelize independent runs without changing determinism.
- [ ] Aggregate RMSE, NEES, NIS, failure counts, and runtime.
- [ ] Generate comparison tables and reports across configurations.

## Logging

- [ ] Add optional runtime log-product enable/disable and verbosity/detail selection once compile-time log-product composition is stable.
- [ ] Migrate or version downstream analysis intentionally when stationary GNSS file names or schemas are changed; until then, preserve compatibility in Phase 3 logging refactors.
- [ ] Add richer log/schema version metadata, compatibility checks, and migration guidance after the generic logger and product payload contracts stabilize.
- [ ] Extend model/state-derived schema helpers or logger policies beyond the first stationary GNSS products as new sensors and state definitions appear.
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

# Phase 9 â€” Robust estimator and embedded readiness

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
- [ ] Add target-specific hot-path evidence: inspect optimized assembly for no-op profiler elision, compare active-vs-no-op profiling cycle counts, and run hardware/compiler-specific microbenchmarks once target clocks and toolchains exist.
- [ ] Extend allocation/resource checks to sensor queues, estimator update operations, and propagation/mechanization once those hot paths exist.
- [ ] Extend IMU error models beyond the v1 bias-random-walk/white-noise emulator with first-order Gauss-Markov dynamics for biases and other calibration/error states once the ideal and random-walk IMU paths are validated.
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
