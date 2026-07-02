# NavKit Agent Guidance

## Scope and source of truth

This file applies to the entire repository.

- Treat the checked-in implementation and build configuration as the source of truth for current behavior.
- Read `docs/README.md` for the documentation map, `docs/ARCHITECTURE.md` for current architecture, `docs/CONFIGURATION.md` before configuration work, and `docs/ROADMAP.md` for verified status, priorities, and product direction. Confirm status claims against the repository before acting on them.
- Read ADR-001, ADR-002, and ADR-003 before architectural work. They document the intended compile-time policy direction, but all three are currently marked **Proposed**, not accepted or fully implemented.
- Preserve runtime behavior unless the task explicitly changes it. Do not turn roadmap language into implementation requirements without confirming it against the current task and code.

## Project intent

NavKit is an embedded-oriented estimation and navigation framework. The long-term design favors compile-time configuration, fixed-size Eigen types, policy-based composition, and minimal domain knowledge in runtime orchestrators. It is intended to grow beyond an EKF into navigation mechanizations and environment-independent models.

For new policy architecture, follow the intended layering where it is useful:

`concept -> optional CRTP policy base -> concrete policy -> runtime algorithm`

Concepts express capabilities; CRTP bases share implementation and are not mandatory when there is nothing useful to share.

## Current implementation reality

- The project requires **C++23** (`CMAKE_CXX_STANDARD 23`). Keep public code and supported toolchains compatible with that configured standard.
- Environment policies are the most complete realization of the target architecture: planet and gravity concepts, CRTP bases, concrete WGS84/Moon/Mars and spherical/J2 implementations, plus frame tags.
- `StateDefPolicy` and `SegmentPolicy` exist. `State<StateDef>` and `StateCov<StateDef>` are fixed-size Eigen aliases, and `InsStateDef`/`GnssTcStateDef` provide named segments.
- The estimator refactor has completed the current Phase 2 boundary pass. `InjectionPolicy`, `ResetPolicy`, `MeasurementPolicy`, and `NoisePolicy` exist. `KalmanFilter` is constrained on `StateDefPolicy`, injection, reset, and measurement-model boundaries, and `Sensor` is constrained on noise-policy compatibility. `SensorPolicy` is intentionally deferred until a real generic consumer boundary needs it. `Navigator` now has first-pass concept coverage for the current filter, sensor-collection, and update-policy boundaries. Propagation policies and `NoOpPropagation` remain future Phase 4 work.
- `KalmanFilter` performs measurement updates, stores optional per-model statistics, and delegates injection/reset. The default injection is INS-specific; the default covariance reset is intentionally a no-op.
- `Navigator` processes a tuple-like sensor collection and applies an update policy. It has no propagation/mechanization policy yet.
- INS propagation is not implemented; `ImuProcessModelPlaceholder` is only a placeholder.
- The reusable product-core CMake target is `navkit_core` with alias `navkit::core`. It is currently an `INTERFACE` target because the core is header-only/template-heavy. Desktop simulation support is built as the compiled `navkit_sim` target with alias `navkit::sim`, and desktop logging/file/JSON support is exposed through `navkit_io` with alias `navkit::io`. Executables remain applications under `apps/` and link against the libraries they need.
- Runtime virtual dispatch is avoided in embedded-facing algorithms, but it is not globally prohibited: simulation infrastructure currently uses a virtual `SensorSimulatorBase` interface.
- Python analysis is modularized under `python/navkit_analysis`; `tools/run_analysis.py` is the entry point.

## Architecture and naming rules

- Prefer compile-time polymorphism in embedded-facing and performance-sensitive code. Introduce runtime polymorphism only with a concrete need, especially outside critical paths.
- Keep runtime algorithms focused on orchestration; place planet, gravity, frames, and future mechanization details in the relevant policies.
- Organize code by product boundary first, then engineering domain. `include/navkit/core` is the reusable product core, not a miscellaneous bucket. Estimation/navigation framework domains live under `core/estimation` (`state`, `measurement`, `filter`, `sensor`, and `navigator`), shared product-core compile-time configuration vocabulary lives under `core/config`, product-core containers live under `core/containers`, physical environment models live under `core/environment`, and reusable `frames`, `units`, and `models` also live under `core`. Domain-specific configuration concepts belong beside the domain that consumes them. Desktop simulation support stays under `include/navkit/sim`; desktop logging/file/JSON support stays under `include/navkit/io`. Runtime app inputs, such as `config/runtime/navkit_sim`, stay outside public headers and are not product-core configuration.
- Keep concepts, optional CRTP bases, and concrete policy implementations in the domain that owns the abstraction. For example, filter injection/reset policies belong under `core/estimation/filter`, sensor noise policies under `core/estimation/sensor`, Navigator update/propagation policies under `core/estimation/navigator`, and planet/gravity policies under `core/environment`.
- Keep CMake targets aligned with product boundaries: `navkit::core` is the reusable product-core boundary, `navkit::sim` is simulator infrastructure, and `navkit::io` is desktop logging/file/JSON infrastructure. Application executables should stay under `apps/`. Do not add desktop-only simulation or IO behavior to `navkit::core` unless the product-core boundary decision is deliberately revisited.
- Keep CMake target kinds honest: use `INTERFACE` for header-only/template-heavy libraries, and use compiled/static libraries only when a component owns meaningful `.cpp` implementation. Do not add dummy source files just to force a static archive.
- Public namespaces mirror the folder structure through the stable domain level. Deeper leaf folders may organize implementation and policy families without adding additional namespaces unless that subdomain becomes independently meaningful. Shared product-core compile-time configuration uses `navkit::core::config`; foundational aliases such as `Scalar_t` and `Time_t` remain in `navkit::core`; other reusable product-core domains use `navkit::core::containers`, `navkit::core::estimation`, `navkit::core::environment`, `navkit::core::frames`, `navkit::core::models`, and `navkit::core::units`. Simulation and IO remain in `namespace navkit::sim` and `namespace navkit::io`.
- For configuration work, follow `docs/CONFIGURATION.md`: domain config concepts live beside the domain that consumes them, concrete app/product configs should eventually live outside public NavKit headers, and example configs plus local `static_assert` checks are the user-facing contract surface.
- Treat `NAVKIT_CONFIG` as a one-config-per-build-tree selection. Use a separate build directory or preset for each selected compile-time config; do not make one executable dynamically switch among compile-time configs.
- Name concepts `<Domain>Policy`, optional CRTP bases `<Domain>PolicyBase`, and concrete policies descriptively. Do not append `Concept` to concept names.
- For a context-dependent concept intended for constrained-parameter syntax, put the candidate type first, but keep the concept definition parameters themselves unconstrained. For example, define `template<typename Candidate, typename StateDef> concept InjectionPolicy = StateDefPolicy<StateDef> && ...`, then use it at public template boundaries as `template<StateDefPolicy StateDef, InjectionPolicy<StateDef> Injection>`.
- Use constrained public template declarations when the constraint exists and improves diagnostics. Do not claim a template boundary is policy-constrained until the concept is actually implemented there.
- Follow `docs/NAMING_CONVENTIONS.md` for navigation variables: Groves-style frame notation, `p` for position, and unit suffixes in CSV headers.
- New source files should retain the repository copyright header and All Rights Reserved notice.

## Tests and verification

- Add compile-time coverage with each new concept: positive `static_assert(Concept<Good>)` and negative `static_assert(!Concept<Bad>)`. Do not add intentionally uncompilable test targets for negative cases.
- Add every new test source to `tests/CMakeLists.txt`; merely placing a file in `tests/` does not compile or run it.
- Add runtime tests when behavior changes; compile-time assertions do not replace numerical or orchestration tests.
- Dependencies are managed by Conan 2 (`Eigen`, `nlohmann_json`, and `doctest`) and builds are driven by CMake.
- Use Python 3.10 or newer and activate the repository `.venv` when available. Conan and the analysis dependencies are intended to live in that virtual environment.
- Prefer the repository's Python tools for normal work. They are the cross-platform developer interface and handle Conan, CMake, generator, and build-directory details. Use raw Conan/CMake/CTest commands mainly for diagnosis or when a wrapper cannot express the needed operation.
- On a fresh local or cloud machine, run `python tools/bootstrap.py`. It creates `.venv`, installs Conan and the analysis package, and detects the default Conan profile. The script is idempotent and safe to rerun; do not assume separate cloud tasks share an installed environment.

After editing source, apply all source mutations before collecting build/test evidence:

```powershell
python tools/copyright.py --write
python tools/format.py
python tools/copyright.py --check
python tools/format.py --check
```

Only after those commands pass should the final build and tests run. CI uses check-only commands and must never modify source.

For the first Debug build, or when a genuinely clean rebuild is needed:

```powershell
python tools/build.py --build-type Debug --clean
```

For normal source-code iteration, after formatting and copyright checks, use the fast rebuild path:

```powershell
python tools/build.py --build-type Debug --build-only
python tools/run_tests.py --build-type Debug
```

After changing CMake files or configuration, reconfigure without reinstalling dependencies:

```powershell
python tools/build.py --build-type Debug --skip-conan
```

Use `python tools/build.py --build-type Release --clean --without-tests` when a Release compile check is relevant. Run the narrowest relevant tests during iteration and the full Debug test executable before handing off changes. Report when a test file is not part of the configured target.

For changes affecting the simulation or navigation results, also run:

```powershell
python tools/run_first_sim.py --build-type Debug
python tools/run_analysis.py data/logs/stationary_gnss_demo --show
```

Simulation logs belong under `data/logs/<run_name>/`. The analysis package is deliberately separate from the embedded C++ library.

Use `python tools/format.py` to apply formatting. Do not include clang-tidy in the normal local agentic workflow; it is intentionally a CI-only gate because Eigen-heavy translation units make local runs slow. Run `python tools/format.py --tidy` locally only when the user explicitly asks for clang-tidy or when diagnosing a clang-tidy CI failure. CI runs `python tools/format.py --check --tidy --tidy-warnings-as-errors` on the Linux Debug compilation database. Do not apply automatic tidy fixes broadly without reviewing their scope.

NavKit-owned C++ targets use warning/profile settings from `cmake/NavKitWarnings.cmake`. CI enables `NAVKIT_WARNINGS_AS_ERRORS`; local builds may opt in with `python tools/build.py --warnings-as-errors`. Release builds use the embedded-oriented optimization profile in `NavKitWarnings.cmake`; verify Release builds when touching compiler flags, optimization settings, or performance-sensitive code.

VS Code launch configurations assume that a Debug build already exists; they do not build automatically. Build with the wrapper before starting a debugging session.

## Change discipline

- For architectural work, use the sequence: inspect relevant ADRs and code, define a small pass, implement it, add compile-time and runtime tests, build, test, update the changelog, and reconcile documentation.
- A normal development pass is: bootstrap or activate `.venv`, edit, update `CHANGELOG.md` for changes worth tracking, reconcile `README.md` and `docs/SETUP.md` when user-facing behavior/layout/tooling/workflow changes, apply copyright headers, format, run copyright/format checks, build, run tests, run simulation/analysis when behavior is affected, inspect `git status`, and then prepare the change for review.
- Prefer small, reviewable changes. Avoid mixing mechanization work, estimator policy refactors, and unrelated cleanup.
- Keep existing public behavior and matrix sign conventions unless the task explicitly changes them; verify estimation changes numerically.
- Do not present planned types or directories from the ADRs as existing code. Future items include propagation/mechanization policies, coordinates, atmosphere, magnetic, and geoid support.
- If implementation and documentation disagree, call out the mismatch and either update both within task scope or leave a clear note rather than silently choosing the aspirational version.
