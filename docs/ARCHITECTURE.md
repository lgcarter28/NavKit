# NavKit Architecture

This document describes the architecture that exists in the repository today.
The ADRs under `docs/adr/` describe proposed direction and design rationale, but
they do not override the checked-in implementation.

## Product boundaries

NavKit is split by product boundary first, then by engineering domain.

| Boundary | CMake target | Header root | Role |
|---|---|---|---|
| Product core | `navkit::core` | `include/navkit/core` | Reusable estimation/navigation framework and domain models |
| Simulation support | `navkit::sim` | `include/navkit/sim` | Desktop simulation infrastructure and generated measurements |
| IO support | `navkit::io` | `include/navkit/io` | Desktop logging, files, CSV, JSON, and run manifests |
| Application support | `navkit::app_support` | `include/navkit/app_support` | Header-only executable support helpers for selected-config app composition, runtime JSON input/validation, emulator binding, trajectory, logging, initialization, and profile export |
| Applications | app targets under `apps/` | `apps/` | Executable entry points that compose the libraries they need |
| Analysis | Python package under `python/` | `python/navkit_analysis` | Offline plots and validation analysis |

The root `CMakeLists.txt` is intentionally an orchestration layer. Header-only
and interface target definitions live under `cmake/targets/`, while compiled
targets live next to the source files they build:

```text
cmake/targets/NavKitCore.cmake   navkit_core / navkit::core
cmake/targets/NavKitIo.cmake     navkit_io / navkit::io
src/sim/CMakeLists.txt           navkit_sim / navkit::sim
src/app_support/CMakeLists.txt   navkit_app_support / navkit::app_support
```

## Header and source layout

```text
include/navkit/
  core/
    config/
    containers/
    estimation/
      state/
      measurement/
      filter/
        injection/
        reset/
      sensor/
        noise/
      navigator/
        PVA/TXA state vocabulary
        update/
        propagation/
    environment/
      planet/
      gravity/
    frames/
    math/
    units/
    models/
    profiling/

  sim/
  io/
    log_payloads/
    log_products/
  app_support/
    config/
    emulation/
      concrete/
    runtime/
    initialization/
    logging/
    profiling/
    trajectory/

config/
  compiletime/
    navkit/
    apps/
      navkit_sim/
  runtime/
    navkit_sim/

src/
  sim/
  app_support/
```

`include/navkit/core` is the reusable product core, not a miscellaneous bucket.
Simulation, desktop IO, concrete app/product compile-time configurations, and
executable runtime input bundles are intentionally outside the core boundary.
Future domains such as atmosphere, magnetic-field, geoid, and terrain policies
should be added when concrete implementations land; the repository should avoid
empty placeholder directories for planned architecture.

## Namespaces

Public namespaces mirror the folder structure through the stable domain level.
Deeper leaf folders may organize implementation and policy families without
adding additional namespaces unless that subdomain becomes independently
meaningful. For example, `include/navkit/core/environment/gravity/J2.hpp`
currently contributes `navkit::core::environment::J2`, not
`navkit::core::environment::gravity::J2`.

| CMake target | Primary namespace | Notes |
|---|---|---|
| `navkit::core` | `navkit::core` | Common product-core foundational aliases such as `Scalar_t` and `Time_t` |
| `navkit::core` | `navkit::core::config` | Shared product-core compile-time configuration vocabulary |
| `navkit::core` | `navkit::core::containers` | Product-core containers |
| `navkit::core` | `navkit::core::estimation` | State definitions, measurements, filters, sensors, navigators, and estimator policies |
| `navkit::core` | `navkit::core::environment` | Planet and gravity policies |
| `navkit::core` | `navkit::core::frames` | Frame tags and frame-typed helpers |
| `navkit::core` | `navkit::core::models` | Reusable product-core measurement and process models |
| `navkit::core` | `navkit::core::profiling` | Zero-overhead-by-default profiling vocabulary, policies, and fixed records |
| `navkit::core` | `navkit::core::units` | Unit and frame helper types |
| `navkit::sim` | `navkit::sim` | Simulation support |
| `navkit::io` | `navkit::io` | Logging, CSV, JSON, and run manifests |
| `navkit::app_support` | `navkit::app_support` | Shared executable support templates that are not product-core API |

Shared compile-time product-core configuration vocabulary lives under
`navkit::core::config`. Domain-specific configuration concepts live beside the
domain that consumes them; for example, estimator sensor-buffer and
measurement-statistics configuration concepts currently live under the
`navkit::core::estimation` namespace.

The public config API include boundary is intentionally narrow.
`include/navkit/api/config/ConfigApi.hpp` collects shared product-graph
vocabulary, required contracts, and defaults exposed directly by primary core
template boundaries. Concrete product configs include `ConfigApi.hpp` plus the
specific model, profiler, target, or component headers they select. It should
not become a universal include for every concrete component choice.

See [`CONFIGURATION.md`](CONFIGURATION.md) for the user-facing configuration
mental model, example config contracts, and the selected-config build workflow.

Runtime scenario inputs for executables live outside public headers, such as
`config/runtime/navkit_sim`. This avoids mixing "what product are we compiling?"
with "what scenario are we running today?" It also avoids overly ceremonial names
such as `navkit::core::environment::planet::Wgs84` until a leaf domain becomes
independently meaningful.

## Target kinds

Use CMake target kinds honestly:

- `navkit::core` is currently an `INTERFACE` target because the core is
  header-only and template-heavy. It carries include directories, C++23 compile
  features, and the Eigen usage requirement.
- `navkit::io` is currently an `INTERFACE` target because IO support is still
  header-only. It carries the desktop JSON dependency.
- `navkit::sim` is a compiled library because simulator implementation sources
  live in `src/sim/*.cpp`.
- `navkit::app_support` is currently an `INTERFACE` target because its reusable
  support is template-heavy and selected-config dependent. It provides C++
  helpers for JSON runtime inputs, compiled configuration description, selected
  simulation app composition, repeated estimator aliases, and profile export.
  Concrete application entry points remain thin selected-config dispatchers.

Do not add dummy `.cpp` files merely to force a static archive. Convert an
`INTERFACE` target to a compiled/static library when the component owns
meaningful `.cpp` implementation.

Target-definition placement follows the same rule. Header-only/interface target
definitions belong under `cmake/targets/` because they do not own local source
files. Compiled targets belong beside their implementation sources, such as
`src/sim/CMakeLists.txt`. The app-support target definition currently lives
under `src/app_support/CMakeLists.txt` to keep that boundary visible; it can
move under `cmake/targets/` if it remains purely header-only.

## Manifest ownership

Python tooling currently orchestrates build commands and writes the outer
`navkit_build_manifest.json` because it knows wrapper-level facts such as build
type, selected build directory, elapsed build time, and resource reports. The
selected-config portion of that manifest comes from C++: after building,
`tools/build.py` asks the executable to describe its compiled config. Runtime
application manifests and log metadata are written by C++ application/IO code.
Application entry points should stay selected-config generic where practical.
The selected app config composes a reusable NavKit library config with app-side
sensor bindings, emulator policies, navigation initialization providers, and
optional transfer-alignment providers. `navkit::app_support` owns reusable
selected-app runner and simulation-loop plumbing at the top level, with focused
subdirectories for app compile-time config vocabulary, generic emulator binding
and runtime dispatch, concrete emulators, JSON/runtime validation, PVA startup
initialization and transfer-alignment seams, app-side logging adapters, profile
export, and trajectory providers. IO log products and typed payload wrappers live under
`include/navkit/io/log_products` and `include/navkit/io/log_payloads` so the
serialization boundary is explicit without forcing logging into product-core
code. `navkit::io::RunLogger<...>` is a small compile-time façade over the
selected app-configured log-product tuple: it owns output-directory setup, product
open/flush/close orchestration, per-product metadata files, and the run
manifest, while concrete products own CSV schemas and payload serialization.

This boundary avoids checked-in sidecar metadata that can drift from the actual
compiled configuration. If build-manifest writing moves fully into C++ later,
preserve that rule: compiled C++ should be the source of truth for compiled
configuration facts.

## Current data flow

The working demonstration is stationary ECEF INS propagation with GNSS-position
aiding:

```text
stationary truth
    -> IMU simulator
    -> Navigator IMU buffer
    -> ECEF INS propagation policy
    -> GNSS simulator
    -> Sensor queue
    -> Navigator
    -> KalmanFilter measurement update
    -> measurement statistics
    -> CSV/JSON logs
    -> Python analysis
```

The target flow will add richer aiding, multi-rate sensors, richer truth
generation, history/replay support, and repeatable validation metrics. Those are
roadmap items, not current behavior.

## Current implementation boundaries

- `KalmanFilter` performs measurement updates, propagates covariance from
  discrete `Phi`/`Qd` inputs, stores optional per-model statistics, and
  delegates injection/reset.
- `Navigator` exposes `update()` as the normal orchestration call and explicit
  stage methods for `process_strapdown_integration()`, `propagate_covariance()`,
  and `process_measurements()`. It owns a fixed-capacity FIFO of timestamped IMU
  increments fed through `push_imu(...)` and composes one pending discrete
  covariance step while draining that buffer. The pending covariance step is
  applied at the selected compile-time medium-rate cadence, and recent applied
  covariance steps are retained in a bounded history buffer for inspection and
  future delayed-measurement support. The selected propagation policy owns state
  propagation plus `F_k`/`G_k`/`Phi_k`/`Q_d` construction; the filter owns
  applying the covariance propagation; and the update policy owns
  measurement update behavior. `NoOpPropagation` remains available for
  measurement-only products, while the selected stationary GNSS products now use
  the first ECEF INS propagation policy.
- `EcefInsPropagation` implements the first single-IMU ECEF propagation path. It
  propagates nominal body-to-ECEF attitude as a unit quaternion, keeps the
  covariance attitude state as a 3D small-angle `Att` perturbation, builds
  first-order `F_k`/`G_k`/`Phi_k`/`Q_d` products, and applies the v1
  PVA+gyro-bias+accelerometer-bias dynamics to the selected
  `DefaultInsStateDef` aggregate. Its nominal layout stores attitude in
  `AttQuat`, while its error/covariance layout keeps attitude as a 3D
  `AttRotVec` perturbation.
- Planet and gravity policies are the most complete examples of the intended
  concept -> optional CRTP base -> concrete policy layering.
- Product-core profiling now provides the embedded-facing vocabulary for future
  instrumentation: enum profile points, fixed timing records, optional
  visualization metadata fields, clock/sink/profiler concepts, a null default
  profiler, and a scoped profiler for deterministic clock/sink policies.
  `KalmanFilter::observation_update` and `Navigator::process_measurements` are
  the first coarse algorithm integration points and both default to
  `NullProfiler`. Sequencing/nesting semantics are not owned by the generic
  record type and remain future profiler/sink policy work.
- `sim::ImuSimulator` generates raw timestamped IMU increments from consecutive
  ECEF truth samples. It derives ideal body-frame inertial angular increments
  and specific-force increments, then applies deterministic gyro/accelerometer
  error-model parameters such as bias, bias random walk, white noise, scale
  factor, misalignment, non-orthogonality, and quantization. The selected
  simulation app validates IMU runtime config, generates these increments, and
  feeds them into the Navigator before GNSS measurement processing.
- Simulation currently contains desktop-oriented support and may use runtime
  polymorphism where practical.
- Python analysis is deliberately outside the embedded-facing C++ product core.

## Policy concepts and template boundaries

Policy concepts are intended to name stable capability boundaries, not to copy
every member function of the first implementation that happens to satisfy them.
Use a concept when multiple consumers rely on the same interface, when a
configuration-selected type is intentionally swappable, or when diagnostics at a
public template boundary would otherwise be poor.

Context-dependent concepts stay candidate-first and carry only the context they
actually require. For example, a measurement model is validated relative to a
state definition at the filter or product-configuration boundary; the generic
sensor container does not need to know that state definition merely to store and
queue measurements.

When two consumers need different parts of a type's capability, prefer separate
narrow concepts over one over-coupled concept. A future filter cleanup may
distinguish the standalone filter lifecycle contract from the sensor-specific
"this filter can process this sensor" contract so Navigator construction,
initialization, and tuple compatibility checks can each name the dependency they
actually own.

Raw `typename` remains appropriate for private tuple expansion helpers, local
implementation details, and deliberately unconstrained utilities. It is a design
smell on primary policy or product-configuration surfaces when a clear concept
already exists and improves readability or diagnostics.

## Explicitly not implemented yet

- GNSS velocity aiding and configured non-IMU sensor lever-arm observations in
  the selected simulation app.
- IMU history/replay, delayed-measurement handling, and latency compensation.
- General coordinate conversions and local-vertical altitude modeling.
- Barometer simulator behavior beyond current shells/placeholders.
- Embedded target profiles, resource budgets, and hardware abstraction layers.

Use `docs/ROADMAP.md` for sequencing and status, and update this document when
implemented architecture changes.
