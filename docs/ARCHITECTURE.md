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
| Applications | app targets under `apps/` | `apps/` | Executable entry points that compose the libraries they need |
| Analysis | Python package under `python/` | `python/navkit_analysis` | Offline plots and validation analysis |

The root `CMakeLists.txt` is intentionally an orchestration layer. Header-only
and interface target definitions live under `cmake/targets/`, while compiled
targets live next to the source files they build:

```text
cmake/targets/NavKitCore.cmake   navkit_core / navkit::core
cmake/targets/NavKitIo.cmake     navkit_io / navkit::io
src/sim/CMakeLists.txt           navkit_sim / navkit::sim
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
        update/
        propagation/
    environment/
      planet/
      gravity/
      atmosphere/
      magnetic/
      geoid/
      terrain/
    frames/
    units/
    models/

  sim/
  io/

config/
  compiletime/
    examples/
    navkit_sim/
  runtime/
    navkit_sim/

src/
  sim/
```

`include/navkit/core` is the reusable product core, not a miscellaneous bucket.
Simulation, desktop IO, concrete app/product compile-time configurations, and
executable runtime input bundles are intentionally outside the core boundary.

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
| `navkit::core` | `navkit::core::units` | Unit and frame helper types |
| `navkit::sim` | `navkit::sim` | Simulation support |
| `navkit::io` | `navkit::io` | Logging, CSV, JSON, and run manifests |

Shared compile-time product-core configuration vocabulary lives under
`navkit::core::config`. Domain-specific configuration concepts live beside the
domain that consumes them; for example, estimator sensor-buffer and
measurement-statistics configuration concepts currently live under the
`navkit::core::estimation` namespace.

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

Do not add dummy `.cpp` files merely to force a static archive. Convert an
`INTERFACE` target to a compiled/static library when the component owns
meaningful `.cpp` implementation.

Target-definition placement follows the same rule. Header-only/interface target
definitions belong under `cmake/targets/` because they do not own local source
files. Compiled targets belong beside their implementation sources, such as
`src/sim/CMakeLists.txt`.

## Current data flow

The working demonstration is GNSS-position-only:

```text
stationary truth
    -> GNSS simulator
    -> Sensor queue
    -> Navigator
    -> KalmanFilter measurement update
    -> measurement statistics
    -> CSV/JSON logs
    -> Python analysis
```

The target flow will add propagation/mechanization, multi-rate sensors, richer
truth generation, and repeatable validation metrics. Those are roadmap items,
not current behavior.

## Current implementation boundaries

- `KalmanFilter` performs measurement updates, stores optional per-model
  statistics, and delegates injection/reset.
- `Navigator` processes a tuple-like sensor collection and applies an update
  policy.
- Planet and gravity policies are the most complete examples of the intended
  concept -> optional CRTP base -> concrete policy layering.
- Simulation currently contains desktop-oriented support and may use runtime
  polymorphism where practical.
- Python analysis is deliberately outside the embedded-facing C++ product core.

## Explicitly not implemented yet

- INS propagation/mechanization.
- A propagation policy and `NoOpPropagation` seam in Navigator.
- Correct stationary Earth-rotation IMU truth.
- General coordinate conversions and local-vertical altitude modeling.
- Barometer and IMU simulator behavior beyond current shells/placeholders.
- Embedded target profiles, resource budgets, and hardware abstraction layers.

Use `docs/ROADMAP.md` for sequencing and status, and update this document when
implemented architecture changes.
