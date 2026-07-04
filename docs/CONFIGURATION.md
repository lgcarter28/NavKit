# NavKit Configuration

This document explains how NavKit's compile-time configuration model is meant
to fit together.

The short version:

```text
domain config concept -> concrete config slice -> composed compile-time config
    -> selected build config -> application/runtime algorithm
```

This replaces the "obvious spine" that a virtual base-class hierarchy would
normally provide. The spine is instead made explicit through domain concepts,
example concrete configs, local `static_assert` checks, tests, and build-system
selection.

## Current status

The current implementation has the first pieces of the configuration model:

- Shared scalar/time aliases live in `include/navkit/core/config/Types.hpp`.
- Shared core configuration concepts live in
  `include/navkit/core/config/ConfigPolicy.hpp`.
- Estimation-specific config concepts live beside the estimation domains that
  consume them:
  - `include/navkit/core/estimation/sensor/SensorConfigPolicy.hpp`
  - `include/navkit/core/estimation/filter/FilterConfigPolicy.hpp`
- User-facing product-configuration concepts live under
  `include/navkit/api/config`. This is the public front door for config authors
  who want to assert that a composed NavKit config exposes the required product
  graph.
- Concrete repository-provided compile-time configs live under
  `config/compiletime`. NavKit library/core configs live under
  `config/compiletime/navkit`, while top-level executable composition configs
  live under `config/compiletime/apps`.
- Runtime inputs live under `config/runtime`.
- CMake selects one compile-time configuration per build tree with
  `NAVKIT_CONFIG`.
- Applications include the generated `navkit/SelectedConfig.hpp` header instead
  of directly including a concrete config header.

## Mental model

### 1. Domain config concepts define local requirements

A config concept belongs next to the code that consumes it. The generic pattern
is:

```text
include/navkit/<product-or-domain>/.../*ConfigPolicy.hpp
```

For example, sensor buffer capacity is an estimation sensor concern, so the
concept lives with the sensor domain rather than in a central god-config header.

This keeps each concept small and honest: it says what one domain needs, not
what every possible NavKit application must provide.

### 2. Concrete config slices provide values

A concrete config slice is a small C++ type that satisfies one local concept.

For example:

```cpp
struct GnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

static_assert(navkit::core::estimation::BufferConfigPolicy<GnssBufferConfig>);
```

The `static_assert` is intentional documentation. It tells users and the
compiler which contract the slice claims to satisfy.

### 3. NavKit configs collect reusable library slices

A NavKit library/core config names reusable slices for the navigation library
itself. It should not know which executable will consume it.

For example:

```cpp
struct MinimalConfig
{
    using Numeric = NumericConfig;
    using GnssBuffer = GnssBufferConfig;
};

static_assert(navkit::core::config::ConfigPolicy<MinimalConfig>);
static_assert(navkit::core::estimation::BufferConfigPolicy<MinimalConfig::GnssBuffer>);
```

`MinimalConfig` is intended as a teaching/example shape, not a universal
production target and not a base class.

Runnable/product NavKit configs also expose the product graph that downstream
apps consume:

```cpp
struct StationaryGnssConfig
{
    using StateDef = navkit::core::estimation::InsStateDef;
    using PrimaryGnssModel = navkit::core::models::GnssPosModel<StateDef>;
    static constexpr navkit::core::estimation::SensorId PrimaryGnssSensorId = 0U;

    using PrimaryGnssSensor =
        navkit::core::estimation::Sensor<PrimaryGnssSensorId,
                                         PrimaryGnssModel,
                                         GnssBuffer::BufferSize>;

    using Sensors = std::tuple<PrimaryGnssSensor>;
    using MeasurementStatisticsTuple =
        std::tuple<navkit::core::estimation::MeasurementStatistics<PrimaryGnssSensor>>;
    using Profiler = navkit::core::profiling::NullProfiler;
    using Filter = /* concrete filter type */;
    using Navigator = /* concrete navigator type */;
};

static_assert(navkit::api::config::NavKitProductConfigPolicy<StationaryGnssConfig>);
```

`SensorId` gives each configured sensor a stable product-graph identity even
when multiple sensors share the same model. `MeasurementStatisticsTuple` is
manually authored on purpose: the filter owns diagnostic storage, and the config
should make each stored diagnostic stream obvious. A statistic entry is keyed by
the configured sensor type, such as `MeasurementStatistics<PrimaryGnssSensor>`,
not by the sensor model alone. This keeps primary and backup sensors
unambiguous even when they use the same measurement model.

### 4. App configs compose a NavKit config with an executable

An application-level config is the usual `NAVKIT_CONFIG` selection for runnable
executables. It consumes a reusable NavKit config and names the app composition
being built:

```cpp
struct StationaryGnssAppConfig
{
    using NavKit = navkit::config::navkit::StationaryGnssConfig;

    static constexpr navkit::app_support::SensorId PrimaryGnssSensorId =
        NavKit::PrimaryGnssSensorId;
    using EmulatorBindings = std::tuple<
        navkit::app_support::EmulatorBinding<
            PrimaryGnssSensorId,
            navkit::app_support::GnssEmulator,
            NavKit::PrimaryGnssSensor>>;

    using App = navkit::app_support::SimulationApp<StationaryGnssAppConfig>;
};
```

This keeps the NavKit library config reusable across apps while still allowing
one build tree to select one concrete executable composition.
App sensor/emulator links use stable unsigned `SensorId` constants for runtime
identity and explicit NavKit sensor aliases for compile-time wiring. The app
does not reconstruct NavKit sensors or write raw tuple indices. The binding ID
must match the selected sensor's configured `Sensor::Id`, which keeps duplicate
sensors of the same model type, such as primary and backup GNSS receivers,
unambiguous.

The app and NavKit config trees are deliberately separate:

```text
config/compiletime/navkit/StationaryGnss.hpp
config/compiletime/apps/navkit_sim/StationaryGnss.hpp
```

Using the same descriptive file name in both places is fine because the
directories communicate ownership. The `navkit/` header is reusable library
configuration. The `apps/navkit_sim/` header is the selected executable
composition that links a library config to an app runner.

### 5. Consumers validate only the slices they need

NavKit should avoid a universal config concept that requires every possible
sensor, filter, logger, simulator, and target option. A GNSS-only application
should not need to provide IMU, barometer, magnetometer, or embedded board
settings merely to satisfy a central type.

Instead, each app, algorithm, or target should validate the slices it actually
uses.

### 6. Build selection is separate from runtime inputs

Compile-time config answers:

```text
What top-level app, target, or NavKit library configuration are we compiling?
```

Runtime input answers:

```text
What scenario are we running today?
```

Those are deliberately separate. The current layout is:

```text
config/
  compiletime/
    navkit/
    apps/
      navkit_sim/
    targets/          # planned
  runtime/
    navkit_sim/
```

The selected concrete config header provides `navkit::config::SelectedConfig`.
CMake generates `build/<type>/generated/navkit/SelectedConfig.hpp`, which exposes
the stable application-facing alias:

```cpp
using AppConfig = navkit::selected_config::Config;
```

## CMake selection model

The CMake model is one compile-time configuration per build tree:

```text
cmake -S . -B build/Debug -DNAVKIT_CONFIG=apps/navkit_sim/StationaryGnss.hpp
```

`NAVKIT_CONFIG` is a CMake cache variable relative to `config/compiletime`. It
has a useful default:

```text
apps/navkit_sim/StationaryGnss.hpp
```

Debug/Release and `NAVKIT_CONFIG` are separate axes:

- `CMAKE_BUILD_TYPE` or `--config` selects compiler/build mode.
- `NAVKIT_CONFIG` selects the top-level compile-time build configuration,
  usually an app config for executables.

Multiple configurations should use multiple build directories or CMake presets,
not a single executable that dynamically switches among compile-time configs.

Runtime JSON is still checked against the compiled app composition. For example,
the stationary GNSS sim config currently requires `trajectory` and `gnss`
sections, rejects unsupported `imu` and `baro` sections, and validates common
numeric/vector fields before the simulation loop starts. This validation lives
in `navkit::app_support` because it is app/runtime-input glue, not reusable
product-core NavKit configuration.

The Python build wrapper forwards the same selection:

```text
python tools/build.py --build-type Debug --skip-conan --navkit-config apps/navkit_sim/StationaryGnss.hpp
```

Use `--build-dir` when keeping more than one selected config locally:

```text
python tools/build.py --build-type Debug --build-dir build/debug-stationary-gnss --navkit-config apps/navkit_sim/StationaryGnss.hpp
```

Each build directory has its own generated `navkit/SelectedConfig.hpp`, so two
different selected configs do not fight over the same generated header.

CMake presets can capture common build-type/config pairings. Repository presets
such as `debug-stationary-gnss` and `release-stationary-gnss` are examples of
that pattern; they do not make Debug/Release part of the config itself.

## Adding a new compile-time config

The expected workflow is:

1. Copy the nearest example, such as `config/compiletime/navkit/MinimalConfig.hpp`.
2. Rename the config type.
3. Adjust or add the NavKit config slices your library/application needs.
4. Add `static_assert` checks for each concept slice the config claims to satisfy.
5. For executables, add or update an app config under `config/compiletime/apps`
   that names `using NavKit = ...` and `using App = ...`.
6. Expose the selected app or library type as `navkit::config::SelectedConfig`.
7. Add or update app-support runtime validation when the executable consumes
   JSON or other runtime inputs whose shape depends on the compiled app/NavKit
   capabilities.
8. Select it with CMake or the build wrapper using `NAVKIT_CONFIG`.
9. Pair it with a runtime input file only if the application needs one.

If the new config should be easy to discover, add a preset or documented wrapper
example that selects it in a separate build directory.

## What not to do

- Do not put app/product concrete configs under `include/navkit`.
- Do not create a universal god config just because one application needs a
  setting.
- Do not encode runtime scenario choices in compile-time config.
- Do not couple `NAVKIT_CONFIG` to Debug/Release build type.
- Do not add a base class solely to make the configuration tree look familiar.

## Tests as executable documentation

For rigorous positive and negative examples, see
`tests/test_config_policy.cpp`.

Negative cases should be expressed as compile-time assertions such as:

```cpp
static_assert(!SomeConfigPolicy<BadConfig>);
```

This documents invalid designs without adding intentionally uncompilable test
targets.
