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
- Concrete repository-provided compile-time configs live under
  `config/compiletime`.
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

### 3. Composed configs collect slices for an app or product

A composed compile-time config names the slices a particular application,
simulation, product target, or test fixture uses.

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

### 4. Consumers validate only the slices they need

NavKit should avoid a universal config concept that requires every possible
sensor, filter, logger, simulator, and target option. A GNSS-only application
should not need to provide IMU, barometer, magnetometer, or embedded board
settings merely to satisfy a central type.

Instead, each app, algorithm, or target should validate the slices it actually
uses.

### 5. Build selection is separate from runtime inputs

Compile-time config answers:

```text
What product/app/target are we compiling?
```

Runtime input answers:

```text
What scenario are we running today?
```

Those are deliberately separate. The current layout is:

```text
config/
  compiletime/
    examples/
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
cmake -S . -B build/Debug -DNAVKIT_CONFIG=navkit_sim/StationaryGnss.hpp
```

`NAVKIT_CONFIG` is a CMake cache variable relative to `config/compiletime`. It
has a useful default:

```text
navkit_sim/StationaryGnss.hpp
```

Debug/Release and `NAVKIT_CONFIG` are separate axes:

- `CMAKE_BUILD_TYPE` or `--config` selects compiler/build mode.
- `NAVKIT_CONFIG` selects the product/app compile-time configuration.

Multiple configurations should use multiple build directories or CMake presets,
not a single executable that dynamically switches among compile-time configs.

The Python build wrapper forwards the same selection:

```text
python tools/build.py --build-type Debug --skip-conan --navkit-config navkit_sim/StationaryGnss.hpp
```

Use `--build-dir` when keeping more than one selected config locally:

```text
python tools/build.py --build-type Debug --build-dir build/debug-stationary-gnss --navkit-config navkit_sim/StationaryGnss.hpp
```

Each build directory has its own generated `navkit/SelectedConfig.hpp`, so two
different selected configs do not fight over the same generated header.

CMake presets can capture common build-type/config pairings. Repository presets
such as `debug-stationary-gnss` and `release-stationary-gnss` are examples of
that pattern; they do not make Debug/Release part of the config itself.

## Adding a new compile-time config

The expected workflow is:

1. Copy the nearest example, such as `config/compiletime/examples/MinimalConfig.hpp`.
2. Rename the config type.
3. Adjust or add the config slices your application needs.
4. Add `static_assert` checks for each concept slice the config claims to satisfy.
5. Expose the selected type as `navkit::config::SelectedConfig`.
6. Select it with CMake or the build wrapper using `NAVKIT_CONFIG`.
7. Pair it with a runtime input file only if the application needs one.

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
