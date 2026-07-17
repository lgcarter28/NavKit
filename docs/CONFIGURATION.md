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
readable concrete configs, aggregate config checks, tests, and build-system
selection.

## Current status

The current implementation has the first pieces of the configuration model:

- Shared scalar/time aliases live in `include/navkit/core/config/Types.hpp`.
- Shared core configuration concepts live in
  `include/navkit/core/config/ConfigPolicy.hpp`.
- Estimation-specific config concepts live beside the estimation domains that
  consume them:
  - `include/navkit/core/estimation/sensor/SensorConfigPolicy.hpp`
- User-facing product-configuration concepts live under
  `include/navkit/api/config`. This is the public front door for config authors
  who want to assert that a composed NavKit config exposes the required product
  graph.
- `include/navkit/api/config/ConfigApi.hpp` is the product-config convenience
  include. It collects the shared core graph machinery used by reusable product
  configs. Product headers should still include their specific model, profiler,
  target, or component choices explicitly.
- Concrete repository-provided compile-time configs live under
  `config/compiletime`. Reusable NavKit product configs live under
  `config/compiletime/navkit/products`, while top-level executable composition
  configs live under `config/compiletime/apps`.
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

### 2. Concrete config slices provide values when they name a real boundary

A concrete config slice is a small C++ type that satisfies one local concept.

For example:

```cpp
struct GnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

static_assert(navkit::core::estimation::BufferConfigPolicy<GnssBufferConfig>);
```

Slice-level `static_assert` checks are useful in teaching examples and tests.
Runnable product configs should usually avoid repeating every consumed slice
check; the aliases that consume those slices plus the aggregate product-config
check are the real validation path. Do not introduce a one-field slice just to
carry a value that is only consumed locally inside one product graph; use a
descriptive role constant instead.

Compile-time config headers may provide values, but those values should be
immutable product facts:

- use `static constexpr` for scalar, enum, size, ID, boolean, and other
  literal-like values;
- use `inline static const` for non-literal fixed-size config objects such as
  Eigen matrices;
- do not expose mutable static config objects.

Runtime JSON should populate runtime-owned values, not mutate compile-time
config objects. If a runtime scenario can override a numeric object such as an
initial covariance, app-support should parse a separate runtime value and choose
between that value and the immutable compile-time default at the owning
boundary.

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
struct EcefInsGnssConfig
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using PrimaryGnssMeasurementModel = navkit::core::models::GnssPosModel<StateDef>;
    static constexpr navkit::core::estimation::SensorId primary_gnss_sensor_id = 0U;
    static constexpr std::size_t primary_gnss_buffer_size = 16U;
    using PrimaryGnssDiagnostics = navkit::core::estimation::DefaultSensorDiagnostics;

    using PrimaryGnssSensor =
        navkit::core::estimation::Sensor<primary_gnss_sensor_id,
                                         PrimaryGnssMeasurementModel,
                                         primary_gnss_buffer_size,
                                         navkit::core::estimation::GnssFixedNoisePolicy,
                                         PrimaryGnssDiagnostics>;

    using Sensors = std::tuple<PrimaryGnssSensor>;
    using Profiler = navkit::core::profiling::NullProfiler;
    using Filter = /* concrete filter type */;
    using Propagation = navkit::core::estimation::NoOpPropagation;
    using NavigatorUpdate = /* concrete update policy type */;
    using Navigator = /* concrete navigator type */;
};

static_assert(navkit::api::config::NavKitProductConfigPolicy<EcefInsGnssConfig>);
```

`SensorId` gives each configured sensor a stable product-graph identity even
when multiple sensors share the same model. Product configs expose the explicit
`Sensors` tuple and can name sensor-local diagnostics choices where that helps
teach or customize the graph. The filter derives its own diagnostic storage
from the sensor graph. This keeps the user-facing config focused on the product
graph while still allowing filter-owned measurement statistics to remain keyed
by the configured sensor type.

`Propagation` is the selected propagation/mechanization behavior for the
configured `StateDef`. It owns nominal state propagation and construction of
the discrete covariance inputs (`Phi` and `Qd`) from the selected dynamics, but
it does not own the filter covariance update itself. `KalmanFilter` applies
`P = Phi P Phi^T + Qd`, and `Navigator` orchestrates when the selected
propagation policy and filter are called. Measurement-only products can still
select `NoOpPropagation`. Concrete propagation choices also expose compile-time
buffer/cadence constants such as IMU buffer capacity, medium-rate covariance
update rate, and covariance-step history capacity. These remain compile-time
because they affect fixed storage requirements.

`NavigatorUpdate` is the selected concrete update behavior for the configured
`Filter` and `Sensors`, such as `UpdatePostFilter<Filter>`. The lower-level
`UpdatePolicy` concept describes the per-sensor update interface, while the
tuple-wide `NavigatorUpdatePolicy` concept proves that the selected
`NavigatorUpdate` is valid for the whole Navigator boundary.

### 4. App configs compose a NavKit config with an executable

An application-level config is the usual `NAVKIT_CONFIG` selection for runnable
executables. It consumes a reusable NavKit config and names the app composition
being built:

```cpp
struct EcefInsGnssAppConfig
{
    using NavKit = navkit::config::navkit::EcefInsGnssConfig;

    using PrimaryGnssSensor = typename NavKit::PrimaryGnssSensor;
    using PrimaryGnssEmulator = navkit::app_support::GnssEmulator<PrimaryGnssSensor::Id>;
    using PrimaryGnssBinding =
        navkit::app_support::EmulatorBinding<PrimaryGnssEmulator, PrimaryGnssSensor>;

    using EmulatorBindings = std::tuple<PrimaryGnssBinding>;
    using ImuSimulator = navkit::sim::ImuSimulator;

    using NavInitializationProvider =
        navkit::app_support::PvaExplicitInitializationProvider;
    using TransferAlignmentProvider = navkit::app_support::NoTransferAlignmentProvider;

    using PrimaryGnssStatistics =
        navkit::core::estimation::MeasurementStatistics<PrimaryGnssSensor>;
    using Logger =
        navkit::io::RunLogger<navkit::io::TruthLogProduct,
                              navkit::io::GnssPositionLogProduct,
                              navkit::io::NavEstimateLogProduct,
                              navkit::io::GnssPositionUpdateLogProduct<PrimaryGnssStatistics>>;
    using App = navkit::app_support::SimulationApp<EcefInsGnssAppConfig>;
};
```

This keeps the NavKit library config reusable across apps while still allowing
one build tree to select one concrete executable composition.
App sensor/emulator links use configured emulator types with stable unsigned
`SensorId` values for runtime identity and explicit NavKit sensor aliases for
compile-time wiring. The app does not reconstruct NavKit sensors, maintain a
parallel `Emulators` tuple, or write raw tuple indices. `EmulatorBindings` is
the owned app graph: each binding carries the configured emulator type, the
target sensor type, and an ID derived from `Emulator::Id`; the binding verifies
that this ID matches the selected sensor's configured `Sensor::Id`. This keeps
duplicate sensors of the same model type, such as primary and backup GNSS
receivers, unambiguous. App configs also select the logger adapter type at
compile time; runtime JSON still owns run-specific choices such as run name and
output directory. App configs also select the concrete IMU simulator type used by
the selected simulation loop; runtime JSON configures that simulator's runtime
mode and numeric error parameters. Logger adapters are composed from concrete log products with
`navkit::io::RunLogger<...>`; the selected app config owns the product set.
Startup navigation data is also an app boundary: `NavInitializationProvider`
turns runtime input, simulated truth, saved state, or future embedded inputs
into a typed PVA `NavInitialization` message. `TransferAlignmentProvider`
describes optional timestamped aiding after construction and startup
initialization. Disabled transfer alignment should be explicit, usually through
`NoTransferAlignmentProvider`, so a runtime `transfer_alignment` section is not
silently ignored.
The built-in PVA initialization providers currently support deterministic
`"type": "pva_error"` inputs and seeded random `"type": "pva_random"` draws
from a configured `pva_cov`; selecting which provider is valid is a compile-time
app-config decision. For deterministic PVA errors, vector keys encode the frame
and units. ECEF-resolved errors use `p_e_m`, `v_e_mps`, and `rotvec_b2e_rad`.
Local-level errors use `p_n_m`, `v_n_mps`, and `rotvec_b2n_rad`; app support
converts those relative NED vectors to the internal ECEF-resolved
initialization convention using the initial truth/reference position. The
`pva_cov` `diag` or `full` matrix follows the same field ordering and frame
conventions implied by the selected `pva_error` keys, then is rotated into the
filter's ECEF-resolved small-angle attitude-error covariance before
initialization. Random PVA initialization uses the same covariance ordering but
has no deterministic `pva_error` keys, so `"type": "pva_random"` may provide
`"pva_error_frame": "ecef"` or `"pva_error_frame": "ned"` to state how the
configured covariance should be interpreted before sampling.

The app and NavKit config trees are deliberately separate:

```text
config/compiletime/navkit/products/EcefInsGnss.hpp
config/compiletime/apps/navkit_sim/EcefInsGnss.hpp
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
CMake generates `generated/navkit/SelectedConfig.hpp` inside the selected build
tree, which exposes the stable application-facing alias:

```cpp
using AppConfig = navkit::selected_config::Config;
```

App-support code validates, translates, and orchestrates selected configuration;
it should not author product or scenario defaults. Scenario-specific values such
as run name, output directory, logging cadence, simulator seeds, simulator noise
levels, trajectory duration, and trajectory cadence belong in runtime JSON.
Compile-time product choices such as selected state definitions, fixed buffer
sizes, covariance cadence, and initial-covariance policy slices belong in the
selected compile-time config. Reusable embedded-library defaults may live inside
`include/navkit/core` only when they are deliberately part of the product-core
contract. Do not hide fallback scenario behavior in app-support helpers.

## CMake selection model

The CMake model is one compile-time configuration per build tree:

```text
cmake -S . -B build/debug/apps/navkit_sim/EcefInsGnss -G Ninja -DNAVKIT_CONFIG=apps/navkit_sim/EcefInsGnss.hpp
```

`NAVKIT_CONFIG` is a CMake cache variable relative to `config/compiletime`. It
has a useful default:

```text
apps/navkit_sim/EcefInsGnss.hpp
```

Debug/Release and `NAVKIT_CONFIG` are separate axes:

- `CMAKE_BUILD_TYPE` or `--config` selects compiler/build mode.
- `NAVKIT_CONFIG` selects the top-level compile-time build configuration,
  usually an app config for executables.

Multiple configurations use multiple build directories or CMake presets, not a
single executable that dynamically switches among compile-time configs. By
default, repository Python tools derive the build directory from the selected
config header:

```text
apps/navkit_sim/EcefInsGnss.hpp
    -> build/debug/apps/navkit_sim/EcefInsGnss

apps/navkit_sim/ProfiledEcefInsGnss.hpp
    -> build/debug/apps/navkit_sim/ProfiledEcefInsGnss
```

Runtime JSON is still checked against the compiled app composition. For example,
the stationary GNSS sim config currently requires `trajectory`, `imu`, `gnss`,
and `initialization` sections, rejects unsupported `baro` and disabled
`transfer_alignment` sections, and validates common numeric/vector fields before
the simulation loop starts. The current initialization provider uses
`"type": "pva_error"` with nested `pva_error` and `pva_cov` sections so runtime
inputs describe startup PVA error and covariance explicitly while still passing
only typed PVA navigation data into the configured Navigator. `pva_error` keys
name the authored convention, for example `p_n_m`, `v_n_mps`, and
`rotvec_b2n_rad` for intuitive local-level position, velocity, and
roll/pitch/yaw-like small-angle attitude errors. The covariance section uses the
same implied order and frame before app support converts it into the internal
ECEF-resolved state covariance. This validation lives in `navkit::app_support`
because it is app/runtime-input glue, not reusable product-core NavKit
configuration.

Filter initial covariance is a separate product/runtime boundary. The compiled
NavKit product config provides:

```cpp
using InitialCovariance_t =
    navkit::core::estimation::InitialCovariance<StateDef>;

inline static const InitialCovariance_t initial_covariance =
    navkit::core::estimation::diagonal_initial_covariance<StateDef>(
        navkit::core::estimation::InitialCovarianceDiagonal<StateDef>{
            .values = {
                // values in the selected error-state order
            },
        });
```

The `values` entries are variances, not sigmas, ordered exactly like the
selected `StateDef::Error`. Runtime scenarios may override the filter initial
covariance by adding `initialization.initial_covariance`:

```json
{
  "initialization": {
    "initial_covariance": {
      "diag": [ /* StateDef::Error::N variance values */ ]
    }
  }
}
```

or:

```json
{
  "initialization": {
    "initial_covariance": {
      "full": [ /* row-major NxN covariance values */ ]
    }
  }
}
```

If `initial_covariance` is absent, app-support uses the immutable compile-time
`NavKit::initial_covariance`.

The Python build wrapper forwards the same selection:

```text
python tools/build.py --build-type Debug --skip-conan --navkit-config apps/navkit_sim/EcefInsGnss.hpp
```

The default build directory is already config-rooted. Use `--build-dir` only
when an explicit custom location is needed:

```text
python tools/build.py --build-type Debug --build-dir build/custom/stationary --navkit-config apps/navkit_sim/EcefInsGnss.hpp
```

Each build directory has its own generated `navkit/SelectedConfig.hpp`, so two
different selected configs do not fight over the same generated header.

CMake presets can capture common build-type/config pairings. Repository presets
such as `debug-ecef-ins-gnss` and `release-ecef-ins-gnss` are examples of
that pattern; they do not make Debug/Release part of the config itself.

## Adding a new compile-time config

The expected workflow is:

1. Copy the nearest example, such as
   `config/compiletime/navkit/products/MinimalConfig.hpp`.
2. Rename the config type.
3. Adjust or add the NavKit config slices your library/application needs.
4. For runnable/product NavKit configs, add the aggregate
   `NavKitProductConfigPolicy` check. Keep detailed slice-level `static_assert`
   checks in teaching examples or focused tests unless they materially improve
   diagnostics for a real consumed alias.
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
