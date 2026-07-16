# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `navkit/products/` contains reusable NavKit product graph configurations,
  including the deliberately small `MinimalConfig.hpp` teaching config.
- `apps/` contains top-level executable composition configurations.
- `targets/` may be added later for desktop and embedded product targets.

The `navkit/` and `apps/` trees are intentionally separate. It is normal for
both trees to contain the same descriptive file name, such as
`EcefInsGnss.hpp`: the NavKit product file owns reusable library
configuration, while the app file owns the executable composition that links a
NavKit config to an app runner.

Provided `NAVKIT_CONFIG` selections:

- `apps/navkit_sim/EcefInsGnss.hpp`: default stationary GNSS demo app using
  the unprofiled NavKit GNSS library config.
- `apps/navkit_sim/ProfiledEcefInsGnss.hpp`: same app shape, but consumes a
  profiled NavKit GNSS library config with a host microsecond clock,
  fixed-capacity profiling ring-buffer sink, and scoped profiler so the app
  emits `profile.csv`.

Runnable/product NavKit configs should read like product graphs and usually end
with a single aggregate check, such as
`static_assert(navkit::api::config::NavKitProductConfigPolicy<Config>);`.
Detailed slice-level concept assertions belong in deliberately educational
examples, such as `navkit/products/MinimalConfig.hpp`, and in focused tests.
Product configs should include `navkit/api/config/ConfigApi.hpp` for shared
core graph machinery, then include only the concrete model, profiler, target, or
component headers they select.

Headers intended for `NAVKIT_CONFIG` selection must also expose:

```cpp
namespace navkit::config
{
using SelectedConfig = ...;
}
```

Application-level configs should compose a reusable NavKit library config:

```cpp
struct ExampleAppConfig
{
    using NavKit = navkit::config::navkit::SomeNavKitConfig;

    using PrimarySensor = typename NavKit::PrimarySensor;
    using PrimaryEmulator = navkit::app_support::SomeEmulator<PrimarySensor::Id>;
    using PrimaryBinding =
        navkit::app_support::EmulatorBinding<PrimaryEmulator, PrimarySensor>;

    using EmulatorBindings = std::tuple<PrimaryBinding>;

    using PrimaryStatistics = navkit::core::estimation::MeasurementStatistics<PrimarySensor>;
    using Logger =
        navkit::io::RunLogger<navkit::io::TruthLogProduct,
                              navkit::io::GnssPositionLogProduct,
                              navkit::io::NavEstimateLogProduct,
                              navkit::io::GnssPositionUpdateLogProduct<PrimaryStatistics>>;
    using App = navkit::app_support::SimulationApp<ExampleAppConfig>;
};
```

`navkit::io::RunLogger<...>` is a generic compile-time logger façade. App
configs select their concrete log-product set explicitly.

Applications should include the generated `navkit/SelectedConfig.hpp` header and
use `navkit::selected_config::Config` rather than including concrete config
headers directly.

When an app consumes runtime JSON, validate that input against the selected
compile-time composition before running. `SimulationApp<Config>` uses
`EmulatorBindings` to decide which runtime sections are required. Stable
unsigned `SensorId` values identify app/runtime streams. Configured emulator
types carry the stream ID, explicit NavKit sensor aliases document which
concrete configured sensor each emulator feeds, and `EmulatorBinding<Emulator,
Sensor>` verifies that `Emulator::Id` matches the selected sensor's
`Sensor::Id`. App-support helpers can query sensors or emulator bindings by ID
without ambiguous model-type lookup.
