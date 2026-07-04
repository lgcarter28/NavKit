# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `navkit/products/` contains reusable NavKit product graph configurations,
  including the deliberately small `MinimalConfig.hpp` teaching config.
- `apps/` contains top-level executable composition configurations.
- `targets/` may be added later for desktop and embedded product targets.

The `navkit/` and `apps/` trees are intentionally separate. It is normal for
both trees to contain the same descriptive file name, such as
`StationaryGnss.hpp`: the NavKit product file owns reusable library
configuration, while the app file owns the executable composition that links a
NavKit config to an app runner.

Provided `NAVKIT_CONFIG` selections:

- `apps/navkit_sim/StationaryGnss.hpp`: default stationary GNSS demo app using
  the unprofiled NavKit GNSS library config.
- `apps/navkit_sim/ProfiledStationaryGnss.hpp`: same app shape, but consumes a
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

    static constexpr navkit::app_support::SensorId PrimarySensorId =
        NavKit::PrimarySensorId;
    using EmulatorBindings = std::tuple<
        navkit::app_support::EmulatorBinding<
            PrimarySensorId,
            navkit::app_support::SomeEmulator,
            NavKit::PrimarySensor>>;

    using App = navkit::app_support::SimulationApp<ExampleAppConfig>;
};
```

Applications should include the generated `navkit/SelectedConfig.hpp` header and
use `navkit::selected_config::Config` rather than including concrete config
headers directly.

When an app consumes runtime JSON, validate that input against the selected
compile-time composition before running. `SimulationApp<Config>` uses
`EmulatorBindings` to decide which runtime sections are required. Stable
unsigned `SensorId` values identify app/runtime streams, while explicit NavKit
sensor aliases document which concrete configured sensor each emulator feeds.
The binding ID must match the selected sensor's `Sensor::Id`, and app-support
helpers can query sensors or emulator bindings by ID without ambiguous model-type
lookup.
