# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `navkit/products/variants/<product_family>/` contains a reusable product-family
  graph and its complete NavKit variants.
- `navkit/products/components/` contains smaller reusable slices grouped by their
  owning domain: `propagation`, `filter`, `sensors`, `profiling`, and `foundation`.
- `apps/<app>/variants/<product_family>/` contains the matching top-level
  executable compositions.
- `targets/` may be added later for desktop and embedded product targets.

The `navkit/` and `apps/` trees are intentionally separate. It is normal for
both variant trees to contain the same descriptive file name, such as
`EcefInsGnssLcGyroAccelBiasDefault.hpp`: the NavKit product file owns reusable
library configuration, while the app file owns the executable composition that
links a NavKit config to an app runner.

Provided `NAVKIT_CONFIG` selections:

- `apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp`: default
  scenario-agnostic simulation app using the unprofiled NavKit GNSS library
  config.
- `apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasProfiled.hpp`: same app shape, but consumes a
  profiled NavKit GNSS library config with a host microsecond clock,
  fixed-capacity profiling ring-buffer sink, and scoped profiler so the app
  emits `profile.csv`.

Runnable/product NavKit configs should read like product graphs and usually end
with a single aggregate check, such as
`static_assert(navkit::api::config::NavKitProductConfigPolicy<Config>);`.
Detailed slice-level concept assertions belong in focused tests.
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

    using App = navkit::app_support::SimulationApp<ExampleAppConfig>;
};
```

`SimulationApp` uses `navkit::app_support::RuntimeLogger<NavKit>` to expose the
simulation log-product catalog. Runtime scenario logging settings select which
non-embedded products are enabled and at what rates; the compile-time NavKit
product config remains focused on the embedded-facing estimator graph.

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
