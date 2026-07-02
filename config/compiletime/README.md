# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `navkit/` contains NavKit library/core policy configurations, including the
  deliberately small `MinimalConfig.hpp` teaching config.
- `apps/` contains top-level executable composition configurations.
- `targets/` may be added later for desktop and embedded product targets.

The `navkit/` and `apps/` trees are intentionally separate. It is normal for
both trees to contain the same descriptive file name, such as
`StationaryGnss.hpp`: the NavKit file owns reusable library configuration,
while the app file owns the executable composition that links a NavKit config to
an app runner.

Provided `NAVKIT_CONFIG` selections:

- `apps/navkit_sim/StationaryGnss.hpp`: default stationary GNSS demo app using
  the unprofiled NavKit GNSS library config.
- `apps/navkit_sim/ProfiledStationaryGnss.hpp`: same app shape, but consumes a
  profiled NavKit GNSS library config with a host microsecond clock,
  fixed-capacity profiling ring-buffer sink, and scoped profiler so the app
  emits `profile.csv`.

Each concrete config header should include local `static_assert` checks for the
concept slices it claims to satisfy.

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
    using App = navkit::app_support::SomeApp<ExampleAppConfig>;
};
```

Applications should include the generated `navkit/SelectedConfig.hpp` header and
use `navkit::selected_config::Config` rather than including concrete config
headers directly.

When an app consumes runtime JSON, validate that input against the selected
compile-time composition before running. The current stationary GNSS app does
this in `navkit::app_support` so missing `trajectory`/`gnss` sections,
unsupported sensor sections, and malformed numeric/vector fields produce clear
early errors.
