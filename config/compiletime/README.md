# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `examples/` contains teaching examples that demonstrate composition shape.
- `navkit_sim/` contains configurations used by the `navkit_sim` application.
- `targets/` may be added later for desktop and embedded product targets.

Provided `navkit_sim` selections:

- `navkit_sim/StationaryGnss.hpp`: default stationary GNSS demo with measurement
  statistics and no embedded profile export.
- `navkit_sim/ProfiledStationaryGnss.hpp`: same demo shape, but selects a host
  microsecond clock, fixed-capacity profiling ring-buffer sink, and scoped
  profiler so the app emits `profile.csv`.

Each concrete config header should include local `static_assert` checks for the
concept slices it claims to satisfy.

Headers intended for `NAVKIT_CONFIG` selection must also expose:

```cpp
namespace navkit::config
{
using SelectedConfig = ...;
}
```

Applications should include the generated `navkit/SelectedConfig.hpp` header and
use `navkit::selected_config::Config` rather than including concrete config
headers directly.
