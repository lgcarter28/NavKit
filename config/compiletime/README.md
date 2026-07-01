# Compile-Time Configurations

This directory contains repository-provided C++ configuration headers.

- `examples/` contains teaching examples that demonstrate composition shape.
- `navkit_sim/` contains configurations used by the `navkit_sim` application.
- `targets/` may be added later for desktop and embedded product targets.

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
