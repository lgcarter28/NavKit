// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::core::environment
{

template<RotatingPlanetPolicy Planet>
[[nodiscard]] inline Vec3 planet_rate_fixed_radps()
{
    return Vec3{0.0, 0.0, Planet::omega_rad_s};
}

} // namespace navkit::core::environment
