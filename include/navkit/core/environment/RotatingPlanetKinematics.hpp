// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/math/Skew.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::core::environment
{

template<RotatingPlanetPolicy Planet>
[[nodiscard]] inline Vec3 planet_rate_fixed_radps()
{
    return Vec3{0.0, 0.0, Planet::omega_rad_s};
}

/**
 * Returns the outward centrifugal pseudo-acceleration in the rotating
 * planet-fixed frame.
 *
 * Gravity policies provide mass-attraction gravitation. ECEF mechanizations
 * add this term explicitly when forming apparent/plumb-bob gravity.
 */
template<RotatingPlanetPolicy Planet>
[[nodiscard]] inline Vec3 centrifugal_acceleration_fixed_mps2(const Vec3& position_fixed_m)
{
    const Vec3 omega_if_fixed_radps = planet_rate_fixed_radps<Planet>();
    return -omega_if_fixed_radps.cross(omega_if_fixed_radps.cross(position_fixed_m));
}

/**
 * Returns the exact position Jacobian of
 * centrifugal_acceleration_fixed_mps2().
 */
template<RotatingPlanetPolicy Planet>
[[nodiscard]] inline Mat3 centrifugal_acceleration_gradient_fixed_1ps2()
{
    const Mat3 omega_if_fixed =
        navkit::core::math::skew_symmetric(planet_rate_fixed_radps<Planet>());
    return -(omega_if_fixed * omega_if_fixed);
}

} // namespace navkit::core::environment
