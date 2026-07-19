// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/math/Types.hpp"

#include <cmath>

namespace navkit::core::frames
{

[[nodiscard]] inline Scalar_t rad_to_deg(const Scalar_t angle_rad)
{
    constexpr Scalar_t pi = 3.141592653589793238462643383279502884;
    return angle_rad * (180.0 / pi);
}

[[nodiscard]] inline Vec3 ecef_m_to_lla_deg_m(const Vec3& p_e_m)
{
    using Planet = environment::Wgs84;

    const Scalar_t a_m = Planet::a_m;
    const Scalar_t b_m = Planet::b_m;
    const Scalar_t e2 = 1.0 - ((b_m * b_m) / (a_m * a_m));
    const Scalar_t ep2 = ((a_m * a_m) - (b_m * b_m)) / (b_m * b_m);
    const Scalar_t p_xy_m = std::hypot(p_e_m.x(), p_e_m.y());
    const Scalar_t lon_rad = std::atan2(p_e_m.y(), p_e_m.x());
    const Scalar_t theta_rad = std::atan2(p_e_m.z() * a_m, p_xy_m * b_m);
    const Scalar_t sin_theta = std::sin(theta_rad);
    const Scalar_t cos_theta = std::cos(theta_rad);
    const Scalar_t lat_rad = std::atan2(p_e_m.z() + (ep2 * b_m * sin_theta * sin_theta * sin_theta),
                                        p_xy_m - (e2 * a_m * cos_theta * cos_theta * cos_theta));
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t N_m = a_m / std::sqrt(1.0 - (e2 * sin_lat * sin_lat));
    const Scalar_t h_m = (p_xy_m / std::cos(lat_rad)) - N_m;

    return Vec3{rad_to_deg(lat_rad), rad_to_deg(lon_rad), h_m};
}

} // namespace navkit::core::frames
