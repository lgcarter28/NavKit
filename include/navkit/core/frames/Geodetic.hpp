// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/math/Types.hpp"

#include <cmath>
#include <numbers>

namespace navkit::core::frames
{

[[nodiscard]] inline Scalar_t rad_to_deg(const Scalar_t angle_rad)
{
    return angle_rad * (180.0 / std::numbers::pi_v<Scalar_t>);
}

[[nodiscard]] inline Scalar_t deg_to_rad(const Scalar_t angle_deg)
{
    return angle_deg * (std::numbers::pi_v<Scalar_t> / 180.0);
}

template<environment::EllipsoidPlanetPolicy Planet>
[[nodiscard]] inline bool fixed_m_to_lla_deg_m(const Vec3& p_f_m, Vec3& p_lla_deg_m)
{
    if (!p_f_m.allFinite() || p_f_m.norm() <= 0.0) {
        return false;
    }
    const Scalar_t a_m = Planet::a_m;
    const Scalar_t b_m = Planet::b_m;
    const Scalar_t e2 = 1.0 - ((b_m * b_m) / (a_m * a_m));
    const Scalar_t ep2 = ((a_m * a_m) - (b_m * b_m)) / (b_m * b_m);
    const Scalar_t p_xy_m = std::hypot(p_f_m.x(), p_f_m.y());
    const Scalar_t lon_rad = std::atan2(p_f_m.y(), p_f_m.x());
    const Scalar_t theta_rad = std::atan2(p_f_m.z() * a_m, p_xy_m * b_m);
    const Scalar_t sin_theta = std::sin(theta_rad);
    const Scalar_t cos_theta = std::cos(theta_rad);
    const Scalar_t lat_rad = std::atan2(p_f_m.z() + (ep2 * b_m * sin_theta * sin_theta * sin_theta),
                                        p_xy_m - (e2 * a_m * cos_theta * cos_theta * cos_theta));
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t N_m = a_m / std::sqrt(1.0 - (e2 * sin_lat * sin_lat));
    Scalar_t h_m{};
    const Scalar_t cos_lat = std::cos(lat_rad);
    if (std::abs(cos_lat) > 1.0e-12) {
        h_m = (p_xy_m / cos_lat) - N_m;
    }
    else {
        h_m = std::abs(p_f_m.z()) - b_m;
    }

    p_lla_deg_m = Vec3{rad_to_deg(lat_rad), rad_to_deg(lon_rad), h_m};
    return p_lla_deg_m.allFinite();
}

template<environment::EllipsoidPlanetPolicy Planet>
[[nodiscard]] inline bool lla_deg_m_to_fixed_m(const Vec3& p_lla_deg_m, Vec3& p_f_m)
{
    if (!p_lla_deg_m.allFinite() || p_lla_deg_m.x() < -90.0 || p_lla_deg_m.x() > 90.0) {
        return false;
    }
    const Scalar_t lat_rad = deg_to_rad(p_lla_deg_m.x());
    const Scalar_t lon_rad = deg_to_rad(p_lla_deg_m.y());
    const Scalar_t h_m = p_lla_deg_m.z();
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t cos_lat = std::cos(lat_rad);
    const Scalar_t sin_lon = std::sin(lon_rad);
    const Scalar_t cos_lon = std::cos(lon_rad);
    const Scalar_t e2 = 1.0 - ((Planet::b_m * Planet::b_m) / (Planet::a_m * Planet::a_m));
    const Scalar_t prime_vertical_radius_m =
        Planet::a_m / std::sqrt(1.0 - (e2 * sin_lat * sin_lat));
    p_f_m = Vec3{(prime_vertical_radius_m + h_m) * cos_lat * cos_lon,
                 (prime_vertical_radius_m + h_m) * cos_lat * sin_lon,
                 ((1.0 - e2) * prime_vertical_radius_m + h_m) * sin_lat};
    return p_f_m.allFinite();
}

[[nodiscard]] inline bool ecef_m_to_lla_deg_m(const Vec3& p_e_m, Vec3& p_lla_deg_m)
{
    return fixed_m_to_lla_deg_m<environment::Wgs84>(p_e_m, p_lla_deg_m);
}

[[nodiscard]] inline Vec3 ecef_m_to_lla_deg_m(const Vec3& p_e_m)
{
    Vec3 p_lla_deg_m{};
    (void)ecef_m_to_lla_deg_m(p_e_m, p_lla_deg_m);
    return p_lla_deg_m;
}

[[nodiscard]] inline bool lla_deg_m_to_ecef_m(const Vec3& p_lla_deg_m, Vec3& p_e_m)
{
    return lla_deg_m_to_fixed_m<environment::Wgs84>(p_lla_deg_m, p_e_m);
}

} // namespace navkit::core::frames
