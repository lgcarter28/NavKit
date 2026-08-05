// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/math/Types.hpp"

#include <cmath>

namespace navkit::core::frames
{

template<environment::EllipsoidPlanetPolicy Planet>
[[nodiscard]] inline bool fixed_to_ned_matrix(const Vec3& p_f_m, Mat3& C_f2n)
{
    Vec3 p_lla_deg_m{};
    if (!fixed_m_to_lla_deg_m<Planet>(p_f_m, p_lla_deg_m)) {
        return false;
    }
    const Scalar_t lat_rad = deg_to_rad(p_lla_deg_m.x());
    const Scalar_t lon_rad = deg_to_rad(p_lla_deg_m.y());
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t cos_lat = std::cos(lat_rad);
    const Scalar_t sin_lon = std::sin(lon_rad);
    const Scalar_t cos_lon = std::cos(lon_rad);

    C_f2n << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon, 0.0,
        -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
    return true;
}

[[nodiscard]] inline bool ecef_to_ned_matrix(const Vec3& p_e_m, Mat3& C_e2n)
{
    return fixed_to_ned_matrix<environment::Wgs84>(p_e_m, C_e2n);
}

[[nodiscard]] inline Mat3 ecef_to_ned_matrix(const Vec3& p_e_m)
{
    Mat3 C_e2n = Mat3::Identity();
    (void)ecef_to_ned_matrix(p_e_m, C_e2n);
    return C_e2n;
}

[[nodiscard]] inline bool ned_to_ecef_matrix(const Vec3& p_e_m, Mat3& C_n2e)
{
    Mat3 C_e2n{};
    if (!ecef_to_ned_matrix(p_e_m, C_e2n)) {
        return false;
    }
    C_n2e = C_e2n.transpose();
    return true;
}

[[nodiscard]] inline Mat3 ned_to_ecef_matrix(const Vec3& p_e_m)
{
    Mat3 C_n2e = Mat3::Identity();
    (void)ned_to_ecef_matrix(p_e_m, C_n2e);
    return C_n2e;
}

template<environment::EllipsoidPlanetPolicy Planet>
[[nodiscard]] inline bool
transport_rate_fixed_to_ned_radps(const Vec3& p_f_m, const Vec3& v_f_mps, Vec3& w_fn_n_radps)
{
    Vec3 p_lla_deg_m{};
    Mat3 C_f2n{};
    if (!fixed_m_to_lla_deg_m<Planet>(p_f_m, p_lla_deg_m) ||
        !fixed_to_ned_matrix<Planet>(p_f_m, C_f2n) || !v_f_mps.allFinite()) {
        return false;
    }
    const Scalar_t lat_rad = deg_to_rad(p_lla_deg_m.x());
    const Scalar_t h_m = p_lla_deg_m.z();
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t e2 = 1.0 - ((Planet::b_m * Planet::b_m) / (Planet::a_m * Planet::a_m));
    const Scalar_t denominator = 1.0 - (e2 * sin_lat * sin_lat);
    const Scalar_t prime_vertical_radius_m = Planet::a_m / std::sqrt(denominator);
    const Scalar_t meridian_radius_m =
        Planet::a_m * (1.0 - e2) / (denominator * std::sqrt(denominator));
    const Vec3 v_n_mps = C_f2n * v_f_mps;

    w_fn_n_radps = Vec3{v_n_mps.y() / (prime_vertical_radius_m + h_m),
                        -v_n_mps.x() / (meridian_radius_m + h_m),
                        -(v_n_mps.y() * std::tan(lat_rad)) / (prime_vertical_radius_m + h_m)};
    return w_fn_n_radps.allFinite();
}

[[nodiscard]] inline bool
transport_rate_en_n_radps(const Vec3& p_e_m, const Vec3& v_e_mps, Vec3& w_en_n_radps)
{
    return transport_rate_fixed_to_ned_radps<environment::Wgs84>(p_e_m, v_e_mps, w_en_n_radps);
}

} // namespace navkit::core::frames
