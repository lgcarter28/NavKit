// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <cmath>

namespace navkit::core::frames
{

[[nodiscard]] inline Mat3 ecef_to_ned_matrix(const Vec3& p_e_m)
{
    const Scalar_t lon_rad = std::atan2(p_e_m.y(), p_e_m.x());
    const Scalar_t hyp_m = std::hypot(p_e_m.x(), p_e_m.y());
    const Scalar_t lat_rad = std::atan2(p_e_m.z(), hyp_m);
    const Scalar_t sin_lat = std::sin(lat_rad);
    const Scalar_t cos_lat = std::cos(lat_rad);
    const Scalar_t sin_lon = std::sin(lon_rad);
    const Scalar_t cos_lon = std::cos(lon_rad);

    Mat3 C_e2n;
    C_e2n << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon, 0.0,
        -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
    return C_e2n;
}

[[nodiscard]] inline Mat3 ned_to_ecef_matrix(const Vec3& p_e_m)
{
    return ecef_to_ned_matrix(p_e_m).transpose();
}

} // namespace navkit::core::frames
