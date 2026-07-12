// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace navkit::core::math
{

[[nodiscard]] inline Eigen::Quaternion<Scalar_t>
normalized_with_positive_scalar(Eigen::Quaternion<Scalar_t> q)
{
    q.normalize();
    if (q.w() < 0.0) {
        q.coeffs() *= -1.0;
    }
    return q;
}

[[nodiscard]] inline Vec3 rotation_vector_from_quaternion(Eigen::Quaternion<Scalar_t> q)
{
    q = normalized_with_positive_scalar(q);
    const auto sin_half_angle = q.vec().norm();
    if (sin_half_angle <= 1.0e-15) {
        return 2.0 * q.vec();
    }

    const auto angle = 2.0 * std::atan2(sin_half_angle, q.w());
    return (angle / sin_half_angle) * q.vec();
}

} // namespace navkit::core::math
