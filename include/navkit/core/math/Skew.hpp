// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Dense>

namespace navkit::core::math
{

[[nodiscard]] inline Eigen::Matrix<Scalar_t, 3, 3> skew_symmetric(const Vec3& vector)
{
    Eigen::Matrix<Scalar_t, 3, 3> matrix;
    matrix << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(),
        0.0;
    return matrix;
}

} // namespace navkit::core::math
