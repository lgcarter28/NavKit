// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/gravity/GravityPolicy.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Dense>
#include <algorithm>

namespace navkit::core::environment
{

template<GravityPolicy Gravity>
[[nodiscard]] inline Eigen::Matrix<Scalar_t, 3, 3>
gravity_gradient_fixed_mps2_per_m(const Vec3& position_fixed_m)
{
    Eigen::Matrix<Scalar_t, 3, 3> gradient = Eigen::Matrix<Scalar_t, 3, 3>::Zero();
    const auto h_m = std::max<Scalar_t>(1.0, position_fixed_m.norm() * 1.0e-7);
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        Vec3 step = Vec3::Zero();
        step(axis) = h_m;
        gradient.col(axis) = (Gravity::acceleration(position_fixed_m + step) -
                              Gravity::acceleration(position_fixed_m - step)) /
                             (2.0 * h_m);
    }
    return gradient;
}

} // namespace navkit::core::environment
