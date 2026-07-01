// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/gravity/GravityPolicyBase.hpp"
#include "navkit/core/environment/planet/PlanetPolicy.hpp"

#include <Eigen/Dense>

namespace navkit::core::environment
{

using navkit::core::Scalar_t;

template<PlanetPolicy Planet>
struct Spherical : GravityPolicyBase<Spherical<Planet>>
{
    using Planet_t = Planet;
    using Frame_t = typename Planet::FixedFrame;

    [[nodiscard]] static Eigen::Matrix<Scalar_t, 3, 1>
    acceleration(const Eigen::Matrix<Scalar_t, 3, 1>& p_fixed)
    {
        const Scalar_t r = p_fixed.norm();
        if (r <= 0.0) {
            return Eigen::Matrix<Scalar_t, 3, 1>::Zero();
        }
        return -(Planet::mu_m3_s2 / (r * r * r)) * p_fixed;
    }
};

} // namespace navkit::core::environment
