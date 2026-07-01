// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/common/Config.hpp"
#include "navkit/environment/gravity/GravityPolicyBase.hpp"
#include "navkit/environment/planet/PlanetPolicy.hpp"

#include <Eigen/Dense>

namespace navkit::gravity
{

template<planet::J2PlanetPolicy Planet>
struct J2 : GravityPolicyBase<J2<Planet>>
{
    using Planet_t = Planet;
    using Frame_t = typename Planet::FixedFrame;

    [[nodiscard]] static Eigen::Matrix<Scalar_t, 3, 1>
    acceleration(const Eigen::Matrix<Scalar_t, 3, 1>& p_fixed)
    {
        const Scalar_t x = p_fixed.x();
        const Scalar_t y = p_fixed.y();
        const Scalar_t z = p_fixed.z();

        const Scalar_t r2 = p_fixed.squaredNorm();
        if (r2 <= 0.0) {
            return Eigen::Matrix<Scalar_t, 3, 1>::Zero();
        }

        const Scalar_t r = std::sqrt(r2);
        const Scalar_t z2_over_r2 = (z * z) / r2;
        const Scalar_t re_over_r = Planet::a_m / r;
        const Scalar_t j2_scale = 1.5 * Planet::J2 * re_over_r * re_over_r;
        const Scalar_t common = -Planet::mu_m3_s2 / (r2 * r);

        Eigen::Matrix<Scalar_t, 3, 1> a{};
        a.x() = common * x * (1.0 - j2_scale * (5.0 * z2_over_r2 - 1.0));
        a.y() = common * y * (1.0 - j2_scale * (5.0 * z2_over_r2 - 1.0));
        a.z() = common * z * (1.0 - j2_scale * (5.0 * z2_over_r2 - 3.0));
        return a;
    }
};

} // namespace navkit::gravity
