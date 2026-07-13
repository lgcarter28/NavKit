// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

namespace navkit::core::estimation
{

struct ConingSculling
{
    Vec3 delta_theta_ib_b_rad{Vec3::Zero()};
    Vec3 delta_v_ib_b_mps{Vec3::Zero()};
};

[[nodiscard]] inline ConingSculling coning_sculling_single(const Vec3& delta_theta_ib_b_rad,
                                                           const Vec3& delta_v_ib_b_mps)
{
    return {.delta_theta_ib_b_rad = delta_theta_ib_b_rad, .delta_v_ib_b_mps = delta_v_ib_b_mps};
}

[[nodiscard]] inline ConingSculling coning_sculling_two_sample(const Vec3& delta_theta_1_ib_b_rad,
                                                               const Vec3& delta_v_1_ib_b_mps,
                                                               const Vec3& delta_theta_2_ib_b_rad,
                                                               const Vec3& delta_v_2_ib_b_mps)
{
    const auto delta_theta = delta_theta_1_ib_b_rad + delta_theta_2_ib_b_rad +
                             ((2.0 / 3.0) * delta_theta_1_ib_b_rad.cross(delta_theta_2_ib_b_rad));
    const auto delta_v = delta_v_1_ib_b_mps + delta_v_2_ib_b_mps +
                         (0.5 * (delta_theta_1_ib_b_rad + delta_theta_2_ib_b_rad)
                                    .cross(delta_v_1_ib_b_mps + delta_v_2_ib_b_mps)) +
                         ((2.0 / 3.0) * ((delta_theta_1_ib_b_rad.cross(delta_v_2_ib_b_mps)) +
                                         (delta_v_1_ib_b_mps.cross(delta_theta_2_ib_b_rad))));
    return {.delta_theta_ib_b_rad = delta_theta, .delta_v_ib_b_mps = delta_v};
}

} // namespace navkit::core::estimation
