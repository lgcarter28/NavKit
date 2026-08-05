// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <Eigen/Core>

namespace navkit::sim
{

/** Final realized vehicle response used by truth integration and IMU synthesis. */
struct VehicleResponseOutput
{
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 specific_force_command_response_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 specific_force_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 a_i_mps2{core::Vec3::Zero()};
    Eigen::Array<bool, 3, 1> angular_rate_limited{Eigen::Array<bool, 3, 1>::Constant(false)};
    Eigen::Array<bool, 3, 1> specific_force_limited{Eigen::Array<bool, 3, 1>::Constant(false)};
};

} // namespace navkit::sim
