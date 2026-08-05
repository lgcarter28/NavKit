// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>

namespace navkit::sim
{

/** Inspectable command-generation and controller-response diagnostics from Autopilot. */
struct AutopilotOutput
{
    Eigen::Quaternion<core::Scalar_t> q_command_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_command_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 w_feedforward_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 w_controller_response_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 gyro_observation_ib_b_radps{core::Vec3::Zero()};
    bool active{false};
};

} // namespace navkit::sim
