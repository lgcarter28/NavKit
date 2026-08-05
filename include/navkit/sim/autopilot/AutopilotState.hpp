// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>

namespace navkit::sim
{

/** Minimal estimated state and frame context consumed by an Autopilot implementation. */
struct AutopilotState
{
    Eigen::Quaternion<core::Scalar_t> q_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 v_eb_n_mps{core::Vec3::Zero()};
    core::Mat3 C_i2n{core::Mat3::Identity()};
};

/** Runtime Autopilot activation state selected by the active Guidance state. */
struct AutopilotExecutionState
{
    bool active{false};
    bool hold_initial_attitude{false};
};

} // namespace navkit::sim
