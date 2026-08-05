// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Geometry>

namespace navkit::sim
{

/** Canonical ECI state advanced by generated dynamic trajectories. */
struct TrajectoryDynamicState
{
    core::Timestamp t{};
    core::Vec3 p_i_m{core::Vec3::Zero()};
    core::Vec3 v_i_mps{core::Vec3::Zero()};
    core::Vec3 a_i_mps2{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 specific_force_ib_b_mps2{core::Vec3::Zero()};
};

/**
 * Source-agnostic state supplied to Guidance and Autopilot.
 *
 * SimulationApp may populate this from truth, a Navigator estimate, or a future
 * hardware adapter without changing either controller implementation.
 */
struct TrajectoryControlState
{
    core::Timestamp t{};
    core::Vec3 p_i_m{core::Vec3::Zero()};
    core::Vec3 v_i_mps{core::Vec3::Zero()};
    core::Vec3 a_i_mps2{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};
};

/** Environment values resolved at the current ECI integration state. */
struct TrajectoryEnvironment
{
    core::Time_t elapsed_s{};
    core::Vec3 p_e_m{core::Vec3::Zero()};
    core::Vec3 v_eb_e_mps{core::Vec3::Zero()};
    core::Vec3 a_eb_e_mps2{core::Vec3::Zero()};
    core::Vec3 v_eb_n_mps{core::Vec3::Zero()};
    core::Vec3 gravity_i_mps2{core::Vec3::Zero()};
    core::Vec3 gravity_n_mps2{core::Vec3::Zero()};
    core::Mat3 C_i2e{core::Mat3::Identity()};
    core::Mat3 C_e2i{core::Mat3::Identity()};
    core::Mat3 C_e2n{core::Mat3::Identity()};
    core::Mat3 C_i2n{core::Mat3::Identity()};
    core::Mat3 C_n2i{core::Mat3::Identity()};
};

} // namespace navkit::sim
