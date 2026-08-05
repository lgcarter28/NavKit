// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Geometry>
#include <cstddef>

namespace navkit::io
{

/**
 * Fully resolved trajectory inspection data at one trajectory timestamp.
 *
 * The simulation/app-support boundary computes this value once from canonical
 * trajectory truth and diagnostics. The individual log products below then
 * select only the frame-specific fields they own.
 */
struct TrajectoryLogData
{
    core::Timestamp t{};

    core::Vec3 p_e_m{core::Vec3::Zero()};
    core::Vec3 v_e_mps{core::Vec3::Zero()};
    core::Vec3 a_e_mps2{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_eb_b_radps{core::Vec3::Zero()};

    core::Vec3 p_i_m{core::Vec3::Zero()};
    core::Vec3 v_i_mps{core::Vec3::Zero()};
    core::Vec3 a_i_mps2{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};

    core::Vec3 p_lla_deg_m{core::Vec3::Zero()};
    core::Vec3 v_n_mps{core::Vec3::Zero()};
    core::Vec3 a_n_mps2{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2n{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_nb_b_radps{core::Vec3::Zero()};

    core::Vec3 v_ib_b_mps{core::Vec3::Zero()};
    core::Vec3 a_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 v_eb_b_mps{core::Vec3::Zero()};
    core::Vec3 a_eb_b_mps2{core::Vec3::Zero()};
    core::Vec3 specific_force_ib_b_mps2{core::Vec3::Zero()};

    core::Vec3 guidance_velocity_reference_i_mps{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_command_i_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_command_n_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_command_b_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_response_i_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_response_n_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_acceleration_response_b_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_specific_force_command_b_mps2{core::Vec3::Zero()};
    core::Vec3 guidance_specific_force_filtered_b_mps2{core::Vec3::Zero()};
    core::Scalar_t guidance_bank_command_n_rad{};
    core::Scalar_t guidance_bank_filtered_n_rad{};
    core::Scalar_t guidance_bank_response_n_rad{};
    core::Vec3 guidance_reference_position_e_m{core::Vec3::Zero()};
    std::size_t guidance_reference_index{};
    std::size_t guidance_state_index{};
    bool guidance_active{false};
    bool pad_constraint_active{false};
    bool guidance_reference_position_valid{false};

    Eigen::Quaternion<core::Scalar_t> autopilot_q_command_b2i{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    Eigen::Quaternion<core::Scalar_t> autopilot_q_response_b2i{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    Eigen::Quaternion<core::Scalar_t> autopilot_q_command_b2n{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    Eigen::Quaternion<core::Scalar_t> autopilot_q_response_b2n{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 autopilot_angular_rate_command_b_radps{core::Vec3::Zero()};
    core::Vec3 autopilot_angular_rate_feedforward_b_radps{core::Vec3::Zero()};
    core::Vec3 autopilot_angular_rate_controller_response_b_radps{core::Vec3::Zero()};
    core::Vec3 autopilot_gyro_observation_b_radps{core::Vec3::Zero()};
    bool autopilot_active{false};

    core::Vec3 vehicle_velocity_ib_b_mps{core::Vec3::Zero()};
    core::Vec3 vehicle_acceleration_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 vehicle_velocity_eb_b_mps{core::Vec3::Zero()};
    core::Vec3 vehicle_acceleration_eb_b_mps2{core::Vec3::Zero()};
    core::Vec3 vehicle_specific_force_command_b_mps2{core::Vec3::Zero()};
    core::Vec3 vehicle_specific_force_command_response_b_mps2{core::Vec3::Zero()};
    core::Vec3 vehicle_specific_force_response_b_mps2{core::Vec3::Zero()};
    core::Vec3 vehicle_angular_rate_command_b_radps{core::Vec3::Zero()};
    core::Vec3 vehicle_angular_rate_response_b_radps{core::Vec3::Zero()};

    core::Vec3 velocity_tracking_error_b_mps{core::Vec3::Zero()};
    core::Vec3 acceleration_tracking_error_b_mps2{core::Vec3::Zero()};
    core::Vec3 attitude_tracking_error_b_rad{core::Vec3::Zero()};
    core::Vec3 angular_rate_tracking_error_b_radps{core::Vec3::Zero()};
    core::Vec3 specific_force_tracking_error_b_mps2{core::Vec3::Zero()};
    Eigen::Array<bool, 3, 1> angular_rate_limited{Eigen::Array<bool, 3, 1>::Constant(false)};
    Eigen::Array<bool, 3, 1> specific_force_limited{Eigen::Array<bool, 3, 1>::Constant(false)};
};

struct TrajectoryEcefLogPayload
{
    const TrajectoryLogData& data;
};

struct TrajectoryEciLogPayload
{
    const TrajectoryLogData& data;
};

struct TrajectoryNedLogPayload
{
    const TrajectoryLogData& data;
};

struct TrajectoryBodyLogPayload
{
    const TrajectoryLogData& data;
};

struct TrajectoryGuidanceLogPayload
{
    const TrajectoryLogData& data;
};

struct TrajectoryAutopilotVehicleLogPayload
{
    const TrajectoryLogData& data;
};

} // namespace navkit::io
