// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/autopilot/FirstOrderAutopilotModel.hpp"

#include "navkit/core/math/Quaternion.hpp"
#include "navkit/sim/math/FirstOrderResponse.hpp"
#include "navkit/sim/trajectory/TrajectoryAttitude.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace navkit::sim
{

namespace
{

[[nodiscard]] bool nonnegative_finite(const core::Vec3& value)
{
    return value.allFinite() && (value.array() >= 0.0).all();
}

} // namespace

bool first_order_autopilot_config_is_valid(const FirstOrderAutopilotConfig& config)
{
    return std::isfinite(config.attitude_command_time_constant_s) &&
           config.attitude_command_time_constant_s >= 0.0 &&
           nonnegative_finite(config.controller_rate_time_constant_pqr_s) &&
           nonnegative_finite(config.attitude_error_gain_pqr_per_s) &&
           nonnegative_finite(config.angular_rate_feedback_gain_pqr) &&
           std::isfinite(config.velocity_alignment_speed_threshold_mps) &&
           config.velocity_alignment_speed_threshold_mps > 0.0 &&
           std::isfinite(config.initial_velocity_alignment_tolerance_rad) &&
           config.initial_velocity_alignment_tolerance_rad >= 0.0 &&
           config.initial_velocity_alignment_tolerance_rad <= std::numbers::pi_v<core::Scalar_t> &&
           config.gyro_moving_average_window_samples > 0U &&
           config.gyro_moving_average_window_samples <=
               FirstOrderAutopilotModel::max_gyro_window_samples;
}

FirstOrderAutopilotModel::FirstOrderAutopilotModel(FirstOrderAutopilotConfig config)
    : m_config(std::move(config))
{}

bool FirstOrderAutopilotModel::initialize(const AutopilotState& initial_state)
{
    const bool first_initialization = !m_initialized;
    if (!first_order_autopilot_config_is_valid(m_config) ||
        !initial_state.q_b2i.coeffs().allFinite() || initial_state.q_b2i.norm() <= 0.0 ||
        !initial_state.w_ib_b_radps.allFinite() || !initial_state.v_eb_n_mps.allFinite() ||
        !initial_state.C_i2n.allFinite()) {
        return false;
    }
    if (first_initialization) {
        if (!m_gyro_average.initialize(m_config.gyro_moving_average_window_samples)) {
            return false;
        }
    }
    else {
        m_gyro_average.clear();
    }

    const core::Scalar_t speed_mps = initial_state.v_eb_n_mps.norm();
    if (first_initialization && speed_mps > m_config.velocity_alignment_speed_threshold_mps) {
        const core::Vec3 forward_n =
            initial_state.C_i2n * (initial_state.q_b2i * core::Vec3::UnitX());
        const core::Scalar_t alignment_cosine = std::clamp(
            forward_n.normalized().dot(initial_state.v_eb_n_mps.normalized()), -1.0, 1.0);
        if (!std::isfinite(alignment_cosine) ||
            std::acos(alignment_cosine) > m_config.initial_velocity_alignment_tolerance_rad) {
            return false;
        }
    }

    const Eigen::Quaternion<core::Scalar_t> normalized_initial_q_b2i =
        core::math::normalized_with_positive_scalar(initial_state.q_b2i);
    if (first_initialization) {
        m_launch_q_command_b2i = normalized_initial_q_b2i;
    }
    m_latched_q_command_b2i = normalized_initial_q_b2i;
    m_previous_q_command_b2i = normalized_initial_q_b2i;
    m_prior_right_n = initial_state.C_i2n * (m_latched_q_command_b2i * core::Vec3::UnitY());
    if (!m_prior_right_n.allFinite() || m_prior_right_n.norm() <= 1.0e-12) {
        return false;
    }
    m_prior_right_n.normalize();
    m_w_controller_response_ib_b_radps = initial_state.w_ib_b_radps;
    m_initialized = true;
    return true;
}

bool FirstOrderAutopilotModel::observe_imu_increment(
    const core::estimation::ImuIncrement& increment)
{
    return m_initialized && m_gyro_average.push(increment);
}

bool FirstOrderAutopilotModel::advance(const GuidanceCommand& guidance,
                                       const AutopilotState& state,
                                       const AutopilotExecutionState& execution,
                                       const core::Time_t dt_s,
                                       VehicleCommand& command,
                                       AutopilotOutput& output)
{
    command = {};
    output = {};
    if (!m_initialized || !std::isfinite(dt_s) || dt_s <= 0.0 ||
        !state.q_b2i.coeffs().allFinite() || state.q_b2i.norm() <= 0.0 ||
        !state.w_ib_b_radps.allFinite() || !state.v_eb_n_mps.allFinite() ||
        !state.C_i2n.allFinite() || !guidance.specific_force_command_ib_b_mps2.allFinite()) {
        return false;
    }

    command.specific_force_command_ib_b_mps2 = guidance.specific_force_command_ib_b_mps2;

    core::Vec3 gyro_observation_radps = state.w_ib_b_radps;
    if (!m_gyro_average.average_rate(gyro_observation_radps)) {
        gyro_observation_radps = state.w_ib_b_radps;
    }

    if (!execution.active) {
        m_latched_q_command_b2i = execution.hold_initial_attitude
                                      ? m_launch_q_command_b2i
                                      : core::math::normalized_with_positive_scalar(state.q_b2i);
        m_prior_right_n = state.C_i2n * (m_latched_q_command_b2i * core::Vec3::UnitY());
        if (!m_prior_right_n.allFinite() || m_prior_right_n.norm() <= 1.0e-12) {
            return false;
        }
        m_prior_right_n.normalize();
        m_w_controller_response_ib_b_radps = state.w_ib_b_radps;
        m_previous_q_command_b2i = m_latched_q_command_b2i;
        output.q_command_b2i = m_latched_q_command_b2i;
        output.w_command_ib_b_radps = state.w_ib_b_radps;
        output.w_controller_response_ib_b_radps = state.w_ib_b_radps;
        output.gyro_observation_ib_b_radps = gyro_observation_radps;
        command.w_command_ib_b_radps = state.w_ib_b_radps;
        return true;
    }

    Eigen::Quaternion<core::Scalar_t> q_target_b2i = m_latched_q_command_b2i;
    if (state.v_eb_n_mps.norm() > m_config.velocity_alignment_speed_threshold_mps) {
        Eigen::Quaternion<core::Scalar_t> q_zero_bank_b2n{};
        core::Vec3 zero_bank_right_n{};
        if (!velocity_aligned_attitude_b2n(state.v_eb_n_mps,
                                           core::Vec3::UnitZ(),
                                           m_prior_right_n,
                                           q_zero_bank_b2n,
                                           zero_bank_right_n)) {
            return false;
        }
        const Eigen::Quaternion<core::Scalar_t> q_bank_b2b_zero =
            core::math::quaternion_from_rotvec_rad(
                core::Vec3{guidance.bank_command_n_rad, 0.0, 0.0});
        const Eigen::Quaternion<core::Scalar_t> q_b2n =
            core::math::normalized_with_positive_scalar(q_zero_bank_b2n * q_bank_b2b_zero);
        q_target_b2i = core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<core::Scalar_t>{state.C_i2n.transpose()} * q_b2n);
        m_prior_right_n = q_b2n * core::Vec3::UnitY();
        if (!m_prior_right_n.allFinite() || m_prior_right_n.norm() <= 1.0e-12) {
            return false;
        }
        m_prior_right_n.normalize();
    }

    const core::Scalar_t command_fraction =
        m_config.attitude_command_time_constant_s == 0.0
            ? 1.0
            : 1.0 - std::exp(-dt_s / m_config.attitude_command_time_constant_s);
    const Eigen::Quaternion<core::Scalar_t> q_command_b2i =
        core::math::normalized_with_positive_scalar(
            m_latched_q_command_b2i.slerp(command_fraction, q_target_b2i));
    const Eigen::Quaternion<core::Scalar_t> command_delta =
        core::math::normalized_with_positive_scalar(m_previous_q_command_b2i.conjugate() *
                                                    q_command_b2i);
    const core::Vec3 command_rate_previous_body_radps =
        core::math::rotvec_rad_from_quaternion(command_delta) / dt_s;
    const Eigen::Quaternion<core::Scalar_t> q_previous_command_to_body =
        core::math::normalized_with_positive_scalar(state.q_b2i.conjugate() *
                                                    m_previous_q_command_b2i);
    const core::Vec3 command_rate_body_radps =
        q_previous_command_to_body * command_rate_previous_body_radps;
    m_latched_q_command_b2i = q_command_b2i;
    m_previous_q_command_b2i = q_command_b2i;

    const Eigen::Quaternion<core::Scalar_t> attitude_error =
        core::math::normalized_with_positive_scalar(state.q_b2i.conjugate() * q_command_b2i);
    const core::Vec3 attitude_error_b_rad = core::math::rotvec_rad_from_quaternion(attitude_error);
    const core::Vec3 angular_rate_command_radps =
        command_rate_body_radps +
        m_config.attitude_error_gain_pqr_per_s.cwiseProduct(attitude_error_b_rad) +
        m_config.angular_rate_feedback_gain_pqr.cwiseProduct(command_rate_body_radps -
                                                             gyro_observation_radps);
    m_w_controller_response_ib_b_radps =
        exact_first_order_step(m_w_controller_response_ib_b_radps,
                               angular_rate_command_radps,
                               m_config.controller_rate_time_constant_pqr_s,
                               dt_s);

    output.q_command_b2i = q_command_b2i;
    output.w_command_ib_b_radps = angular_rate_command_radps;
    output.w_feedforward_ib_b_radps = command_rate_body_radps;
    output.w_controller_response_ib_b_radps = m_w_controller_response_ib_b_radps;
    output.gyro_observation_ib_b_radps = gyro_observation_radps;
    output.active = true;
    command.w_command_ib_b_radps = m_w_controller_response_ib_b_radps;
    return true;
}

std::unique_ptr<AutopilotModel>
make_autopilot_model(const AutopilotModelType type,
                     const FirstOrderAutopilotConfig& first_order_config)
{
    switch (type) {
    case AutopilotModelType::FirstOrder:
        return std::make_unique<FirstOrderAutopilotModel>(first_order_config);
    }
    return {};
}

} // namespace navkit::sim
