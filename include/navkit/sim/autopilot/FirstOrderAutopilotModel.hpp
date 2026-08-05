// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/autopilot/AutopilotModel.hpp"
#include "navkit/sim/autopilot/ImuIncrementMovingAverage.hpp"

#include <memory>

namespace navkit::sim
{

enum class AutopilotModelType
{
    FirstOrder,
};

/** Runtime configuration for the controller-side first-order Autopilot response. */
struct FirstOrderAutopilotConfig
{
    core::Time_t attitude_command_time_constant_s{};
    core::Vec3 controller_rate_time_constant_pqr_s{core::Vec3::Zero()};
    core::Vec3 attitude_error_gain_pqr_per_s{core::Vec3::Ones()};
    core::Vec3 angular_rate_feedback_gain_pqr{core::Vec3::Zero()};
    core::Scalar_t velocity_alignment_speed_threshold_mps{1.0};
    core::Scalar_t initial_velocity_alignment_tolerance_rad{0.05};
    std::size_t gyro_moving_average_window_samples{1U};
};

[[nodiscard]] bool first_order_autopilot_config_is_valid(const FirstOrderAutopilotConfig& config);

class FirstOrderAutopilotModel final : public AutopilotModel
{
public:
    static constexpr std::size_t max_gyro_window_samples = 4096U;

    explicit FirstOrderAutopilotModel(FirstOrderAutopilotConfig config);

    [[nodiscard]] bool initialize(const AutopilotState& initial_state) override;

    [[nodiscard]] bool
    observe_imu_increment(const core::estimation::ImuIncrement& increment) override;

    [[nodiscard]] bool advance(const GuidanceCommand& guidance,
                               const AutopilotState& state,
                               const AutopilotExecutionState& execution,
                               core::Time_t dt_s,
                               VehicleCommand& command,
                               AutopilotOutput& output) override;

private:
    FirstOrderAutopilotConfig m_config{};
    ImuIncrementMovingAverage<max_gyro_window_samples> m_gyro_average{};
    Eigen::Quaternion<core::Scalar_t> m_launch_q_command_b2i{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    Eigen::Quaternion<core::Scalar_t> m_latched_q_command_b2i{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    Eigen::Quaternion<core::Scalar_t> m_previous_q_command_b2i{
        Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 m_prior_right_n{core::Vec3::UnitY()};
    core::Vec3 m_w_controller_response_ib_b_radps{core::Vec3::Zero()};
    bool m_initialized{false};
};

/** Creates the selected simulation-only Autopilot model. */
[[nodiscard]] std::unique_ptr<AutopilotModel>
make_autopilot_model(AutopilotModelType type, const FirstOrderAutopilotConfig& first_order_config);

} // namespace navkit::sim
