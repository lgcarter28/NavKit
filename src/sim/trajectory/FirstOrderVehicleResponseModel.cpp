// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/FirstOrderVehicleResponseModel.hpp"

#include "navkit/sim/math/FirstOrderResponse.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navkit::sim
{

namespace
{

[[nodiscard]] bool nonnegative_finite(const core::Vec3& value)
{
    return value.allFinite() && (value.array() >= 0.0).all();
}

[[nodiscard]] bool positive_finite(const core::Vec3& value)
{
    return value.allFinite() && (value.array() > 0.0).all();
}

[[nodiscard]] core::Vec3
clamp_symmetric(const core::Vec3& value, const core::Vec3& limit, Eigen::Array<bool, 3, 1>& limited)
{
    core::Vec3 result = value;
    for (int axis = 0; axis < 3; ++axis) {
        const core::Scalar_t clamped = std::clamp(value(axis), -limit(axis), limit(axis));
        limited(axis) = clamped != value(axis);
        result(axis) = clamped;
    }
    return result;
}

} // namespace

bool first_order_vehicle_response_config_is_valid(const FirstOrderVehicleResponseConfig& config)
{
    return nonnegative_finite(config.vehicle_rate_time_constant_pqr_s) &&
           nonnegative_finite(config.specific_force_command_time_constant_b_s) &&
           nonnegative_finite(config.specific_force_response_time_constant_b_s) &&
           (!config.angular_rate_limits_enabled ||
            positive_finite(config.angular_rate_limit_pqr_radps)) &&
           (!config.specific_force_limits_enabled ||
            positive_finite(config.specific_force_limit_b_mps2));
}

FirstOrderVehicleResponseModel::FirstOrderVehicleResponseModel(
    FirstOrderVehicleResponseConfig config)
    : m_config(std::move(config))
{}

bool FirstOrderVehicleResponseModel::initialize(const TrajectoryDynamicState& initial_state)
{
    if (!first_order_vehicle_response_config_is_valid(m_config) ||
        !initial_state.w_ib_b_radps.allFinite() ||
        !initial_state.specific_force_ib_b_mps2.allFinite()) {
        return false;
    }
    m_w_ib_b_radps = initial_state.w_ib_b_radps;
    m_specific_force_command_response_ib_b_mps2 = initial_state.specific_force_ib_b_mps2;
    m_specific_force_ib_b_mps2 = initial_state.specific_force_ib_b_mps2;
    m_initialized = true;
    return true;
}

bool FirstOrderVehicleResponseModel::advance(const VehicleCommand& command,
                                             const TrajectoryDynamicState& state,
                                             const TrajectoryEnvironment& environment,
                                             const core::Time_t dt_s,
                                             VehicleResponseOutput& output)
{
    output = {};
    if (!m_initialized || !std::isfinite(dt_s) || dt_s <= 0.0 ||
        !command.w_command_ib_b_radps.allFinite() ||
        !command.specific_force_command_ib_b_mps2.allFinite() ||
        !state.q_b2i.coeffs().allFinite() || !environment.gravity_i_mps2.allFinite()) {
        return false;
    }

    m_w_ib_b_radps = exact_first_order_step(m_w_ib_b_radps,
                                            command.w_command_ib_b_radps,
                                            m_config.vehicle_rate_time_constant_pqr_s,
                                            dt_s);
    m_specific_force_command_response_ib_b_mps2 =
        exact_first_order_step(m_specific_force_command_response_ib_b_mps2,
                               command.specific_force_command_ib_b_mps2,
                               m_config.specific_force_command_time_constant_b_s,
                               dt_s);
    m_specific_force_ib_b_mps2 =
        exact_first_order_step(m_specific_force_ib_b_mps2,
                               m_specific_force_command_response_ib_b_mps2,
                               m_config.specific_force_response_time_constant_b_s,
                               dt_s);

    if (m_config.angular_rate_limits_enabled) {
        m_w_ib_b_radps = clamp_symmetric(
            m_w_ib_b_radps, m_config.angular_rate_limit_pqr_radps, output.angular_rate_limited);
    }
    if (m_config.specific_force_limits_enabled) {
        m_specific_force_ib_b_mps2 = clamp_symmetric(m_specific_force_ib_b_mps2,
                                                     m_config.specific_force_limit_b_mps2,
                                                     output.specific_force_limited);
    }

    output.w_ib_b_radps = m_w_ib_b_radps;
    output.specific_force_command_response_ib_b_mps2 = m_specific_force_command_response_ib_b_mps2;
    output.specific_force_ib_b_mps2 = m_specific_force_ib_b_mps2;
    output.a_i_mps2 = (state.q_b2i * m_specific_force_ib_b_mps2) + environment.gravity_i_mps2;
    return output.a_i_mps2.allFinite();
}

std::unique_ptr<VehicleResponseModel>
make_vehicle_response_model(const VehicleResponseModelType type,
                            const FirstOrderVehicleResponseConfig& first_order_config)
{
    switch (type) {
    case VehicleResponseModelType::FirstOrder:
        return std::make_unique<FirstOrderVehicleResponseModel>(first_order_config);
    }
    return {};
}

} // namespace navkit::sim
