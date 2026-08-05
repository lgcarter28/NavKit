// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/trajectory/VehicleResponseModel.hpp"

#include <memory>

namespace navkit::sim
{

enum class VehicleResponseModelType
{
    FirstOrder,
};

/** Runtime configuration for the first-order vehicle/plant response. */
struct FirstOrderVehicleResponseConfig
{
    core::Vec3 vehicle_rate_time_constant_pqr_s{core::Vec3::Zero()};
    core::Vec3 specific_force_command_time_constant_b_s{core::Vec3::Zero()};
    core::Vec3 specific_force_response_time_constant_b_s{core::Vec3::Zero()};
    bool angular_rate_limits_enabled{false};
    core::Vec3 angular_rate_limit_pqr_radps{core::Vec3::Zero()};
    bool specific_force_limits_enabled{false};
    core::Vec3 specific_force_limit_b_mps2{core::Vec3::Zero()};
};

[[nodiscard]] bool
first_order_vehicle_response_config_is_valid(const FirstOrderVehicleResponseConfig& config);

class FirstOrderVehicleResponseModel final : public VehicleResponseModel
{
public:
    explicit FirstOrderVehicleResponseModel(FirstOrderVehicleResponseConfig config);

    [[nodiscard]] bool initialize(const TrajectoryDynamicState& initial_state) override;

    [[nodiscard]] bool advance(const VehicleCommand& command,
                               const TrajectoryDynamicState& state,
                               const TrajectoryEnvironment& environment,
                               core::Time_t dt_s,
                               VehicleResponseOutput& output) override;

private:
    FirstOrderVehicleResponseConfig m_config{};
    core::Vec3 m_w_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 m_specific_force_command_response_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 m_specific_force_ib_b_mps2{core::Vec3::Zero()};
    bool m_initialized{false};
};

/** Creates the selected simulation-only vehicle-response model. */
[[nodiscard]] std::unique_ptr<VehicleResponseModel>
make_vehicle_response_model(VehicleResponseModelType type,
                            const FirstOrderVehicleResponseConfig& first_order_config);

} // namespace navkit::sim
