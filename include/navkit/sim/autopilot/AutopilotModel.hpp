// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/sim/autopilot/AutopilotOutput.hpp"
#include "navkit/sim/autopilot/AutopilotState.hpp"
#include "navkit/sim/autopilot/VehicleCommand.hpp"
#include "navkit/sim/guidance/GuidanceCommand.hpp"

namespace navkit::sim
{

/**
 * Simulation-only source-agnostic attitude/rate tracking boundary.
 *
 * The application feeds actual simulated or hardware IMU increments through
 * observe_imu_increment(); implementations decide how to filter and use them.
 */
class AutopilotModel
{
public:
    virtual ~AutopilotModel() = default;

    [[nodiscard]] virtual bool initialize(const AutopilotState& initial_state) = 0;

    [[nodiscard]] virtual bool
    observe_imu_increment(const core::estimation::ImuIncrement& increment) = 0;

    [[nodiscard]] virtual bool advance(const GuidanceCommand& guidance,
                                       const AutopilotState& state,
                                       const AutopilotExecutionState& execution,
                                       core::Time_t dt_s,
                                       VehicleCommand& command,
                                       AutopilotOutput& output) = 0;
};

} // namespace navkit::sim
