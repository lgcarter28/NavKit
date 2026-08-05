// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/sim/autopilot/VehicleCommand.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"
#include "navkit/sim/trajectory/VehicleResponseOutput.hpp"

namespace navkit::sim
{

/** Simulation-only runtime boundary from command to realized vehicle/plant response. */
class VehicleResponseModel
{
public:
    virtual ~VehicleResponseModel() = default;

    [[nodiscard]] virtual bool initialize(const TrajectoryDynamicState& initial_state) = 0;

    [[nodiscard]] virtual bool advance(const VehicleCommand& command,
                                       const TrajectoryDynamicState& state,
                                       const TrajectoryEnvironment& environment,
                                       core::Time_t dt_s,
                                       VehicleResponseOutput& output) = 0;
};

} // namespace navkit::sim
