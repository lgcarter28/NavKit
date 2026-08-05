// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/sim/guidance/GuidanceCommand.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"

namespace navkit::sim
{

/**
 * Simulation-only source-agnostic Guidance boundary.
 *
 * Implementations receive a kinematic estimate chosen by application wiring.
 * They never know whether it came from truth, Navigator, or external hardware.
 */
class GuidanceModel
{
public:
    virtual ~GuidanceModel() = default;

    [[nodiscard]] virtual bool initialize(const TrajectoryControlState& initial_state,
                                          const TrajectoryEnvironment& environment) = 0;

    [[nodiscard]] virtual bool advance(const TrajectoryControlState& state,
                                       const TrajectoryEnvironment& environment,
                                       core::Time_t dt_s,
                                       GuidanceOutput& output) = 0;
};

} // namespace navkit::sim
