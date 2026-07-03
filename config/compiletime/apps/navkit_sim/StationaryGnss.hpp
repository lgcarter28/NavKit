// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/StationaryGnss.hpp"
#include "navkit/app_support/GnssEmulator.hpp"
#include "navkit/app_support/SensorId.hpp"
#include "navkit/app_support/SimulationApp.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct StationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::StationaryGnssConfig;

    static constexpr ::navkit::app_support::SensorId PrimaryGnssSensorId =
        NavKit::PrimaryGnssSensorId;
    using EmulatorBindings =
        std::tuple<::navkit::app_support::EmulatorBinding<PrimaryGnssSensorId,
                                                          ::navkit::app_support::GnssEmulator,
                                                          NavKit::PrimaryGnssSensor>>;

    using App = ::navkit::app_support::SimulationApp<StationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::StationaryGnssConfig;

} // namespace navkit::config
