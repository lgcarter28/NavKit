// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/ProfiledStationaryGnss.hpp"
#include "navkit/app_support/GnssEmulator.hpp"
#include "navkit/app_support/SensorId.hpp"
#include "navkit/app_support/SimulationApp.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct ProfiledStationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::ProfiledStationaryGnssConfig;

    static constexpr ::navkit::app_support::SensorId PrimaryGnssSensorId =
        NavKit::PrimaryGnssSensorId;
    using EmulatorBindings =
        std::tuple<::navkit::app_support::EmulatorBinding<PrimaryGnssSensorId,
                                                          ::navkit::app_support::GnssEmulator,
                                                          NavKit::PrimaryGnssSensor>>;

    using App = ::navkit::app_support::SimulationApp<ProfiledStationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::ProfiledStationaryGnssConfig;

} // namespace navkit::config
