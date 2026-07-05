// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/app_support/GnssEmulator.hpp"
#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/products/ProfiledStationaryGnss.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct ProfiledStationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::ProfiledStationaryGnssConfig;

    static constexpr ::navkit::app_support::SensorId primary_gnss_sensor_id =
        NavKit::primary_gnss_sensor_id;
    using EmulatorBindings =
        std::tuple<::navkit::app_support::EmulatorBinding<primary_gnss_sensor_id,
                                                          ::navkit::app_support::GnssEmulator,
                                                          NavKit::PrimaryGnssSensor>>;

    using App = ::navkit::app_support::SimulationApp<ProfiledStationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::ProfiledStationaryGnssConfig;

} // namespace navkit::config
