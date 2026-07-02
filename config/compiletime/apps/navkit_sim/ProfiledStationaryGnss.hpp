// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/ProfiledGnss.hpp"
#include "navkit/app_support/StationaryGnssApp.hpp"

namespace navkit::config::apps::navkit_sim
{

struct ProfiledStationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::ProfiledGnssConfig;
    using App = ::navkit::app_support::StationaryGnssApp<ProfiledStationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::ProfiledStationaryGnssConfig;

} // namespace navkit::config
