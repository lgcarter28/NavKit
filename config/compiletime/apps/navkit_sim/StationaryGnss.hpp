// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/StationaryGnss.hpp"
#include "navkit/app_support/StationaryGnssApp.hpp"

namespace navkit::config::apps::navkit_sim
{

struct StationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::StationaryGnssConfig;
    using App = ::navkit::app_support::StationaryGnssApp<StationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::StationaryGnssConfig;

} // namespace navkit::config
