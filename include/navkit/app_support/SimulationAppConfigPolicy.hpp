// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/EmulatorBindingTuplePolicy.hpp"

namespace navkit::app_support
{

template<typename Config>
concept SimulationAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    EmulatorBindingTuplePolicy<typename Config::EmulatorBindings, typename Config::NavKit::Sensors>;

} // namespace navkit::app_support
