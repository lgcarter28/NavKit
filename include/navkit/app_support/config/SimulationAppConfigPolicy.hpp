// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/io/LoggerPolicy.hpp"

namespace navkit::app_support
{

template<typename Config>
concept SimulationAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
        typename Config::Logger;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    navkit::io::LoggerPolicy<LoggerConfig_t<Config>> &&
    EmulatorBindingTuplePolicy<typename Config::EmulatorBindings,
                               typename Config::NavKit::Sensors,
                               LoggerConfig_t<Config>>;

} // namespace navkit::app_support
