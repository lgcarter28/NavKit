// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/app_support/initialization/NavInitializationProviderPolicy.hpp"
#include "navkit/app_support/initialization/TransferAlignmentProviderPolicy.hpp"
#include "navkit/app_support/logging/RuntimeLogger.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/sim/sensors/ImuSimulatorPolicy.hpp"

namespace navkit::app_support
{

template<typename Config>
concept SimulationAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
        typename Config::ImuSimulator;
        typename Config::NavInitializationProvider;
        typename Config::TransferAlignmentProvider;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    navkit::sim::ImuSimulatorPolicy<typename Config::ImuSimulator> &&
    navkit::io::LoggerPolicy<RuntimeLogger<typename Config::NavKit>> &&
    EmulatorBindingTuplePolicy<typename Config::EmulatorBindings,
                               typename Config::NavKit::Sensors,
                               RuntimeLogger<typename Config::NavKit>> &&
    NavInitializationProviderPolicy<typename Config::NavInitializationProvider> &&
    TransferAlignmentProviderPolicy<typename Config::TransferAlignmentProvider,
                                    typename Config::NavKit::Navigator>;

} // namespace navkit::app_support
