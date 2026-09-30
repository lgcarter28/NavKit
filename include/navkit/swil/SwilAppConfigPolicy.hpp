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

#include <tuple>

namespace navkit::swil
{

/**
 * \brief Compile-time product-composition contract for the SWIL adapter.
 *
 * \details In addition to simulator and initialization capabilities, a pure SWIL product must
 * bind every configured aiding sensor to exactly one emulator. Binding validity and unique IDs
 * prevent duplicate producers; equal tuple sizes make that mapping complete. The IMU producer is
 * represented separately by `ImuSimulator`.
 */
template<typename Config>
concept SwilAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
        typename Config::ImuSimulator;
        typename Config::NavInitializationProvider;
        typename Config::TransferAlignmentProvider;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    navkit::sim::ImuSimulatorPolicy<typename Config::ImuSimulator> &&
    navkit::io::LoggerPolicy<app_support::RuntimeLogger<typename Config::NavKit>> &&
    app_support::EmulatorBindingTuplePolicy<typename Config::EmulatorBindings,
                                            typename Config::NavKit::Sensors,
                                            app_support::RuntimeLogger<typename Config::NavKit>> &&
    (std::tuple_size_v<typename Config::EmulatorBindings> ==
     std::tuple_size_v<typename Config::NavKit::Sensors>) &&
    app_support::NavInitializationProviderPolicy<typename Config::NavInitializationProvider> &&
    app_support::TransferAlignmentProviderPolicy<typename Config::TransferAlignmentProvider,
                                                 typename Config::NavKit::Navigator>;

} // namespace navkit::swil
