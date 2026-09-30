// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigPolicy.hpp"
#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/logging/RuntimeLogger.hpp"
#include "navkit/io/LoggerPolicy.hpp"

namespace navkit::app_support
{

/**
 * \brief Compile-time configuration contract required by the target-neutral mission host.
 *
 * \details `MissionApp` depends only on the selected NavKit product and its common runtime
 * logger. Target-specific requirements such as synthetic sensor emulators belong to the
 * adapter factory selected by the thin application executable.
 */
template<typename Config>
concept MissionAppConfigPolicy =
    navkit::api::config::NavKitProductConfigPolicy<NavKitConfig_t<Config>> &&
    navkit::io::LoggerPolicy<RuntimeLogger<NavKitConfig_t<Config>>>;

} // namespace navkit::app_support
