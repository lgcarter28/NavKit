// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"

#include <nlohmann/json.hpp>
#include <ostream>
#include <string>

namespace navkit::app_support
{

template<typename Config>
[[nodiscard]] nlohmann::json profile_compiletime_metadata()
{
    if constexpr (has_profile_export_v<Config>) {
        using Clock = typename Config::Profiling::ProfileClock;
        using Sink = typename Config::Profiling::ProfileSink;

        return {{"enabled", true},
                {"clock_source", std::string(Clock::source)},
                {"tick_period_us", Clock::tick_period_us},
                {"sink_capacity", Sink::capacity()},
                {"sink_overflow_policy", std::string(profile_overflow_policy_name<Sink>())},
                {"profile_points", io::profile_point_mapping()}};
    }
    else {
        return {{"enabled", false}};
    }
}

template<typename Config>
[[nodiscard]] nlohmann::json compiletime_config_metadata()
{
    return {{"schema", "navkit.compiletime_config_metadata.v1"},
            {"profiling", profile_compiletime_metadata<NavKitConfig_t<Config>>()}};
}

template<typename Config>
int describe_compiletime_config(std::ostream& output)
{
    output << compiletime_config_metadata<Config>().dump(2) << '\n';
    return 0;
}

} // namespace navkit::app_support
