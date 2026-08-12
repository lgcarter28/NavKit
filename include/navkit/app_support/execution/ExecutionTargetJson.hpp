// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/execution/ExecutionTarget.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support
{

/**
 * \brief Parses and validates the required runtime execution-target component.
 *
 * \details The accepted schema is
 * `execution_target { type, clock, rate_hz | dt_s }`. The target type is deliberately
 * fail-closed: `swil` is the only currently implemented application adapter.
 * Clock and cadence parsing reuse the shared app-support runtime configuration paths.
 */
[[nodiscard]] inline ExecutionTargetSettings
execution_target_settings_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& execution_target = detail::require_object(cfg, "execution_target");
    detail::reject_unknown_top_level_keys(
        execution_target, std::vector<std::string_view>{"type", "clock", "dt_s", "rate_hz"});

    detail::require_string(execution_target, "type");
    const std::string type = execution_target.at("type").get<std::string>();
    if (type != "swil") {
        detail::throw_runtime_config_error(
            "execution_target.type must be 'swil'; no other execution-target "
            "adapter is currently implemented");
    }

    detail::require_string(execution_target, "clock");
    ClockMode clock_mode{};
    if (!detail::clock_mode_from_json(execution_target, "clock", clock_mode)) {
        detail::throw_runtime_config_error(
            "execution_target.clock must be 'simulated' or 'realtime'");
    }

    return {.type = ExecutionTargetType::NavKitSwil,
            .clock_mode = clock_mode,
            .application_rate =
                rational_rate_from_required_runtime_rate(execution_target, "execution_target")};
}

} // namespace navkit::app_support
