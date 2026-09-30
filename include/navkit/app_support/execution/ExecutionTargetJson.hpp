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
 * fail-closed: `swil` and `hwil` are recognized target selections, while adapter
 * availability is validated separately before any target-specific schema is parsed.
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
    ExecutionTargetType target_type{};
    if (type == "swil") {
        target_type = ExecutionTargetType::Swil;
    }
    else if (type == "hwil") {
        target_type = ExecutionTargetType::Hwil;
    }
    else {
        detail::throw_runtime_config_error("execution_target.type must be 'swil' or 'hwil'");
    }

    detail::require_string(execution_target, "clock");
    ClockMode clock_mode{};
    if (!detail::clock_mode_from_json(execution_target, "clock", clock_mode)) {
        detail::throw_runtime_config_error(
            "execution_target.clock must be 'simulated' or 'realtime'");
    }

    return {.type = target_type,
            .clock_mode = clock_mode,
            .application_rate =
                rational_rate_from_required_runtime_rate(execution_target, "execution_target")};
}

/**
 * \brief Returns the stable runtime spelling for an execution-target type.
 *
 * \details This is used in diagnostics that compare an executable's fixed adapter target
 * with the target requested by a runtime scenario.
 */
[[nodiscard]] constexpr std::string_view execution_target_type_name(const ExecutionTargetType type)
{
    switch (type) {
    case ExecutionTargetType::Swil:
        return "swil";
    case ExecutionTargetType::Hwil:
        return "hwil";
    }
    return "unknown";
}

/**
 * \brief Rejects a runtime scenario that targets a different application executable.
 *
 * \details The check intentionally runs before target-specific schema validation so a HWIL
 * file passed to `navkit_swil`, for example, fails with one precise target-mismatch diagnostic
 * instead of a misleading missing-simulation-field error.
 */
inline void require_execution_target_type(const ExecutionTargetSettings& settings,
                                          const ExecutionTargetType required_type,
                                          const std::string_view application_name)
{
    if (settings.type == required_type) {
        return;
    }
    detail::throw_runtime_config_error(
        std::string{application_name} + " requires execution_target.type '" +
        std::string{execution_target_type_name(required_type)} + "'; received '" +
        std::string{execution_target_type_name(settings.type)} + "'");
}

} // namespace navkit::app_support
