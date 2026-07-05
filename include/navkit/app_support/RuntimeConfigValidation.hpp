// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/EmulatorRuntimeKeys.hpp"
#include "navkit/app_support/RuntimeConfigJson.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <tuple>

namespace navkit::app_support
{

template<typename Config>
void validate_runtime_config(const nlohmann::json& cfg)
{
    using EmulatorBindings = typename Config::EmulatorBindings;

    if (!cfg.is_object()) {
        detail::throw_runtime_config_error("root input must be a JSON object");
    }

    auto allowed_keys = EmulatorRuntimeKeys<EmulatorBindings>::values();
    allowed_keys.push_back("run_name");
    allowed_keys.push_back("output_dir");
    allowed_keys.push_back("trajectory");
    allowed_keys.push_back("filter");
    detail::reject_unknown_top_level_keys(cfg, allowed_keys);

    detail::require_optional_string(cfg, "run_name");
    detail::require_optional_string(cfg, "output_dir");

    const auto& trajectory = detail::require_object(cfg, "trajectory");
    detail::require_optional_string(trajectory, "type");
    if (const auto type_iter = trajectory.find("type");
        type_iter != trajectory.end() && type_iter->get<std::string>() != "stationary") {
        detail::throw_runtime_config_error(
            "trajectory.type must be 'stationary' for the current navkit_sim trajectory provider");
    }
    detail::require_optional_positive_number(trajectory, "duration_s");
    detail::require_optional_positive_number(trajectory, "dt_s");
    detail::require_vec3(trajectory, "p_e_m");

    std::apply(
        [&cfg](auto... binding) {
            ((decltype(binding)::Emulator_t::validate_runtime_config(cfg)), ...);
        },
        EmulatorBindings{});

    if (const auto filter_iter = cfg.find("filter"); filter_iter != cfg.end()) {
        if (!filter_iter->is_object()) {
            detail::throw_runtime_config_error("expected 'filter' to be an object");
        }
        detail::require_optional_vec3(*filter_iter, "initial_position_offset_m");
        detail::require_optional_nonnegative_number(*filter_iter, "initial_position_sigma_m");
    }
}

} // namespace navkit::app_support
