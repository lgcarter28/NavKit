// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntimeKeys.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <tuple>
#include <utility>

namespace navkit::app_support
{

namespace detail
{

template<typename EmulatorBindings, std::size_t... Is>
void validate_emulator_runtime_config(const nlohmann::json& cfg, std::index_sequence<Is...>)
{
    (std::tuple_element_t<Is, EmulatorBindings>::Emulator_t::validate_runtime_config(cfg), ...);
}

} // namespace detail

template<SimulationAppConfigPolicy Config>
void validate_runtime_config(const nlohmann::json& cfg)
{
    using EmulatorBindings = typename Config::EmulatorBindings;
    using NavInitializationProvider = typename Config::NavInitializationProvider;
    using TransferAlignmentProvider = typename Config::TransferAlignmentProvider;

    if (!cfg.is_object()) {
        detail::throw_runtime_config_error("root input must be a JSON object");
    }

    auto allowed_keys = EmulatorRuntimeKeys<EmulatorBindings>::values();
    allowed_keys.push_back("run_name");
    allowed_keys.push_back("output_dir");
    allowed_keys.push_back("trajectory");
    allowed_keys.push_back("initialization");
    allowed_keys.push_back("transfer_alignment");
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

    detail::validate_emulator_runtime_config<EmulatorBindings>(
        cfg, std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});

    NavInitializationProvider::validate_runtime_config(cfg);
    TransferAlignmentProvider::validate_runtime_config(cfg);
}

} // namespace navkit::app_support
