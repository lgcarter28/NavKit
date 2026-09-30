// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/MissionAppConfigPolicy.hpp"
#include "navkit/app_support/initialization/CovarianceFloorJson.hpp"
#include "navkit/app_support/initialization/InitialCovarianceJson.hpp"
#include "navkit/app_support/initialization/InitialEstimateErrorJson.hpp"
#include "navkit/app_support/initialization/NominalStateOverrideJson.hpp"
#include "navkit/app_support/mission/MissionRuntimeJson.hpp"
#include "navkit/app_support/runtime/PropagationRuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support
{

namespace detail
{

inline void validate_filter_initialization_runtime_config_shape(const nlohmann::json& cfg)
{
    const nlohmann::json::const_iterator filter_initialization_iter =
        cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end()) {
        return;
    }
    if (!filter_initialization_iter->is_object()) {
        throw_runtime_config_error("expected 'filter_initialization' to be an object");
    }

    const std::vector<std::string_view> allowed_filter_initialization_keys{
        "initial_covariance", "covariance_floor", "nominal_state", "initial_estimate_error"};
    for (nlohmann::json::const_iterator iter = filter_initialization_iter->begin();
         iter != filter_initialization_iter->end();
         ++iter) {
        const std::string& key = iter.key();
        if (!contains_key(allowed_filter_initialization_keys, key)) {
            throw_runtime_config_error("unknown key '" + key + "' in 'filter_initialization'");
        }
    }
}

} // namespace detail

/**
 * \brief Validates runtime configuration shared by every mission execution target.
 *
 * \details The common boundary validates host output/logging, the authoritative mission graph,
 * and filter/propagation initialization. It deliberately does not reject root keys or inspect
 * simulation, emulation, hardware, transport, or target-specific initialization fields; each
 * adapter factory owns those parts of its schema.
 */
template<MissionAppConfigPolicy Config>
void validate_common_mission_runtime_config(const nlohmann::json& cfg)
{
    if (!cfg.is_object()) {
        detail::throw_runtime_config_error("root input must be a JSON object");
    }

    detail::require_string(cfg, "run_name");
    detail::require_string(cfg, "output_dir");
    validate_logging_runtime_config(cfg);
    static_cast<void>(mission_runtime_from_json(cfg));

    detail::validate_filter_initialization_runtime_config_shape(cfg);
    detail::validate_runtime_initial_covariance_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_covariance_floor_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_nominal_state_override_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_initial_estimate_error_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_propagation_config_shape<typename Config::NavKit::Propagation>(cfg);
}

} // namespace navkit::app_support
