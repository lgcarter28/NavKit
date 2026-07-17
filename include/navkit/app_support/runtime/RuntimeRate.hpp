// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/config/Types.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::app_support
{

inline void validate_runtime_rate(const nlohmann::json& cfg, std::string_view object_name)
{
    detail::require_optional_positive_number(cfg, "dt_s");
    detail::require_optional_positive_number(cfg, "rate_hz");
    if (cfg.contains("dt_s") && cfg.contains("rate_hz")) {
        detail::throw_runtime_config_error(std::string{object_name} +
                                           " must specify only one of 'dt_s' or 'rate_hz'");
    }
}

[[nodiscard]] inline core::Time_t dt_s_from_required_runtime_rate(const nlohmann::json& cfg,
                                                                  std::string_view object_name)
{
    validate_runtime_rate(cfg, object_name);
    if (cfg.contains("dt_s")) {
        return cfg.at("dt_s").get<core::Time_t>();
    }
    if (cfg.contains("rate_hz")) {
        return 1.0 / cfg.at("rate_hz").get<core::Time_t>();
    }
    detail::throw_runtime_config_error(std::string{object_name} +
                                       " must specify one of 'dt_s' or 'rate_hz'");
}

} // namespace navkit::app_support
