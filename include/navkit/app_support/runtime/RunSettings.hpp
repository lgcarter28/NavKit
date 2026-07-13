// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/config/Types.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace navkit::app_support
{

struct LoggingSchedule
{
    core::Time_t console_dt_s{1.0};
    core::Time_t truth_dt_s{0.1};
    core::Time_t nav_dt_s{0.1};
    core::Time_t measurement_statistics_dt_s{1.0};
};

struct RunSettings
{
    std::string run_name;
    std::filesystem::path output_dir;
    LoggingSchedule logging;
};

inline void validate_logging_runtime_config(const nlohmann::json& cfg)
{
    if (!cfg.contains("logging")) {
        return;
    }

    const auto& logging = detail::require_object(cfg, "logging");
    detail::reject_unknown_top_level_keys(
        logging,
        std::vector<std::string_view>{"console", "truth", "nav", "measurement_statistics"});

    for (const auto key : {"console", "truth", "nav", "measurement_statistics"}) {
        if (logging.contains(key)) {
            validate_runtime_rate(detail::require_object(logging, key), key);
        }
    }
}

[[nodiscard]] inline core::Time_t logging_dt_s_from_json(const nlohmann::json& logging,
                                                         const char* key,
                                                         const core::Time_t default_dt_s)
{
    if (!logging.contains(key)) {
        return default_dt_s;
    }
    return dt_s_from_runtime_rate(detail::require_object(logging, key), default_dt_s);
}

inline RunSettings run_settings_from_json(const nlohmann::json& cfg)
{
    const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
    const std::filesystem::path output_dir =
        cfg.value("output_dir", std::string("output/logs/") + run_name);

    LoggingSchedule logging{};
    if (cfg.contains("logging")) {
        const auto& logging_json = detail::require_object(cfg, "logging");
        logging.console_dt_s =
            logging_dt_s_from_json(logging_json, "console", logging.console_dt_s);
        logging.truth_dt_s = logging_dt_s_from_json(logging_json, "truth", logging.truth_dt_s);
        logging.nav_dt_s = logging_dt_s_from_json(logging_json, "nav", logging.nav_dt_s);
        logging.measurement_statistics_dt_s = logging_dt_s_from_json(
            logging_json, "measurement_statistics", logging.measurement_statistics_dt_s);
    }

    return {.run_name = run_name, .output_dir = output_dir, .logging = logging};
}

} // namespace navkit::app_support
