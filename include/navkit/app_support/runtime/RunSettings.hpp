// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/config/Types.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support
{

struct LoggingSchedule
{
    core::Time_t console_dt_s{1.0};
    core::Time_t truth_dt_s{0.1};
    core::Time_t nav_dt_s{0.1};
    core::Time_t measurement_statistics_dt_s{1.0};
    core::Time_t imu_dt_s{0.1};
    core::Time_t imu_debug_dt_s{0.1};
    core::Time_t filter_correction_dt_s{0.1};
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
    detail::reject_unknown_top_level_keys(logging,
                                          std::vector<std::string_view>{"console",
                                                                        "truth",
                                                                        "nav",
                                                                        "measurement_statistics",
                                                                        "imu",
                                                                        "imu_debug",
                                                                        "filter_correction",
                                                                        "nav_estimate"});

    for (const auto key : {"console",
                           "truth",
                           "nav",
                           "nav_estimate",
                           "measurement_statistics",
                           "imu",
                           "imu_debug",
                           "filter_correction"}) {
        if (logging.contains(key)) {
            const auto& logging_object = detail::require_object(logging, key);
            validate_runtime_rate(logging_object, key);
            if (std::string_view{key} == "filter_correction" ||
                std::string_view{key} == "nav_estimate") {
                detail::require_optional_string(logging_object, "covariance");
                const auto mode = logging_object.value("covariance", std::string("diagonal"));
                if (mode != "diagonal" && mode != "triangular") {
                    detail::throw_runtime_config_error(
                        std::string{key} + ".covariance must be 'diagonal' or 'triangular'");
                }
            }
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
        logging.nav_dt_s = logging_dt_s_from_json(logging_json, "nav_estimate", logging.nav_dt_s);
        logging.measurement_statistics_dt_s = logging_dt_s_from_json(
            logging_json, "measurement_statistics", logging.measurement_statistics_dt_s);
        logging.imu_dt_s = logging_dt_s_from_json(logging_json, "imu", logging.imu_dt_s);
        logging.imu_debug_dt_s =
            logging_dt_s_from_json(logging_json, "imu_debug", logging.imu_debug_dt_s);
        logging.filter_correction_dt_s = logging_dt_s_from_json(
            logging_json, "filter_correction", logging.filter_correction_dt_s);
    }

    return {.run_name = run_name, .output_dir = output_dir, .logging = logging};
}

} // namespace navkit::app_support
