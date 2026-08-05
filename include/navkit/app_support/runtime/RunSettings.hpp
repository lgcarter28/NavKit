// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/app_support/time/ClockMode.hpp"
#include "navkit/app_support/trajectory/ControlStateSourceMode.hpp"
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
    bool console_enabled{false};
    bool truth_enabled{false};
    bool nav_enabled{false};
    bool measurement_statistics_enabled{false};
    bool imu_enabled{false};
    bool imu_debug_enabled{false};
    bool filter_correction_enabled{false};
    bool trajectory_kinematics_ecef_enabled{false};
    bool trajectory_kinematics_eci_enabled{false};
    bool trajectory_kinematics_ned_enabled{false};
    bool trajectory_kinematics_body_enabled{false};
    bool trajectory_guidance_enabled{false};
    bool trajectory_autopilot_vehicle_enabled{false};

    core::RationalRate console_rate{};
    core::RationalRate truth_rate{};
    core::RationalRate nav_rate{};
    core::RationalRate measurement_statistics_rate{};
    core::RationalRate imu_rate{};
    core::RationalRate imu_debug_rate{};
    core::RationalRate filter_correction_rate{};
    core::RationalRate trajectory_kinematics_ecef_rate{};
    core::RationalRate trajectory_kinematics_eci_rate{};
    core::RationalRate trajectory_kinematics_ned_rate{};
    core::RationalRate trajectory_kinematics_body_rate{};
    core::RationalRate trajectory_guidance_rate{};
    core::RationalRate trajectory_autopilot_vehicle_rate{};
};

struct RunSettings
{
    std::string run_name;
    std::filesystem::path output_dir;
    std::filesystem::path data_dir;
    std::filesystem::path figures_dir;
    core::RationalRate application_rate{};
    ClockMode clock_mode{ClockMode::Simulated};
    ControlStateSourceMode control_state_source{ControlStateSourceMode::NavigationEstimate};
    LoggingSchedule logging;
};

inline void validate_logging_runtime_config(const nlohmann::json& cfg)
{
    const auto& logging = detail::require_object(cfg, "logging");
    detail::reject_unknown_top_level_keys(
        logging,
        std::vector<std::string_view>{"console",
                                      "truth",
                                      "measurement_statistics",
                                      "imu",
                                      "imu_debug",
                                      "filter_correction",
                                      "nav_estimate",
                                      "trajectory_kinematics_ecef",
                                      "trajectory_kinematics_eci",
                                      "trajectory_kinematics_ned",
                                      "trajectory_kinematics_body",
                                      "trajectory_guidance",
                                      "trajectory_autopilot_vehicle"});

    for (const auto key : {"console",
                           "truth",
                           "nav_estimate",
                           "measurement_statistics",
                           "imu",
                           "imu_debug",
                           "filter_correction"}) {
        const auto& logging_object = detail::require_object(logging, key);
        detail::require_bool(logging_object, "enabled");
        validate_runtime_rate(logging_object, key);
        const bool enabled = logging_object.at("enabled").get<bool>();
        if (enabled && !logging_object.contains("dt_s") && !logging_object.contains("rate_hz")) {
            detail::throw_runtime_config_error(
                std::string{key} + " logging is enabled but has no 'dt_s' or 'rate_hz'");
        }
        if (std::string_view{key} == "filter_correction" ||
            std::string_view{key} == "nav_estimate") {
            detail::require_string(logging_object, "covariance");
            const std::string mode = logging_object.at("covariance").get<std::string>();
            if (mode != "diagonal" && mode != "triangular") {
                detail::throw_runtime_config_error(
                    std::string{key} + ".covariance must be 'diagonal' or 'triangular'");
            }
        }
    }

    for (const char* key : {"trajectory_kinematics_ecef",
                            "trajectory_kinematics_eci",
                            "trajectory_kinematics_ned",
                            "trajectory_kinematics_body",
                            "trajectory_guidance",
                            "trajectory_autopilot_vehicle"}) {
        if (!logging.contains(key)) {
            continue;
        }
        const nlohmann::json& logging_object = detail::require_object(logging, key);
        detail::require_bool(logging_object, "enabled");
        validate_runtime_rate(logging_object, key);
        const bool enabled = logging_object.at("enabled").get<bool>();
        if (enabled && !logging_object.contains("dt_s") && !logging_object.contains("rate_hz")) {
            detail::throw_runtime_config_error(
                std::string{key} + " logging is enabled but has no 'dt_s' or 'rate_hz'");
        }
    }
}

[[nodiscard]] inline bool logging_enabled_from_json(const nlohmann::json& logging, const char* key)
{
    return detail::require_object(logging, key).at("enabled").get<bool>();
}

[[nodiscard]] inline core::RationalRate logging_rate_from_json(const nlohmann::json& logging,
                                                               const char* key)
{
    const auto& logging_object = detail::require_object(logging, key);
    if (!logging_object.at("enabled").get<bool>()) {
        return {};
    }
    return rational_rate_from_required_runtime_rate(logging_object, key);
}

[[nodiscard]] inline bool optional_logging_enabled_from_json(const nlohmann::json& logging,
                                                             const char* key)
{
    return logging.contains(key) && logging_enabled_from_json(logging, key);
}

[[nodiscard]] inline core::RationalRate
optional_logging_rate_from_json(const nlohmann::json& logging, const char* key)
{
    if (!logging.contains(key)) {
        return {};
    }
    return logging_rate_from_json(logging, key);
}

inline RunSettings run_settings_from_json(const nlohmann::json& cfg)
{
    const std::string run_name = cfg.at("run_name").get<std::string>();
    const std::filesystem::path output_dir = cfg.at("output_dir").get<std::string>();
    const std::filesystem::path data_dir = output_dir / "data";
    const std::filesystem::path figures_dir = output_dir / "figures";
    const nlohmann::json& application = detail::require_object(cfg, "application");
    const core::RationalRate application_rate =
        rational_rate_from_required_runtime_rate(application, "application");
    ClockMode clock_mode{};
    if (!detail::clock_mode_from_json(application, "clock", clock_mode)) {
        detail::throw_runtime_config_error("application.clock must be 'simulated' or 'realtime'");
    }
    ControlStateSourceMode control_state_source{};
    if (!control_state_source_mode_from_string(
            application.at("control_state_source").get<std::string>(), control_state_source)) {
        detail::throw_runtime_config_error(
            "application.control_state_source must be 'navigation_estimate' or "
            "'truth_passthrough'");
    }

    LoggingSchedule logging{};
    const auto& logging_json = detail::require_object(cfg, "logging");
    logging.console_enabled = logging_enabled_from_json(logging_json, "console");
    logging.truth_enabled = logging_enabled_from_json(logging_json, "truth");
    logging.nav_enabled = logging_enabled_from_json(logging_json, "nav_estimate");
    logging.measurement_statistics_enabled =
        logging_enabled_from_json(logging_json, "measurement_statistics");
    logging.imu_enabled = logging_enabled_from_json(logging_json, "imu");
    logging.imu_debug_enabled = logging_enabled_from_json(logging_json, "imu_debug");
    logging.filter_correction_enabled =
        logging_enabled_from_json(logging_json, "filter_correction");
    logging.trajectory_kinematics_ecef_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_kinematics_ecef");
    logging.trajectory_kinematics_eci_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_kinematics_eci");
    logging.trajectory_kinematics_ned_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_kinematics_ned");
    logging.trajectory_kinematics_body_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_kinematics_body");
    logging.trajectory_guidance_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_guidance");
    logging.trajectory_autopilot_vehicle_enabled =
        optional_logging_enabled_from_json(logging_json, "trajectory_autopilot_vehicle");

    logging.console_rate = logging_rate_from_json(logging_json, "console");
    logging.truth_rate = logging_rate_from_json(logging_json, "truth");
    logging.nav_rate = logging_rate_from_json(logging_json, "nav_estimate");
    logging.measurement_statistics_rate =
        logging_rate_from_json(logging_json, "measurement_statistics");
    logging.imu_rate = logging_rate_from_json(logging_json, "imu");
    logging.imu_debug_rate = logging_rate_from_json(logging_json, "imu_debug");
    logging.filter_correction_rate = logging_rate_from_json(logging_json, "filter_correction");
    logging.trajectory_kinematics_ecef_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_kinematics_ecef");
    logging.trajectory_kinematics_eci_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_kinematics_eci");
    logging.trajectory_kinematics_ned_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_kinematics_ned");
    logging.trajectory_kinematics_body_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_kinematics_body");
    logging.trajectory_guidance_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_guidance");
    logging.trajectory_autopilot_vehicle_rate =
        optional_logging_rate_from_json(logging_json, "trajectory_autopilot_vehicle");

    return {.run_name = run_name,
            .output_dir = output_dir,
            .data_dir = data_dir,
            .figures_dir = figures_dir,
            .application_rate = application_rate,
            .clock_mode = clock_mode,
            .control_state_source = control_state_source,
            .logging = logging};
}

} // namespace navkit::app_support
