// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntimeKeys.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntimeConfig.hpp"
#include "navkit/app_support/initialization/CovarianceFloorJson.hpp"
#include "navkit/app_support/initialization/InitialCovarianceJson.hpp"
#include "navkit/app_support/initialization/InitialEstimateErrorJson.hpp"
#include "navkit/app_support/initialization/NominalStateOverrideJson.hpp"
#include "navkit/app_support/runtime/PropagationRuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
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

template<typename EmulatorBindings, std::size_t... Is>
void validate_emulator_runtime_config(const nlohmann::json& cfg, std::index_sequence<Is...>)
{
    (std::tuple_element_t<Is, EmulatorBindings>::Emulator_t::validate_runtime_config(cfg), ...);
}

template<typename EmulatorBindings, std::size_t... Is>
void validate_application_rate_for_emulators(const core::RationalRate& application_rate,
                                             const nlohmann::json& cfg,
                                             std::index_sequence<Is...>)
{
    const bool all_aligned =
        (core::rational_rate_is_integer_multiple(
             application_rate,
             std::tuple_element_t<Is, EmulatorBindings>::Emulator_t::runtime_rate_from_json(cfg)) &&
         ...);
    if (!all_aligned) {
        throw_runtime_config_error(
            "application rate must be an integer multiple of every emulator rate");
    }
}

inline void validate_generated_trajectory_common(const nlohmann::json& trajectory,
                                                 const bool allow_initial_velocity,
                                                 const bool allow_angular_rate)
{
    detail::require_positive_number(trajectory, "duration_s");
    validate_runtime_rate(trajectory, "trajectory");
    if (!trajectory.contains("dt_s") && !trajectory.contains("rate_hz")) {
        detail::throw_runtime_config_error("trajectory must specify one of 'dt_s' or 'rate_hz'");
    }
    detail::require_optional_vec3(trajectory, "p_e_m");
    detail::require_optional_vec3(trajectory, "p_lla_deg_m");
    detail::require_optional_vec3(trajectory, "v_e_mps");
    detail::require_optional_vec3(trajectory, "v_n_mps");
    detail::require_optional_numeric_array(trajectory, "q_b2e", 4U);
    detail::require_optional_vec3(trajectory, "rpy_b2e_rad");
    detail::require_optional_numeric_array(trajectory, "dcm_b2e", 9U);
    detail::require_optional_numeric_array(trajectory, "q_b2n", 4U);
    detail::require_optional_vec3(trajectory, "rpy_b2n_rad");
    detail::require_optional_numeric_array(trajectory, "dcm_b2n", 9U);
    detail::require_optional_vec3(trajectory, "w_ib_b_radps");
    detail::require_optional_vec3(trajectory, "w_eb_b_radps");
    detail::require_optional_vec3(trajectory, "w_nb_b_radps");

    const int position_count =
        (trajectory.contains("p_e_m") ? 1 : 0) + (trajectory.contains("p_lla_deg_m") ? 1 : 0);
    if (position_count != 1) {
        detail::throw_runtime_config_error(
            "trajectory must specify exactly one of 'p_e_m' or 'p_lla_deg_m'");
    }
    if (trajectory.contains("v_e_mps") && trajectory.contains("v_n_mps")) {
        detail::throw_runtime_config_error(
            "trajectory must specify only one of 'v_e_mps' or 'v_n_mps'");
    }
    if (!allow_initial_velocity &&
        (trajectory.contains("v_e_mps") || trajectory.contains("v_n_mps"))) {
        detail::throw_runtime_config_error(
            "this generated trajectory derives velocity from its profile; do not specify "
            "'v_e_mps' or 'v_n_mps'");
    }
    const int attitude_count =
        (trajectory.contains("q_b2e") ? 1 : 0) + (trajectory.contains("rpy_b2e_rad") ? 1 : 0) +
        (trajectory.contains("dcm_b2e") ? 1 : 0) + (trajectory.contains("q_b2n") ? 1 : 0) +
        (trajectory.contains("rpy_b2n_rad") ? 1 : 0) + (trajectory.contains("dcm_b2n") ? 1 : 0);
    if (attitude_count > 1) {
        detail::throw_runtime_config_error(
            "trajectory must specify at most one attitude convention");
    }
    const int angular_rate_count = (trajectory.contains("w_ib_b_radps") ? 1 : 0) +
                                   (trajectory.contains("w_eb_b_radps") ? 1 : 0) +
                                   (trajectory.contains("w_nb_b_radps") ? 1 : 0);
    if (angular_rate_count > 1) {
        detail::throw_runtime_config_error(
            "trajectory must specify at most one angular-rate convention");
    }
    if (!allow_angular_rate && angular_rate_count != 0) {
        detail::throw_runtime_config_error(
            "this generated trajectory derives 'w_ib_b_radps'; do not specify an angular-rate "
            "convention");
    }
}

inline void validate_waypoints_lla_deg_m(const nlohmann::json& trajectory)
{
    const nlohmann::json::const_iterator waypoints = trajectory.find("waypoints_lla_deg_m");
    if (waypoints == trajectory.end() || !waypoints->is_array() || waypoints->empty()) {
        detail::throw_runtime_config_error(
            "waypoint trajectory must specify a nonempty 'waypoints_lla_deg_m' array");
    }
    for (const nlohmann::json& waypoint : *waypoints) {
        if (!waypoint.is_array() || waypoint.size() != 3U) {
            detail::throw_runtime_config_error(
                "each waypoint must have exactly three numeric lla_deg_m entries");
        }
        for (const nlohmann::json& value : waypoint) {
            if (!value.is_number()) {
                detail::throw_runtime_config_error("waypoint entries must be numeric");
            }
        }
    }
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
    allowed_keys.push_back("logging");
    allowed_keys.push_back("application");
    allowed_keys.push_back("trajectory");
    allowed_keys.push_back("imu");
    allowed_keys.push_back("pva_initialization");
    allowed_keys.push_back("filter_initialization");
    allowed_keys.push_back("propagation");
    allowed_keys.push_back("transfer_alignment");
    detail::reject_unknown_top_level_keys(cfg, allowed_keys);

    detail::require_string(cfg, "run_name");
    detail::require_string(cfg, "output_dir");
    validate_logging_runtime_config(cfg);
    const nlohmann::json& application = detail::require_object(cfg, "application");
    detail::reject_unknown_top_level_keys(
        application, std::vector<std::string_view>{"clock", "dt_s", "rate_hz"});
    validate_runtime_rate(application, "application");
    if (!application.contains("dt_s") && !application.contains("rate_hz")) {
        detail::throw_runtime_config_error("application must specify one of 'dt_s' or 'rate_hz'");
    }
    const core::RationalRate application_rate =
        rational_rate_from_required_runtime_rate(application, "application");
    detail::require_string(application, "clock");
    ClockMode clock_mode{};
    if (!detail::clock_mode_from_json(application, "clock", clock_mode)) {
        detail::throw_runtime_config_error("application.clock must be 'simulated' or 'realtime'");
    }

    const auto& trajectory = detail::require_object(cfg, "trajectory");
    detail::require_optional_string(trajectory, "type");
    const std::string trajectory_type = trajectory.value("type", "stationary");
    if (trajectory_type == "csv") {
        detail::require_string(trajectory, "csv_path");
    }
    else if (trajectory_type == "stationary" || trajectory_type == "ballistic" ||
             trajectory_type == "constant_altitude" || trajectory_type == "calibration" ||
             trajectory_type == "waypoint") {
        const bool is_stationary = trajectory_type == "stationary";
        const bool is_ballistic = trajectory_type == "ballistic";
        detail::validate_generated_trajectory_common(
            trajectory, is_stationary || is_ballistic, is_stationary);
        if (trajectory_type == "ballistic") {
            detail::require_optional_nonnegative_number(trajectory, "launch_pad_duration_s");
            detail::require_positive_number(trajectory, "boost_duration_s");
            detail::require_positive_number(trajectory, "boost_acceleration_b_x_mps2");
        }
        else if (trajectory_type == "constant_altitude") {
            detail::require_positive_number(trajectory, "speed_mps");
        }
        else if (trajectory_type == "calibration") {
            detail::require_string(trajectory, "maneuver");
            const std::string maneuver = trajectory.at("maneuver").get<std::string>();
            if (maneuver != "horizontal_s_turn" && maneuver != "vertical_s_turn" &&
                maneuver != "bank_left_right") {
                detail::throw_runtime_config_error(
                    "calibration trajectory maneuver must be 'horizontal_s_turn', "
                    "'vertical_s_turn', or 'bank_left_right'");
            }
            detail::require_positive_number(trajectory, "speed_mps");
            detail::require_positive_number(trajectory, "amplitude_rad");
            detail::require_positive_number(trajectory, "period_s");
        }
        else if (trajectory_type == "waypoint") {
            detail::validate_waypoints_lla_deg_m(trajectory);
            detail::require_positive_number(trajectory, "speed_mps");
            detail::require_positive_number(trajectory, "bank_limit_rad");
            detail::require_positive_number(trajectory, "acceptance_radius_m");
        }
    }
    else {
        detail::throw_runtime_config_error(
            "trajectory.type must be 'stationary', 'csv', 'ballistic', 'constant_altitude', "
            "'calibration', or 'waypoint'");
    }

    detail::validate_emulator_runtime_config<EmulatorBindings>(
        cfg, std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    validate_imu_runtime_config(cfg);
    const core::RationalRate imu_rate =
        rational_rate_from_required_runtime_rate(cfg.at("imu"), "imu");
    if (!core::rational_rate_is_integer_multiple(application_rate, imu_rate)) {
        detail::throw_runtime_config_error(
            "application rate must be an integer multiple of the IMU rate");
    }
    detail::validate_application_rate_for_emulators<EmulatorBindings>(
        application_rate, cfg, std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});

    NavInitializationProvider::validate_runtime_config(cfg);
    detail::validate_filter_initialization_runtime_config_shape(cfg);
    detail::validate_runtime_initial_covariance_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_covariance_floor_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_nominal_state_override_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_initial_estimate_error_shape<typename Config::NavKit::StateDef>(cfg);
    detail::validate_runtime_propagation_config_shape<typename Config::NavKit::Propagation>(cfg);
    TransferAlignmentProvider::validate_runtime_config(cfg);
}

} // namespace navkit::app_support
