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
#include "navkit/app_support/trajectory/TrajectoryAttitudeJson.hpp"

#include <cmath>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
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
    detail::require_optional_positive_number(trajectory, "dynamics_rate_hz");
    detail::require_optional_positive_number(trajectory, "dynamics_dt_s");
    if (trajectory.contains("dynamics_rate_hz") == trajectory.contains("dynamics_dt_s")) {
        detail::throw_runtime_config_error(
            "trajectory must specify exactly one of 'dynamics_rate_hz' or 'dynamics_dt_s'");
    }
    detail::require_optional_vec3(trajectory, "p_e_m");
    detail::require_optional_vec3(trajectory, "p_lla_deg_m");
    detail::require_optional_vec3(trajectory, "v_e_mps");
    detail::require_optional_vec3(trajectory, "v_n_mps");
    detail::validate_trajectory_attitude_json(trajectory);
    detail::require_optional_vec3(trajectory, "w_ib_b_degps");
    detail::require_optional_vec3(trajectory, "w_eb_b_degps");
    detail::require_optional_vec3(trajectory, "w_nb_b_degps");

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
    const int angular_rate_count = (trajectory.contains("w_ib_b_degps") ? 1 : 0) +
                                   (trajectory.contains("w_eb_b_degps") ? 1 : 0) +
                                   (trajectory.contains("w_nb_b_degps") ? 1 : 0);
    if (angular_rate_count > 1) {
        detail::throw_runtime_config_error(
            "trajectory must specify at most one angular-rate convention");
    }
    if (!allow_angular_rate && angular_rate_count != 0) {
        detail::throw_runtime_config_error(
            "this generated trajectory derives 'w_ib_b_degps'; do not specify an angular-rate "
            "convention");
    }
}

inline void validate_trajectory_vec3_bounds(const nlohmann::json& object,
                                            const std::string_view key,
                                            const bool strictly_positive)
{
    require_optional_vec3(object, key);
    if (!object.contains(std::string{key})) {
        throw_runtime_config_error("missing required trajectory vector '" + std::string{key} + "'");
    }
    for (const nlohmann::json& value : object.at(std::string{key})) {
        if (!value.is_number() || !std::isfinite(value.get<double>()) ||
            (strictly_positive ? value.get<double>() <= 0.0 : value.get<double>() < 0.0)) {
            throw_runtime_config_error("trajectory vector '" + std::string{key} +
                                       (strictly_positive
                                            ? "' entries must be finite and positive"
                                            : "' entries must be finite and nonnegative"));
        }
    }
}

inline void validate_trajectory_subsystem_rate(const nlohmann::json& trajectory,
                                               const std::string_view subsystem)
{
    const std::string rate_key = std::string{subsystem} + "_rate_hz";
    const std::string period_key = std::string{subsystem} + "_dt_s";
    detail::require_optional_positive_number(trajectory, rate_key);
    detail::require_optional_positive_number(trajectory, period_key);
    if (trajectory.contains(rate_key) == trajectory.contains(period_key)) {
        throw_runtime_config_error("trajectory must specify exactly one of '" + rate_key +
                                   "' or '" + period_key + "'");
    }
}

inline void validate_trajectory_dynamics_config(const nlohmann::json& trajectory)
{
    require_string(trajectory, "translational_integration");
    const std::string integration = trajectory.at("translational_integration").get<std::string>();
    if (integration != "semi_implicit_euler" && integration != "trapezoidal_predictor_corrector") {
        throw_runtime_config_error(
            "trajectory.translational_integration must be 'semi_implicit_euler' or "
            "'trapezoidal_predictor_corrector'");
    }

    validate_trajectory_subsystem_rate(trajectory, "guidance");
    validate_trajectory_subsystem_rate(trajectory, "autopilot");

    const nlohmann::json& guidance_filter = require_object(trajectory, "guidance_command_filter");
    reject_unknown_top_level_keys(guidance_filter,
                                  std::vector<std::string_view>{
                                      "specific_force_time_constant_b_s",
                                      "bank_time_constant_s",
                                  });
    validate_trajectory_vec3_bounds(guidance_filter, "specific_force_time_constant_b_s", false);
    require_optional_nonnegative_number(guidance_filter, "bank_time_constant_s");
    if (!guidance_filter.contains("bank_time_constant_s")) {
        throw_runtime_config_error(
            "missing required trajectory.guidance_command_filter.bank_time_constant_s");
    }
    const nlohmann::json& autopilot = require_object(trajectory, "autopilot");
    reject_unknown_top_level_keys(
        autopilot,
        std::vector<std::string_view>{"type",
                                      "controller_rate_time_constant_pqr_s",
                                      "attitude_command_time_constant_s",
                                      "attitude_error_gain_pqr_per_s",
                                      "angular_rate_feedback_gain_pqr",
                                      "velocity_alignment_speed_threshold_mps",
                                      "initial_velocity_alignment_tolerance_deg",
                                      "gyro_moving_average_window_samples"});
    require_string(autopilot, "type");
    if (autopilot.at("type").get<std::string>() != "first_order") {
        throw_runtime_config_error("trajectory.autopilot.type must be 'first_order'");
    }
    validate_trajectory_vec3_bounds(autopilot, "controller_rate_time_constant_pqr_s", false);
    require_optional_nonnegative_number(autopilot, "attitude_command_time_constant_s");
    validate_trajectory_vec3_bounds(autopilot, "attitude_error_gain_pqr_per_s", false);
    validate_trajectory_vec3_bounds(autopilot, "angular_rate_feedback_gain_pqr", false);
    require_positive_number(autopilot, "velocity_alignment_speed_threshold_mps");
    require_optional_nonnegative_number(autopilot, "initial_velocity_alignment_tolerance_deg");
    if (!autopilot.contains("initial_velocity_alignment_tolerance_deg")) {
        throw_runtime_config_error(
            "missing required trajectory.autopilot.initial_velocity_alignment_tolerance_deg");
    }
    require_unsigned_integer(autopilot, "gyro_moving_average_window_samples");
    if (autopilot.at("gyro_moving_average_window_samples").get<std::size_t>() == 0U) {
        throw_runtime_config_error(
            "trajectory.autopilot.gyro_moving_average_window_samples must be positive");
    }

    const nlohmann::json& vehicle_response = require_object(trajectory, "vehicle_response");
    reject_unknown_top_level_keys(
        vehicle_response,
        std::vector<std::string_view>{"type",
                                      "vehicle_rate_time_constant_pqr_s",
                                      "specific_force_command_time_constant_b_s",
                                      "specific_force_response_time_constant_b_s",
                                      "angular_rate_limit_pqr_degps",
                                      "specific_force_limit_b_mps2"});
    require_string(vehicle_response, "type");
    if (vehicle_response.at("type").get<std::string>() != "first_order") {
        throw_runtime_config_error("trajectory.vehicle_response.type must be 'first_order'");
    }
    validate_trajectory_vec3_bounds(vehicle_response, "vehicle_rate_time_constant_pqr_s", false);
    validate_trajectory_vec3_bounds(
        vehicle_response, "specific_force_command_time_constant_b_s", false);
    validate_trajectory_vec3_bounds(
        vehicle_response, "specific_force_response_time_constant_b_s", false);
    if (vehicle_response.contains("angular_rate_limit_pqr_degps")) {
        validate_trajectory_vec3_bounds(vehicle_response, "angular_rate_limit_pqr_degps", true);
    }
    if (vehicle_response.contains("specific_force_limit_b_mps2")) {
        validate_trajectory_vec3_bounds(vehicle_response, "specific_force_limit_b_mps2", true);
    }

    require_positive_number(trajectory, "maximum_bank_angle_deg");
    if (trajectory.at("maximum_bank_angle_deg").get<core::Scalar_t>() > 60.0) {
        throw_runtime_config_error("trajectory.maximum_bank_angle_deg must not exceed 60 degrees");
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

inline void validate_guidance_filter_object(const nlohmann::json& config, const bool entry_window)
{
    std::vector<std::string_view> keys{"specific_force_time_constant_b_s", "bank_time_constant_s"};
    if (entry_window) {
        keys.push_back("duration_s");
    }
    reject_unknown_top_level_keys(config, keys);
    validate_trajectory_vec3_bounds(config, "specific_force_time_constant_b_s", false);
    require_nonnegative_number(config, "bank_time_constant_s");
    if (entry_window) {
        require_positive_number(config, "duration_s");
    }
}

inline void validate_guidance_reference_channel(const nlohmann::json& channel,
                                                const std::string& path)
{
    require_string(channel, "type");
    const std::string type = channel.at("type").get<std::string>();
    if (type == "constant") {
        reject_unknown_top_level_keys(channel, {"type", "value_deg"});
        require_optional_number(channel, "value_deg");
        if (!channel.contains("value_deg")) {
            throw_runtime_config_error("missing required '" + path + ".value_deg'");
        }
        return;
    }
    if (type == "sine") {
        reject_unknown_top_level_keys(
            channel, {"type", "center_deg", "amplitude_deg", "period_s", "phase_deg"});
        require_optional_number(channel, "center_deg");
        require_positive_number(channel, "amplitude_deg");
        require_positive_number(channel, "period_s");
        require_optional_number(channel, "phase_deg");
        if (!channel.contains("center_deg") || !channel.contains("phase_deg")) {
            throw_runtime_config_error("Guidance sine reference '" + path +
                                       "' requires center_deg and phase_deg");
        }
        return;
    }
    throw_runtime_config_error("Guidance reference channel '" + path +
                               "' type must be 'constant' or 'sine'");
}

inline void validate_guidance_reference(const nlohmann::json& reference)
{
    require_string(reference, "type");
    const std::string type = reference.at("type").get<std::string>();
    if (type == "current_state") {
        reject_unknown_top_level_keys(reference, {"type"});
        return;
    }
    if (type == "local_flight_path") {
        reject_unknown_top_level_keys(reference, {"type", "speed", "heading", "flight_path"});
        const nlohmann::json& speed = require_object(reference, "speed");
        reject_unknown_top_level_keys(speed, {"type", "value_mps"});
        require_string(speed, "type");
        if (speed.at("type").get<std::string>() != "constant") {
            throw_runtime_config_error("Guidance local-flight-path speed.type must be 'constant'");
        }
        require_positive_number(speed, "value_mps");
        validate_guidance_reference_channel(require_object(reference, "heading"), "heading");
        validate_guidance_reference_channel(require_object(reference, "flight_path"),
                                            "flight_path");
        return;
    }
    if (type == "waypoint_path") {
        reject_unknown_top_level_keys(reference,
                                      {"type",
                                       "speed",
                                       "acceptance_radius_m",
                                       "heading_error_gain_mps2_per_deg",
                                       "waypoints_lla_deg_m"});
        const nlohmann::json& speed = require_object(reference, "speed");
        reject_unknown_top_level_keys(speed, {"type", "value_mps"});
        require_string(speed, "type");
        if (speed.at("type").get<std::string>() != "constant") {
            throw_runtime_config_error("Guidance waypoint speed.type must be 'constant'");
        }
        require_positive_number(speed, "value_mps");
        require_positive_number(reference, "acceptance_radius_m");
        require_nonnegative_number(reference, "heading_error_gain_mps2_per_deg");
        validate_waypoints_lla_deg_m(reference);
        return;
    }
    throw_runtime_config_error(
        "Guidance reference.type must be 'current_state', 'local_flight_path', or "
        "'waypoint_path'");
}

inline void validate_guidance_acceleration(const nlohmann::json& acceleration)
{
    if (!acceleration.is_array() || acceleration.empty()) {
        throw_runtime_config_error("Guidance translation.acceleration must be a nonempty array");
    }
    std::size_t direct_count = 0U;
    std::size_t path_feedforward_count = 0U;
    for (const nlohmann::json& contribution : acceleration) {
        if (!contribution.is_object()) {
            throw_runtime_config_error("Guidance acceleration entries must be objects");
        }
        require_string(contribution, "type");
        const std::string type = contribution.at("type").get<std::string>();
        if (type == "path_feedforward" || type == "free_fall") {
            reject_unknown_top_level_keys(contribution, {"type"});
            path_feedforward_count += type == "path_feedforward" ? 1U : 0U;
            direct_count += type == "free_fall" ? 1U : 0U;
        }
        else if (type == "velocity_hold") {
            reject_unknown_top_level_keys(contribution, {"type", "gain_n_1ps"});
            validate_trajectory_vec3_bounds(contribution, "gain_n_1ps", false);
        }
        else if (type == "altitude_hold_pd") {
            reject_unknown_top_level_keys(
                contribution, {"type", "target", "proportional_gain_1ps2", "derivative_gain_1ps"});
            const nlohmann::json& target = require_object(contribution, "target");
            require_string(target, "type");
            const std::string target_type = target.at("type").get<std::string>();
            if (target_type == "initial_altitude") {
                reject_unknown_top_level_keys(target, {"type"});
            }
            else if (target_type == "fixed") {
                reject_unknown_top_level_keys(target, {"type", "value_m"});
                require_optional_number(target, "value_m");
                if (!target.contains("value_m")) {
                    throw_runtime_config_error("Guidance fixed altitude target requires value_m");
                }
            }
            else {
                throw_runtime_config_error(
                    "Guidance altitude target.type must be 'initial_altitude' or 'fixed'");
            }
            require_nonnegative_number(contribution, "proportional_gain_1ps2");
            require_nonnegative_number(contribution, "derivative_gain_1ps");
        }
        else if (type == "body_specific_force") {
            reject_unknown_top_level_keys(contribution, {"type", "specific_force_ib_b_mps2"});
            require_optional_vec3(contribution, "specific_force_ib_b_mps2");
            if (!contribution.contains("specific_force_ib_b_mps2")) {
                throw_runtime_config_error(
                    "Guidance body_specific_force requires specific_force_ib_b_mps2");
            }
            ++direct_count;
        }
        else {
            throw_runtime_config_error("unsupported Guidance acceleration type '" + type + "'");
        }
    }
    if (direct_count > 0U && acceleration.size() != 1U) {
        throw_runtime_config_error(
            "Guidance direct body-specific-force/free-fall commands must be the sole "
            "acceleration entry");
    }
    if (path_feedforward_count > 1U) {
        throw_runtime_config_error(
            "Guidance translation may contain at most one path_feedforward entry");
    }
}

inline void validate_guidance_state(const nlohmann::json& state)
{
    reject_unknown_top_level_keys(state,
                                  {"id",
                                   "plant",
                                   "guidance",
                                   "autopilot",
                                   "guidance_command_filter",
                                   "on_entry",
                                   "transitions",
                                   "terminal"});
    require_string(state, "id");
    if (state.at("id").get<std::string>().empty()) {
        throw_runtime_config_error("Guidance state id must not be empty");
    }
    const nlohmann::json& plant = require_object(state, "plant");
    reject_unknown_top_level_keys(plant, {"constraint"});
    require_string(plant, "constraint");
    const std::string constraint = plant.at("constraint").get<std::string>();
    if (constraint != "none" && constraint != "hold_initial_ecef") {
        throw_runtime_config_error(
            "Guidance state plant.constraint must be 'none' or 'hold_initial_ecef'");
    }

    const nlohmann::json& guidance = require_object(state, "guidance");
    reject_unknown_top_level_keys(
        guidance, {"enabled", "translation", "bank", "body_y_specific_force_enabled"});
    require_bool(guidance, "enabled");
    require_optional_bool(guidance, "body_y_specific_force_enabled");
    const nlohmann::json& translation = require_object(guidance, "translation");
    reject_unknown_top_level_keys(translation, {"reference", "acceleration"});
    validate_guidance_reference(require_object(translation, "reference"));
    validate_guidance_acceleration(translation.at("acceleration"));
    const nlohmann::json& bank = require_object(guidance, "bank");
    reject_unknown_top_level_keys(bank, {"type"});
    require_string(bank, "type");
    const std::string bank_type = bank.at("type").get<std::string>();
    if (bank_type != "zero" && bank_type != "coordinated_bank_to_turn") {
        throw_runtime_config_error(
            "Guidance bank.type must be 'zero' or 'coordinated_bank_to_turn'");
    }

    const nlohmann::json& autopilot = require_object(state, "autopilot");
    reject_unknown_top_level_keys(autopilot, {"enabled"});
    require_bool(autopilot, "enabled");
    if (state.contains("guidance_command_filter")) {
        validate_guidance_filter_object(state.at("guidance_command_filter"), false);
    }
    if (state.contains("on_entry")) {
        const nlohmann::json& on_entry = require_object(state, "on_entry");
        reject_unknown_top_level_keys(on_entry, {"guidance_command_filter"});
        validate_guidance_filter_object(require_object(on_entry, "guidance_command_filter"), true);
    }

    const bool has_transitions = state.contains("transitions");
    const bool terminal = state.contains("terminal");
    if (has_transitions == terminal) {
        throw_runtime_config_error(
            "each Guidance state must specify exactly one of transitions or terminal");
    }
    if (terminal) {
        const nlohmann::json& terminal_config = require_object(state, "terminal");
        reject_unknown_top_level_keys(terminal_config, {"behavior"});
        require_string(terminal_config, "behavior");
        if (terminal_config.at("behavior").get<std::string>() !=
            "run_until_trajectory_termination") {
            throw_runtime_config_error(
                "Guidance terminal.behavior must be 'run_until_trajectory_termination'");
        }
        return;
    }

    const nlohmann::json& transitions = state.at("transitions");
    if (!transitions.is_array() || transitions.empty()) {
        throw_runtime_config_error("Guidance transitions must be a nonempty array");
    }
    std::unordered_set<std::size_t> priorities{};
    for (const nlohmann::json& transition : transitions) {
        reject_unknown_top_level_keys(transition, {"to", "priority", "when"});
        require_string(transition, "to");
        require_unsigned_integer(transition, "priority");
        const std::size_t priority = transition.at("priority").get<std::size_t>();
        if (!priorities.insert(priority).second) {
            throw_runtime_config_error(
                "Guidance transition priorities must be unique within a state");
        }
        const nlohmann::json& when = require_object(transition, "when");
        reject_unknown_top_level_keys(when, {"type", "greater_equal_s"});
        require_string(when, "type");
        if (when.at("type").get<std::string>() != "elapsed_in_state") {
            throw_runtime_config_error("Guidance transition when.type must be 'elapsed_in_state'");
        }
        require_positive_number(when, "greater_equal_s");
    }
}

inline void validate_guidance_state_machine(const nlohmann::json& trajectory)
{
    std::vector<std::string_view> allowed_trajectory_keys{
        "type",
        "duration_s",
        "dynamics_rate_hz",
        "dynamics_dt_s",
        "guidance_rate_hz",
        "guidance_dt_s",
        "autopilot_rate_hz",
        "autopilot_dt_s",
        "translational_integration",
        "termination",
        "autopilot",
        "maximum_bank_angle_deg",
        "guidance_command_filter",
        "vehicle_response",
        "state_machine",
        "p_e_m",
        "p_lla_deg_m",
        "v_e_mps",
        "v_n_mps",
        "w_ib_b_degps",
        "w_eb_b_degps",
        "w_nb_b_degps",
    };
    allowed_trajectory_keys.insert(allowed_trajectory_keys.end(),
                                   trajectory_attitude_json_keys.begin(),
                                   trajectory_attitude_json_keys.end());
    reject_unknown_top_level_keys(trajectory, allowed_trajectory_keys);

    const nlohmann::json& termination = require_object(trajectory, "termination");
    reject_unknown_top_level_keys(termination, {"type"});
    require_string(termination, "type");
    const std::string termination_type = termination.at("type").get<std::string>();
    if (termination_type != "configured_duration" && termination_type != "ground_impact") {
        throw_runtime_config_error(
            "trajectory.termination.type must be 'configured_duration' or 'ground_impact'");
    }

    const nlohmann::json& machine = require_object(trajectory, "state_machine");
    reject_unknown_top_level_keys(machine, {"initial_state_id", "cycle_policy", "states"});
    require_string(machine, "initial_state_id");
    require_string(machine, "cycle_policy");
    const std::string cycle_policy = machine.at("cycle_policy").get<std::string>();
    if (cycle_policy != "reject" && cycle_policy != "allow") {
        throw_runtime_config_error("Guidance cycle_policy must be 'reject' or 'allow'");
    }
    const nlohmann::json& states = machine.at("states");
    if (!states.is_array() || states.empty()) {
        throw_runtime_config_error("Guidance state_machine.states must be a nonempty array");
    }

    std::unordered_map<std::string, std::size_t> state_indices{};
    for (std::size_t index = 0U; index < states.size(); ++index) {
        const nlohmann::json& state = states.at(index);
        if (!state.is_object()) {
            throw_runtime_config_error("Guidance states must be objects");
        }
        validate_guidance_state(state);
        const std::string id = state.at("id").get<std::string>();
        if (!state_indices.emplace(id, index).second) {
            throw_runtime_config_error("Guidance state IDs must be unique");
        }
    }
    const std::string initial_state_id = machine.at("initial_state_id").get<std::string>();
    if (!state_indices.contains(initial_state_id)) {
        throw_runtime_config_error("Guidance initial_state_id does not name a configured state");
    }

    std::vector<std::vector<std::size_t>> edges(states.size());
    for (std::size_t index = 0U; index < states.size(); ++index) {
        const nlohmann::json& state = states.at(index);
        if (!state.contains("transitions")) {
            continue;
        }
        for (const nlohmann::json& transition : state.at("transitions")) {
            const std::string target = transition.at("to").get<std::string>();
            const std::unordered_map<std::string, std::size_t>::const_iterator iter =
                state_indices.find(target);
            if (iter == state_indices.end()) {
                throw_runtime_config_error("Guidance transition target '" + target +
                                           "' does not exist");
            }
            edges.at(index).push_back(iter->second);
        }
    }

    std::vector<int> color(states.size(), 0);
    bool cycle_detected = false;
    std::function<void(std::size_t)> visit = [&](const std::size_t index) {
        color.at(index) = 1;
        for (const std::size_t target : edges.at(index)) {
            if (color.at(target) == 0) {
                visit(target);
            }
            else if (color.at(target) == 1) {
                cycle_detected = true;
            }
        }
        color.at(index) = 2;
    };
    visit(state_indices.at(initial_state_id));
    for (const int state_color : color) {
        if (state_color == 0) {
            throw_runtime_config_error("Guidance state graph contains an unreachable state");
        }
    }
    if (cycle_detected && cycle_policy == "reject") {
        throw_runtime_config_error(
            "Guidance state graph contains a cycle while cycle_policy is 'reject'");
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

    std::vector<std::string_view> allowed_keys = EmulatorRuntimeKeys<EmulatorBindings>::values();
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
        application,
        std::vector<std::string_view>{"clock", "control_state_source", "dt_s", "rate_hz"});
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
    detail::require_string(application, "control_state_source");
    ControlStateSourceMode control_state_source{};
    if (!control_state_source_mode_from_string(
            application.at("control_state_source").get<std::string>(), control_state_source)) {
        detail::throw_runtime_config_error(
            "application.control_state_source must be 'navigation_estimate' or "
            "'truth_passthrough'");
    }

    const nlohmann::json& trajectory = detail::require_object(cfg, "trajectory");
    detail::require_optional_string(trajectory, "type");
    const std::string trajectory_type = trajectory.value("type", "stationary");
    if (trajectory_type == "csv") {
        detail::require_string(trajectory, "csv_path");
    }
    else if (trajectory_type == "stationary" || trajectory_type == "state_machine") {
        const bool is_stationary = trajectory_type == "stationary";
        detail::validate_generated_trajectory_common(trajectory, true, is_stationary);
        if (!is_stationary) {
            detail::validate_trajectory_dynamics_config(trajectory);
            detail::validate_guidance_state_machine(trajectory);
            if (!trajectory.contains("v_e_mps") && !trajectory.contains("v_n_mps")) {
                detail::throw_runtime_config_error(
                    "state-machine trajectory must explicitly configure initial velocity");
            }
        }
    }
    else {
        detail::throw_runtime_config_error(
            "trajectory.type must be 'stationary', 'csv', or 'state_machine'");
    }
    if (trajectory_type != "csv") {
        const core::RationalRate trajectory_rate = rational_rate_from_required_named_runtime_rate(
            trajectory, "dynamics_rate_hz", "dynamics_dt_s", "trajectory.dynamics");
        if (!core::rational_rate_is_integer_multiple(application_rate, trajectory_rate)) {
            detail::throw_runtime_config_error(
                "application rate must be an integer multiple of the generated trajectory "
                "physics rate");
        }
        if (trajectory_type != "stationary") {
            const core::RationalRate guidance_rate = rational_rate_from_required_named_runtime_rate(
                trajectory, "guidance_rate_hz", "guidance_dt_s", "trajectory.guidance");
            const core::RationalRate autopilot_rate =
                rational_rate_from_required_named_runtime_rate(
                    trajectory, "autopilot_rate_hz", "autopilot_dt_s", "trajectory.autopilot");
            if (!core::rational_rate_is_integer_multiple(trajectory_rate, autopilot_rate) ||
                !core::rational_rate_is_integer_multiple(autopilot_rate, guidance_rate)) {
                detail::throw_runtime_config_error(
                    "generated trajectory rates must satisfy physics >= autopilot >= "
                    "guidance as integer multiples");
            }
        }
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
