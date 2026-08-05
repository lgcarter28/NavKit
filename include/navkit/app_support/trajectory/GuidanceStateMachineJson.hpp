// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"
#include "navkit/sim/guidance/GuidanceStateMachine.hpp"

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <utility>

namespace navkit::app_support
{

namespace detail
{

[[nodiscard]] inline sim::GuidanceCommandFilterConfig
guidance_filter_from_object(const nlohmann::json& config)
{
    sim::GuidanceCommandFilterConfig result{
        .specific_force_time_constant_b_s =
            vec3_from_json<core::Vec3>(config.at("specific_force_time_constant_b_s")),
        .bank_time_constant_s = config.at("bank_time_constant_s").get<core::Time_t>(),
    };
    if (!sim::guidance_command_filter_config_is_valid(result)) {
        throw_runtime_config_error(
            "Guidance command-filter time constants must be finite and nonnegative");
    }
    return result;
}

[[nodiscard]] inline sim::GuidanceCommandFilterStateEntryConfig
guidance_entry_filter_from_object(const nlohmann::json& config)
{
    return sim::GuidanceCommandFilterStateEntryConfig{
        .specific_force_time_constant_b_s =
            vec3_from_json<core::Vec3>(config.at("specific_force_time_constant_b_s")),
        .bank_time_constant_s = config.at("bank_time_constant_s").get<core::Time_t>(),
        .duration_s = config.at("duration_s").get<core::Time_t>(),
        .enabled = true,
    };
}

[[nodiscard]] inline sim::SinusoidalGuidanceReferenceModifierConfig
sinusoidal_reference_modifier_from_json(const nlohmann::json& config)
{
    return sim::SinusoidalGuidanceReferenceModifierConfig{
        .amplitude_rad = radians_from_degrees(config.at("amplitude_deg").get<core::Scalar_t>()),
        .frequency_hz = 1.0 / config.at("period_s").get<core::Time_t>(),
        .phase_rad = radians_from_degrees(config.at("phase_deg").get<core::Scalar_t>()),
    };
}

[[nodiscard]] inline core::Scalar_t reference_channel_center_rad(const nlohmann::json& channel)
{
    const std::string type = channel.at("type").get<std::string>();
    if (type == "constant") {
        return radians_from_degrees(channel.at("value_deg").get<core::Scalar_t>());
    }
    if (type == "sine") {
        return radians_from_degrees(channel.at("center_deg").get<core::Scalar_t>());
    }
    throw_runtime_config_error("Guidance reference channel type must be 'constant' or 'sine'");
}

[[nodiscard]] inline std::vector<core::Vec3>
state_machine_waypoints_e_m_from_json(const nlohmann::json& reference)
{
    std::vector<core::Vec3> result{};
    const nlohmann::json& waypoints = reference.at("waypoints_lla_deg_m");
    result.reserve(waypoints.size());
    for (const nlohmann::json& waypoint : waypoints) {
        core::Vec3 p_e_m{};
        if (!core::frames::lla_deg_m_to_ecef_m(vec3_from_json<core::Vec3>(waypoint), p_e_m)) {
            throw_runtime_config_error(
                "Guidance waypoint must contain finite latitude, longitude, and height");
        }
        result.push_back(p_e_m);
    }
    return result;
}

inline void populate_guidance_reference(const nlohmann::json& config,
                                        sim::GuidanceTranslationDefinition& translation)
{
    const std::string type = config.at("type").get<std::string>();
    if (type == "current_state") {
        translation.reference = std::make_unique<sim::CurrentStateGuidanceReference>();
        return;
    }
    if (type == "waypoint_path") {
        const nlohmann::json& speed = config.at("speed");
        const core::Scalar_t heading_gain_mps2_per_deg =
            config.at("heading_error_gain_mps2_per_deg").get<core::Scalar_t>();
        translation.reference =
            std::make_unique<sim::WaypointGuidanceReference>(sim::WaypointGuidanceReferenceConfig{
                .waypoint_e_m = state_machine_waypoints_e_m_from_json(config),
                .speed_mps = speed.at("value_mps").get<core::Scalar_t>(),
                .acceptance_radius_m = config.at("acceptance_radius_m").get<core::Scalar_t>(),
                .heading_error_gain_mps2_per_rad =
                    heading_gain_mps2_per_deg * 180.0 / std::numbers::pi_v<core::Scalar_t>,
            });
        return;
    }
    if (type != "local_flight_path") {
        throw_runtime_config_error(
            "Guidance reference type must be 'current_state', 'local_flight_path', or "
            "'waypoint_path'");
    }

    const nlohmann::json& speed = config.at("speed");
    const nlohmann::json& heading = config.at("heading");
    const nlohmann::json& flight_path = config.at("flight_path");
    translation.reference = std::make_unique<sim::ConstantSpeedGuidanceReference>(
        sim::ConstantSpeedGuidanceReferenceConfig{
            .speed_mps = speed.at("value_mps").get<core::Scalar_t>(),
            .heading_rad = reference_channel_center_rad(heading),
            .pitch_rad = reference_channel_center_rad(flight_path),
            .capture_initial_heading = false,
            .capture_initial_pitch = false,
        });
    if (heading.at("type").get<std::string>() == "sine") {
        translation.reference_modifiers.push_back(
            std::make_unique<sim::HorizontalSinusoidalGuidanceReferenceModifier>(
                sinusoidal_reference_modifier_from_json(heading)));
    }
    if (flight_path.at("type").get<std::string>() == "sine") {
        translation.reference_modifiers.push_back(
            std::make_unique<sim::VerticalSinusoidalGuidanceReferenceModifier>(
                sinusoidal_reference_modifier_from_json(flight_path)));
    }
}

inline void populate_guidance_acceleration(const nlohmann::json& acceleration,
                                           sim::GuidanceTranslationDefinition& translation)
{
    for (const nlohmann::json& contribution : acceleration) {
        const std::string type = contribution.at("type").get<std::string>();
        if (type == "path_feedforward") {
            translation.acceleration.push_back(
                std::make_unique<sim::PathFeedforwardGuidanceAcceleration>());
            continue;
        }
        if (type == "velocity_hold") {
            translation.acceleration.push_back(
                std::make_unique<sim::VelocityTrackingGuidanceAcceleration>(
                    sim::VelocityTrackingGuidanceAccelerationConfig{
                        .gain_n_1ps = vec3_from_json<core::Vec3>(contribution.at("gain_n_1ps")),
                    }));
            continue;
        }
        if (type == "altitude_hold_pd") {
            const nlohmann::json& target = contribution.at("target");
            const bool capture_initial = target.at("type").get<std::string>() == "initial_altitude";
            translation.acceleration.push_back(
                std::make_unique<sim::AltitudeHoldPdGuidanceAcceleration>(
                    sim::AltitudeHoldPdGuidanceAccelerationConfig{
                        .target_altitude_m = target.value("value_m", core::Scalar_t{}),
                        .proportional_gain_1ps2 =
                            contribution.at("proportional_gain_1ps2").get<core::Scalar_t>(),
                        .derivative_gain_1ps =
                            contribution.at("derivative_gain_1ps").get<core::Scalar_t>(),
                        .capture_initial_altitude = capture_initial,
                    }));
            continue;
        }
        if (type == "body_specific_force" || type == "free_fall") {
            const core::Vec3 specific_force_ib_b_mps2 =
                type == "free_fall"
                    ? core::Vec3::Zero()
                    : vec3_from_json<core::Vec3>(contribution.at("specific_force_ib_b_mps2"));
            translation.acceleration.push_back(
                std::make_unique<sim::BodySpecificForceGuidanceAcceleration>(
                    sim::BodySpecificForceGuidanceAccelerationConfig{
                        .specific_force_ib_b_mps2 = specific_force_ib_b_mps2,
                    }));
            continue;
        }
        throw_runtime_config_error("Unsupported Guidance acceleration contribution '" + type + "'");
    }
}

[[nodiscard]] inline sim::GuidanceStateDefinition
guidance_state_from_json(const nlohmann::json& state,
                         const sim::GuidanceCommandFilterConfig& global_filter,
                         const core::Scalar_t maximum_bank_angle_rad)
{
    sim::GuidanceStateDefinition result{};
    result.id = state.at("id").get<std::string>();
    const std::string plant_constraint = state.at("plant").at("constraint").get<std::string>();
    if (plant_constraint == "hold_initial_ecef") {
        result.plant_constraint = sim::GuidancePlantConstraint::StaticLaunchPad;
    }
    else if (plant_constraint == "none") {
        result.plant_constraint = sim::GuidancePlantConstraint::None;
    }
    else {
        throw_runtime_config_error(
            "Guidance state plant.constraint must be 'none' or 'hold_initial_ecef'");
    }

    const nlohmann::json& guidance = state.at("guidance");
    result.guidance_enabled = guidance.at("enabled").get<bool>();
    result.body_y_specific_force_enabled = guidance.value("body_y_specific_force_enabled", true);
    const nlohmann::json& translation = guidance.at("translation");
    populate_guidance_reference(translation.at("reference"), result.translation);
    populate_guidance_acceleration(translation.at("acceleration"), result.translation);

    const std::string bank_type = guidance.at("bank").at("type").get<std::string>();
    if (bank_type == "zero") {
        result.bank = std::make_unique<sim::ZeroBankGuidancePolicy>();
    }
    else if (bank_type == "coordinated_bank_to_turn") {
        result.bank = std::make_unique<sim::CoordinatedBankToTurnGuidancePolicy>(
            sim::CoordinatedBankToTurnGuidancePolicyConfig{
                .maximum_bank_angle_rad = maximum_bank_angle_rad,
            });
    }
    else {
        throw_runtime_config_error("Unsupported Guidance bank policy '" + bank_type + "'");
    }

    result.autopilot_enabled = state.at("autopilot").at("enabled").get<bool>();
    result.guidance_command_filter =
        state.contains("guidance_command_filter")
            ? guidance_filter_from_object(state.at("guidance_command_filter"))
            : global_filter;
    if (state.contains("on_entry")) {
        result.on_entry_guidance_command_filter =
            guidance_entry_filter_from_object(state.at("on_entry").at("guidance_command_filter"));
    }
    if (state.contains("terminal")) {
        result.terminal = true;
    }
    if (state.contains("transitions")) {
        for (const nlohmann::json& transition : state.at("transitions")) {
            result.transitions.push_back(sim::GuidanceElapsedTransition{
                .to = transition.at("to").get<std::string>(),
                .elapsed_in_state_s =
                    transition.at("when").at("greater_equal_s").get<core::Time_t>(),
                .priority = transition.at("priority").get<std::size_t>(),
            });
        }
    }
    return result;
}

[[nodiscard]] inline sim::GuidanceStateMachineDefinition
guidance_state_machine_from_json(const nlohmann::json& trajectory,
                                 const sim::GuidanceCommandFilterConfig& global_filter,
                                 const core::Scalar_t maximum_bank_angle_rad)
{
    const nlohmann::json& state_machine = trajectory.at("state_machine");
    sim::GuidanceStateMachineDefinition result{};
    result.initial_state_id = state_machine.at("initial_state_id").get<std::string>();
    const std::string cycle_policy = state_machine.at("cycle_policy").get<std::string>();
    if (cycle_policy == "allow") {
        result.cycle_policy = sim::GuidanceCyclePolicy::Allow;
    }
    else if (cycle_policy == "reject") {
        result.cycle_policy = sim::GuidanceCyclePolicy::Reject;
    }
    else {
        throw_runtime_config_error("Guidance cycle_policy must be 'reject' or 'allow'");
    }
    const nlohmann::json& states = state_machine.at("states");
    result.states.reserve(states.size());
    for (const nlohmann::json& state : states) {
        result.states.push_back(
            guidance_state_from_json(state, global_filter, maximum_bank_angle_rad));
    }
    std::string diagnostic{};
    if (!sim::guidance_state_machine_definition_is_valid(result, diagnostic)) {
        throw_runtime_config_error("invalid Guidance state machine: " + diagnostic);
    }
    return result;
}

} // namespace detail

} // namespace navkit::app_support
