// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/mission/MissionRuntimeJson.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>

namespace navkit::swil
{

namespace detail
{

/** Copy one optional member without changing its JSON representation. */
inline void copy_optional_json_member(const nlohmann::json& source,
                                      const std::string_view key,
                                      nlohmann::json& destination)
{
    const std::string owned_key{key};
    if (source.contains(owned_key)) {
        destination[owned_key] = source.at(owned_key);
    }
}

/**
 * \brief Validates the SWIL plant-behavior mapping against a resolved mission graph.
 *
 * \details `MissionRuntime` has already validated phase IDs, transition targets, reachability,
 * and cycle policy. This SWIL-only check requires exactly one synthetic plant behavior object for
 * every mission phase and rejects orphaned behavior entries.
 */
inline void validate_swil_phase_behavior_mapping(const app_support::MissionRuntime& mission_runtime,
                                                 const nlohmann::json& simulation)
{
    const nlohmann::json& phase_behavior =
        app_support::detail::require_object(simulation, "phase_behavior");
    if (phase_behavior.size() != mission_runtime.phase_count()) {
        app_support::detail::throw_runtime_config_error(
            "simulation.phase_behavior must define exactly one entry for every mission phase");
    }

    for (nlohmann::json::const_iterator iter = phase_behavior.begin(); iter != phase_behavior.end();
         ++iter) {
        if (!iter.value().is_object()) {
            app_support::detail::throw_runtime_config_error("simulation.phase_behavior entry '" +
                                                            iter.key() + "' must be an object");
        }
        std::size_t phase_index{};
        if (!mission_runtime.phase_index(iter.key(), phase_index)) {
            app_support::detail::throw_runtime_config_error(
                "simulation.phase_behavior defines unknown mission phase '" + iter.key() + "'");
        }
    }
}

/**
 * \brief Compiles a resolved mission and SWIL simulation model into trajectory configuration.
 *
 * \details `mission.phases` is the sole ordered phase graph. Guidance and autopilot behavior are
 * read directly from each resolved mission phase. The simulation owns phase-specific plant
 * behavior in `simulation.phase_behavior`, keyed by the stable mission phase ID. The compiled
 * JSON is an app-support adapter detail consumed by the existing simulation trajectory parser;
 * it is not a second public runtime schema.
 */
[[nodiscard]] inline nlohmann::json swil_trajectory_config_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& mission = app_support::detail::require_object(cfg, "mission");
    const nlohmann::json& simulation = app_support::detail::require_object(cfg, "simulation");
    const app_support::MissionRuntime mission_runtime = app_support::mission_runtime_from_json(cfg);
    const nlohmann::json& source = app_support::detail::require_object(simulation, "source");
    app_support::detail::reject_unknown_top_level_keys(
        mission,
        {"duration_s", "termination", "gnc", "initial_phase_id", "cycle_policy", "phases"});
    app_support::detail::reject_unknown_top_level_keys(simulation,
                                                       {"source",
                                                        "dynamics",
                                                        "initial_truth",
                                                        "vehicle_response",
                                                        "phase_behavior",
                                                        "control_state_source"});
    app_support::detail::reject_unknown_top_level_keys(source, {"type", "csv_path"});
    app_support::detail::require_string(source, "type");
    const std::string source_type = source.at("type").get<std::string>();
    if (source_type != "stationary" && source_type != "generated" && source_type != "csv") {
        app_support::detail::throw_runtime_config_error(
            "simulation.source.type must be 'stationary', 'generated', or 'csv'");
    }

    validate_swil_phase_behavior_mapping(mission_runtime, simulation);
    const nlohmann::json& phases = mission.at("phases");
    if ((source_type == "stationary" || source_type == "csv") &&
        mission_runtime.phase_count() != 1U) {
        app_support::detail::throw_runtime_config_error(
            "simulation.source.type '" + source_type +
            "' supports exactly one mission phase because that source does not emit mission "
            "transition events");
    }

    nlohmann::json trajectory = nlohmann::json::object();
    trajectory["type"] = source_type == "generated" ? "state_machine" : source_type;

    if (source_type == "csv") {
        app_support::detail::require_string(source, "csv_path");
        trajectory["csv_path"] = source.at("csv_path");
        return trajectory;
    }

    app_support::detail::require_positive_number(mission, "duration_s");
    trajectory["duration_s"] = mission.at("duration_s");
    const nlohmann::json& dynamics = app_support::detail::require_object(simulation, "dynamics");
    app_support::detail::reject_unknown_top_level_keys(
        dynamics, {"rate_hz", "dt_s", "translational_integration"});
    if (dynamics.contains("rate_hz")) {
        trajectory["dynamics_rate_hz"] = dynamics.at("rate_hz");
    }
    if (dynamics.contains("dt_s")) {
        trajectory["dynamics_dt_s"] = dynamics.at("dt_s");
    }

    const nlohmann::json& initial_truth =
        app_support::detail::require_object(simulation, "initial_truth");
    for (nlohmann::json::const_iterator iter = initial_truth.begin(); iter != initial_truth.end();
         ++iter) {
        trajectory[iter.key()] = iter.value();
    }

    if (source_type == "stationary") {
        return trajectory;
    }
    const nlohmann::json& gnc = app_support::detail::require_object(mission, "gnc");
    app_support::detail::reject_unknown_top_level_keys(gnc, {"guidance", "autopilot"});
    const nlohmann::json& guidance = app_support::detail::require_object(gnc, "guidance");
    const nlohmann::json& autopilot = app_support::detail::require_object(gnc, "autopilot");
    app_support::detail::reject_unknown_top_level_keys(
        guidance, {"rate_hz", "dt_s", "maximum_bank_angle_deg", "command_filter"});
    app_support::detail::reject_unknown_top_level_keys(autopilot, {"rate_hz", "dt_s", "model"});
    const nlohmann::json& autopilot_model = app_support::detail::require_object(autopilot, "model");

    if (guidance.contains("rate_hz")) {
        trajectory["guidance_rate_hz"] = guidance.at("rate_hz");
    }
    if (guidance.contains("dt_s")) {
        trajectory["guidance_dt_s"] = guidance.at("dt_s");
    }
    if (autopilot.contains("rate_hz")) {
        trajectory["autopilot_rate_hz"] = autopilot.at("rate_hz");
    }
    if (autopilot.contains("dt_s")) {
        trajectory["autopilot_dt_s"] = autopilot.at("dt_s");
    }
    trajectory["translational_integration"] = dynamics.at("translational_integration");
    trajectory["termination"] = mission.at("termination");
    trajectory["autopilot"] = autopilot_model;
    trajectory["maximum_bank_angle_deg"] = guidance.at("maximum_bank_angle_deg");
    trajectory["guidance_command_filter"] = guidance.at("command_filter");
    trajectory["vehicle_response"] = simulation.at("vehicle_response");

    const nlohmann::json& phase_behavior =
        app_support::detail::require_object(simulation, "phase_behavior");

    nlohmann::json compiled_machine{
        {"initial_state_id", mission.at("initial_phase_id")},
        {"cycle_policy", mission.at("cycle_policy")},
        {"states", nlohmann::json::array()},
    };
    for (const nlohmann::json& phase : phases) {
        const std::string phase_id = phase.at("id").get<std::string>();

        const nlohmann::json& guidance_phase =
            app_support::detail::require_object(phase, "guidance");
        const nlohmann::json& autopilot_phase =
            app_support::detail::require_object(phase, "autopilot");
        nlohmann::json compiled_phase{
            {"id", phase_id},
            {"guidance", guidance_phase},
            {"autopilot", autopilot_phase},
            {"plant", phase_behavior.at(phase_id)},
        };
        copy_optional_json_member(guidance_phase, "guidance_command_filter", compiled_phase);
        copy_optional_json_member(guidance_phase, "on_entry", compiled_phase);
        compiled_phase["guidance"].erase("guidance_command_filter");
        compiled_phase["guidance"].erase("on_entry");
        copy_optional_json_member(phase, "transitions", compiled_phase);
        copy_optional_json_member(phase, "terminal", compiled_phase);
        compiled_machine["states"].push_back(std::move(compiled_phase));
    }

    trajectory["state_machine"] = std::move(compiled_machine);
    return trajectory;
}

} // namespace detail

} // namespace navkit::swil
