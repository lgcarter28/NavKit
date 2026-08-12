// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <cstddef>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navkit::app_support
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
 * \brief Validates the source-independent mission phase graph and SWIL behavior mapping.
 *
 * \details Every SWIL truth source consumes the same authoritative mission phase IDs, initial
 * phase, transition topology, and cycle policy. The simulation must provide exactly one plant
 * behavior object for every phase. This check intentionally runs before source-specific adapter
 * branches so stationary and CSV sources cannot bypass graph validation.
 */
inline void validate_swil_mission_phase_graph(const nlohmann::json& mission,
                                              const nlohmann::json& simulation)
{
    require_string(mission, "initial_phase_id");
    require_string(mission, "cycle_policy");
    const std::string cycle_policy = mission.at("cycle_policy").get<std::string>();
    if (cycle_policy != "reject" && cycle_policy != "allow") {
        throw_runtime_config_error("mission.cycle_policy must be 'reject' or 'allow'");
    }

    const nlohmann::json& phases = mission.at("phases");
    if (!phases.is_array() || phases.empty()) {
        throw_runtime_config_error("mission.phases must be a nonempty array");
    }
    const nlohmann::json& phase_behavior = require_object(simulation, "phase_behavior");
    if (phase_behavior.size() != phases.size()) {
        throw_runtime_config_error(
            "simulation.phase_behavior must define exactly one entry for every mission phase");
    }

    std::unordered_map<std::string, std::size_t> phase_indices{};
    phase_indices.reserve(phases.size());
    for (std::size_t index = 0U; index < phases.size(); ++index) {
        const nlohmann::json& phase = phases.at(index);
        if (!phase.is_object()) {
            throw_runtime_config_error("mission.phases entries must be objects");
        }
        reject_unknown_top_level_keys(
            phase, {"id", "navigation", "guidance", "autopilot", "transitions", "terminal"});
        require_string(phase, "id");
        const std::string phase_id = phase.at("id").get<std::string>();
        if (phase_id.empty() || !phase_indices.emplace(phase_id, index).second) {
            throw_runtime_config_error("mission phase IDs must be nonempty and unique");
        }
        if (!phase_behavior.contains(phase_id) || !phase_behavior.at(phase_id).is_object()) {
            throw_runtime_config_error("simulation.phase_behavior does not define mission phase '" +
                                       phase_id + "'");
        }

        const bool has_transitions = phase.contains("transitions");
        const bool has_terminal = phase.contains("terminal");
        if (has_transitions == has_terminal) {
            throw_runtime_config_error("mission phase '" + phase_id +
                                       "' must specify exactly one of transitions or terminal");
        }
        if (has_transitions &&
            (!phase.at("transitions").is_array() || phase.at("transitions").empty())) {
            throw_runtime_config_error("mission phase '" + phase_id +
                                       "' transitions must be a nonempty array");
        }
    }

    for (nlohmann::json::const_iterator iter = phase_behavior.begin(); iter != phase_behavior.end();
         ++iter) {
        if (!phase_indices.contains(iter.key())) {
            throw_runtime_config_error("simulation.phase_behavior defines unknown mission phase '" +
                                       iter.key() + "'");
        }
    }

    const std::string initial_phase_id = mission.at("initial_phase_id").get<std::string>();
    const std::unordered_map<std::string, std::size_t>::const_iterator initial_iter =
        phase_indices.find(initial_phase_id);
    if (initial_iter == phase_indices.end()) {
        throw_runtime_config_error(
            "mission.initial_phase_id does not name a configured mission phase");
    }

    std::vector<std::vector<std::size_t>> edges(phases.size());
    for (std::size_t index = 0U; index < phases.size(); ++index) {
        const nlohmann::json& phase = phases.at(index);
        if (!phase.contains("transitions")) {
            continue;
        }
        for (const nlohmann::json& transition : phase.at("transitions")) {
            if (!transition.is_object()) {
                throw_runtime_config_error("mission phase transitions must be objects");
            }
            require_string(transition, "to");
            const std::string target = transition.at("to").get<std::string>();
            const std::unordered_map<std::string, std::size_t>::const_iterator target_iter =
                phase_indices.find(target);
            if (target_iter == phase_indices.end()) {
                throw_runtime_config_error("mission transition target '" + target +
                                           "' does not exist");
            }
            edges.at(index).push_back(target_iter->second);
        }
    }

    std::vector<int> color(phases.size(), 0);
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
    visit(initial_iter->second);
    for (const int phase_color : color) {
        if (phase_color == 0) {
            throw_runtime_config_error("mission phase graph contains an unreachable phase");
        }
    }
    if (cycle_detected && cycle_policy == "reject") {
        throw_runtime_config_error(
            "mission phase graph contains a cycle while cycle_policy is 'reject'");
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
    const nlohmann::json& mission = require_object(cfg, "mission");
    const nlohmann::json& simulation = require_object(cfg, "simulation");
    const nlohmann::json& source = require_object(simulation, "source");
    reject_unknown_top_level_keys(
        mission,
        {"duration_s", "termination", "gnc", "initial_phase_id", "cycle_policy", "phases"});
    reject_unknown_top_level_keys(simulation,
                                  {"source",
                                   "dynamics",
                                   "initial_truth",
                                   "vehicle_response",
                                   "phase_behavior",
                                   "control_state_source"});
    reject_unknown_top_level_keys(source, {"type", "csv_path"});
    require_string(source, "type");
    const std::string source_type = source.at("type").get<std::string>();
    if (source_type != "stationary" && source_type != "generated" && source_type != "csv") {
        throw_runtime_config_error(
            "simulation.source.type must be 'stationary', 'generated', or 'csv'");
    }

    validate_swil_mission_phase_graph(mission, simulation);
    const nlohmann::json& phases = mission.at("phases");
    if ((source_type == "stationary" || source_type == "csv") && phases.size() != 1U) {
        throw_runtime_config_error(
            "simulation.source.type '" + source_type +
            "' supports exactly one mission phase until a source-independent MissionRuntime "
            "owns phase advancement");
    }

    nlohmann::json trajectory = nlohmann::json::object();
    trajectory["type"] = source_type == "generated" ? "state_machine" : source_type;

    if (source_type == "csv") {
        require_string(source, "csv_path");
        trajectory["csv_path"] = source.at("csv_path");
        return trajectory;
    }

    require_positive_number(mission, "duration_s");
    trajectory["duration_s"] = mission.at("duration_s");
    const nlohmann::json& dynamics = require_object(simulation, "dynamics");
    reject_unknown_top_level_keys(dynamics, {"rate_hz", "dt_s", "translational_integration"});
    if (dynamics.contains("rate_hz")) {
        trajectory["dynamics_rate_hz"] = dynamics.at("rate_hz");
    }
    if (dynamics.contains("dt_s")) {
        trajectory["dynamics_dt_s"] = dynamics.at("dt_s");
    }

    const nlohmann::json& initial_truth = require_object(simulation, "initial_truth");
    for (nlohmann::json::const_iterator iter = initial_truth.begin(); iter != initial_truth.end();
         ++iter) {
        trajectory[iter.key()] = iter.value();
    }

    if (source_type == "stationary") {
        return trajectory;
    }
    const nlohmann::json& gnc = require_object(mission, "gnc");
    reject_unknown_top_level_keys(gnc, {"guidance", "autopilot"});
    const nlohmann::json& guidance = require_object(gnc, "guidance");
    const nlohmann::json& autopilot = require_object(gnc, "autopilot");
    reject_unknown_top_level_keys(guidance,
                                  {"rate_hz", "dt_s", "maximum_bank_angle_deg", "command_filter"});
    reject_unknown_top_level_keys(autopilot, {"rate_hz", "dt_s", "model"});
    const nlohmann::json& autopilot_model = require_object(autopilot, "model");

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

    const nlohmann::json& phase_behavior = require_object(simulation, "phase_behavior");

    nlohmann::json compiled_machine{
        {"initial_state_id", mission.at("initial_phase_id")},
        {"cycle_policy", mission.at("cycle_policy")},
        {"states", nlohmann::json::array()},
    };
    for (const nlohmann::json& phase : phases) {
        const std::string phase_id = phase.at("id").get<std::string>();

        const nlohmann::json& guidance_phase = require_object(phase, "guidance");
        const nlohmann::json& autopilot_phase = require_object(phase, "autopilot");
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

} // namespace navkit::app_support
