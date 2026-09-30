// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/mission/MissionRuntime.hpp"
#include "navkit/app_support/navigation/NavigationPhaseJson.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <cstddef>
#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navkit::app_support
{

/**
 * \brief Resolves the authoritative mission JSON phase graph into a `MissionRuntime`.
 *
 * \details Phase IDs and transition targets are converted to checked indices once during runtime
 * configuration. The navigation selections retain the exact authoritative mission order.
 */
[[nodiscard]] inline MissionRuntime mission_runtime_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& mission = detail::require_object(cfg, "mission");
    detail::require_string(mission, "initial_phase_id");
    detail::require_string(mission, "cycle_policy");
    const std::string cycle_policy = mission.at("cycle_policy").get<std::string>();
    if (cycle_policy != "reject" && cycle_policy != "allow") {
        detail::throw_runtime_config_error("mission.cycle_policy must be 'reject' or 'allow'");
    }
    const nlohmann::json& phase_json = mission.at("phases");
    if (!phase_json.is_array() || phase_json.empty()) {
        detail::throw_runtime_config_error("mission.phases must be a nonempty array");
    }

    std::vector<NavigationPhase> navigation_phases = navigation_phases_from_mission_json(cfg);
    if (navigation_phases.size() != phase_json.size()) {
        detail::throw_runtime_config_error(
            "mission Navigation phase count does not match the authoritative phase graph");
    }

    std::unordered_map<std::string, std::size_t> phase_indices{};
    phase_indices.reserve(phase_json.size());
    for (std::size_t index = 0U; index < phase_json.size(); ++index) {
        const std::string& id = navigation_phases.at(index).id;
        if (id.empty() || !phase_indices.emplace(id, index).second) {
            detail::throw_runtime_config_error("mission phase IDs must be nonempty and unique");
        }
    }

    const std::string initial_phase_id = mission.at("initial_phase_id").get<std::string>();
    const std::unordered_map<std::string, std::size_t>::const_iterator initial_iter =
        phase_indices.find(initial_phase_id);
    if (initial_iter == phase_indices.end()) {
        detail::throw_runtime_config_error(
            "mission.initial_phase_id does not name a configured mission phase");
    }

    std::vector<MissionPhase> phases{};
    std::vector<std::vector<std::size_t>> edges(phase_json.size());
    phases.reserve(phase_json.size());
    for (std::size_t index = 0U; index < phase_json.size(); ++index) {
        const nlohmann::json& source_phase = phase_json.at(index);
        detail::reject_unknown_top_level_keys(
            source_phase, {"id", "navigation", "guidance", "autopilot", "transitions", "terminal"});
        std::vector<std::size_t> allowed_transition_indices{};
        const bool has_transitions = source_phase.contains("transitions");
        const bool terminal = source_phase.contains("terminal");
        if (has_transitions == terminal) {
            detail::throw_runtime_config_error("mission phase '" + navigation_phases.at(index).id +
                                               "' must specify exactly one of transitions or "
                                               "terminal");
        }
        if (has_transitions) {
            const nlohmann::json& transitions = source_phase.at("transitions");
            if (!transitions.is_array() || transitions.empty()) {
                detail::throw_runtime_config_error(
                    "mission nonterminal phases must define at least one transition");
            }
            allowed_transition_indices.reserve(transitions.size());
            for (const nlohmann::json& transition : transitions) {
                if (!transition.is_object()) {
                    detail::throw_runtime_config_error("mission phase transitions must be objects");
                }
                detail::require_string(transition, "to");
                const std::string target_id = transition.at("to").get<std::string>();
                const std::unordered_map<std::string, std::size_t>::const_iterator target_iter =
                    phase_indices.find(target_id);
                if (target_iter == phase_indices.end()) {
                    detail::throw_runtime_config_error("mission transition target '" + target_id +
                                                       "' does not exist");
                }
                allowed_transition_indices.push_back(target_iter->second);
                edges.at(index).push_back(target_iter->second);
            }
        }

        phases.push_back(MissionPhase{
            .id = navigation_phases.at(index).id,
            .navigation = std::move(navigation_phases.at(index)),
            .allowed_transition_indices = std::move(allowed_transition_indices),
            .terminal = terminal,
        });
    }

    std::vector<int> visit_state(phases.size(), 0);
    bool cycle_detected = false;
    std::function<void(std::size_t)> visit = [&](const std::size_t index) {
        visit_state.at(index) = 1;
        for (const std::size_t target_index : edges.at(index)) {
            if (visit_state.at(target_index) == 0) {
                visit(target_index);
            }
            else if (visit_state.at(target_index) == 1) {
                cycle_detected = true;
            }
        }
        visit_state.at(index) = 2;
    };
    visit(initial_iter->second);
    for (const int state : visit_state) {
        if (state == 0) {
            detail::throw_runtime_config_error("mission phase graph contains an unreachable phase");
        }
    }
    if (cycle_detected && cycle_policy == "reject") {
        detail::throw_runtime_config_error(
            "mission phase graph contains a cycle while cycle_policy is 'reject'");
    }

    MissionRuntime result{std::move(phases), initial_iter->second};
    if (!result.is_valid()) {
        detail::throw_runtime_config_error("resolved mission runtime graph is invalid");
    }
    return result;
}

} // namespace navkit::app_support
