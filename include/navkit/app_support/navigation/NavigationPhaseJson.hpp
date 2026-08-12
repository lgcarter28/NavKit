// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/navigation/NavigationPhase.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <cmath>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support
{

namespace detail
{

[[nodiscard]] inline std::optional<core::Scalar_t>
navigation_phase_probability(const nlohmann::json& gate,
                             const nlohmann::json& baseline_gate,
                             const std::string_view phase_id,
                             const std::string_view sensor_role)
{
    const nlohmann::json* probability = nullptr;
    if (gate.contains("probability")) {
        probability = &gate.at("probability");
    }
    else if (baseline_gate.contains("probability")) {
        probability = &baseline_gate.at("probability");
    }
    if (probability == nullptr) {
        return std::nullopt;
    }
    if (!probability->is_number()) {
        throw_runtime_config_error("Navigation phase '" + std::string{phase_id} +
                                   "' sensor role '" + std::string{sensor_role} +
                                   "' chi-square probability must be numeric");
    }

    const core::Scalar_t result = probability->get<core::Scalar_t>();
    if (!navigation_phase_probability_is_valid(result)) {
        throw_runtime_config_error("Navigation phase '" + std::string{phase_id} +
                                   "' sensor role '" + std::string{sensor_role} +
                                   "' chi-square probability must be finite and strictly between "
                                   "zero and one");
    }
    return std::optional<core::Scalar_t>{result};
}

[[nodiscard]] inline NavigationInnovationGatePhase
navigation_gate_phase_from_json(const nlohmann::json& sensors,
                                const nlohmann::json& baseline_acceptance,
                                const std::string_view phase_id,
                                const std::string_view sensor_role,
                                const std::string_view baseline_family)
{
    const nlohmann::json& sensor = require_object(sensors, sensor_role);
    reject_unknown_top_level_keys(sensor, {"chi_square_acceptance"});
    const nlohmann::json& gate = require_object(sensor, "chi_square_acceptance");
    reject_unknown_top_level_keys(gate, {"enabled", "probability"});
    require_bool(gate, "enabled");
    require_optional_number(gate, "probability");

    const nlohmann::json& baseline_gate = require_object(baseline_acceptance, baseline_family);
    const bool enabled = gate.at("enabled").get<bool>();
    const std::optional<core::Scalar_t> probability =
        navigation_phase_probability(gate, baseline_gate, phase_id, sensor_role);
    if (enabled && !probability.has_value()) {
        throw_runtime_config_error("Navigation phase '" + std::string{phase_id} +
                                   "' sensor role '" + std::string{sensor_role} +
                                   "' requires either a phase probability override or a baseline "
                                   "GNSS chi-square probability when enabled");
    }
    return NavigationInnovationGatePhase{.probability = probability, .enabled = enabled};
}

} // namespace detail

/**
 * \brief Parses Navigation controls directly from the authoritative mission phase sequence.
 *
 * \details `mission.phases` is the only runtime phase ordering. Each phase owns a resolved
 * `navigation` object containing both primary GNSS sensor-role controls. An omitted phase-level
 * probability is resolved from the corresponding `gnss.chi_square_acceptance` baseline when one
 * is configured; an enabled gate must resolve a probability. The resulting vector retains mission
 * order exactly so the SWIL execution loop can apply the phase index published by the trajectory
 * source without joining a second phase catalog by name.
 */
[[nodiscard]] inline std::vector<NavigationPhase>
navigation_phases_from_mission_json(const nlohmann::json& cfg)
{
    const nlohmann::json& mission = detail::require_object(cfg, "mission");
    const nlohmann::json& phases = mission.at("phases");
    if (!phases.is_array() || phases.empty()) {
        detail::throw_runtime_config_error("mission.phases must be a nonempty array");
    }
    const nlohmann::json& gnss = detail::require_object(cfg, "gnss");
    const nlohmann::json& baseline_acceptance =
        detail::require_object(gnss, "chi_square_acceptance");

    std::vector<NavigationPhase> result{};
    result.reserve(phases.size());
    for (const nlohmann::json& phase : phases) {
        if (!phase.is_object()) {
            detail::throw_runtime_config_error("mission phases must be objects");
        }
        detail::require_string(phase, "id");
        const std::string phase_id = phase.at("id").get<std::string>();
        if (phase_id.empty()) {
            detail::throw_runtime_config_error("mission phase IDs must be nonempty");
        }
        const nlohmann::json& navigation = detail::require_object(phase, "navigation");
        detail::reject_unknown_top_level_keys(navigation, {"sensors"});
        const nlohmann::json& sensors = detail::require_object(navigation, "sensors");
        detail::reject_unknown_top_level_keys(sensors,
                                              {"primary_gnss_position", "primary_gnss_velocity"});

        result.push_back(NavigationPhase{
            .id = phase_id,
            .primary_gnss_position = detail::navigation_gate_phase_from_json(
                sensors, baseline_acceptance, phase_id, "primary_gnss_position", "position"),
            .primary_gnss_velocity = detail::navigation_gate_phase_from_json(
                sensors, baseline_acceptance, phase_id, "primary_gnss_velocity", "velocity"),
        });
    }
    return result;
}

} // namespace navkit::app_support
