// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/trajectory/ControlStateSourceMode.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/sim/trajectory/TrajectorySource.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"
#include "navkit/swil/SwilRuntimeConfig.hpp"

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace navkit::swil
{

/**
 * \brief SWIL-owned truth source and feedback selection for one resolved runtime scenario.
 *
 * \details Navigation phases deliberately do not live here. `MissionRuntime` owns mission
 * identity and legal transitions, while this aggregate owns only synthetic truth and the
 * feedback mode needed by the SWIL plant.
 */
struct SwilRuntime
{
    std::unique_ptr<sim::TrajectorySource> source{};
    sim::TruthSample initial_truth{};
    app_support::ControlStateSourceMode control_state_source{
        app_support::ControlStateSourceMode::NavigationEstimate};
};

/** \brief Parses the SWIL plant feedback source selected by the runtime scenario. */
[[nodiscard]] inline app_support::ControlStateSourceMode
swil_control_state_source_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& simulation = app_support::detail::require_object(cfg, "simulation");
    app_support::detail::require_string(simulation, "control_state_source");
    app_support::ControlStateSourceMode mode{};
    if (!app_support::control_state_source_mode_from_string(
            simulation.at("control_state_source").get<std::string>(), mode)) {
        app_support::detail::throw_runtime_config_error(
            "simulation.control_state_source must be 'navigation_estimate' or "
            "'truth_passthrough'");
    }
    return mode;
}

/**
 * \brief Constructs the target-specific SWIL runtime from a resolved configuration graph.
 *
 * \details Mission graph parsing is intentionally absent. The returned runtime is suitable only
 * for the concrete SWIL execution adapter and never enters the target-neutral mission boundary.
 */
[[nodiscard]] inline SwilRuntime
swil_runtime_from_json(const nlohmann::json& cfg, const std::filesystem::path& source_base_dir = {})
{
    const nlohmann::json trajectory_config = detail::swil_trajectory_config_from_json(cfg);
    const std::string type = trajectory_config.value("type", "stationary");
    const app_support::ControlStateSourceMode control_state_source =
        swil_control_state_source_from_json(cfg);

    if (type == "csv") {
        const std::filesystem::path csv_path =
            source_base_dir / trajectory_config.at("csv_path").get<std::string>();
        sim::TruthTrajectory truth = app_support::detail::truth_trajectory_from_csv(csv_path);
        const sim::TruthSample initial_truth = truth.first();
        return {.source = std::make_unique<sim::TabulatedTrajectorySource>(std::move(truth)),
                .initial_truth = initial_truth,
                .control_state_source = control_state_source};
    }

    if (type == "stationary") {
        const sim::StationaryTrajectoryConfig trajectory =
            app_support::stationary_trajectory_config_from_compiled_json(trajectory_config);
        const sim::TruthSample initial_truth{
            .t = trajectory.t_epoch,
            .p_e = trajectory.p_e,
            .v_e = trajectory.v_e,
            .q_b2e = trajectory.q_b2e,
            .w_ib_b_radps = trajectory.w_ib_b_radps,
        };
        return {.source = std::make_unique<sim::StationaryTrajectorySource>(trajectory),
                .initial_truth = initial_truth,
                .control_state_source = control_state_source};
    }

    if (type != "state_machine") {
        app_support::detail::throw_runtime_config_error(
            "simulation.source.type must be 'stationary', 'csv', or 'generated'");
    }

    std::unique_ptr<sim::TrajectorySource> source = sim::state_machine_trajectory_source(
        app_support::state_machine_trajectory_config_from_compiled_json(trajectory_config));
    if (!source || !source->advance_to(source->t_start())) {
        app_support::detail::throw_runtime_config_error(
            "trajectory generation failed for the configured profile");
    }
    sim::TruthSample initial_truth{};
    if (!source->query(source->t_start(), initial_truth)) {
        app_support::detail::throw_runtime_config_error(
            "trajectory generation did not provide its initial truth sample");
    }
    return {.source = std::move(source),
            .initial_truth = initial_truth,
            .control_state_source = control_state_source};
}

} // namespace navkit::swil
