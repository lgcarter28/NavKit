// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <nlohmann/json.hpp>
#include <vector>

namespace navkit::app_support
{

struct TrajectoryRun
{
    core::Vec3 initial_position_e_m{};
    std::vector<sim::TruthSample> truth_samples{};
};

inline sim::StationaryTrajectoryConfig
stationary_trajectory_config_from_json(const nlohmann::json& cfg)
{
    const auto& trajectory_config = cfg.at("trajectory");
    sim::StationaryTrajectoryConfig traj_cfg;
    traj_cfg.duration_s = trajectory_config.value("duration_s", 60.0);
    traj_cfg.dt_s = dt_s_from_runtime_rate(trajectory_config, 0.001);
    traj_cfg.p_e = vec3_from_json<core::Vec3>(trajectory_config.at("p_e_m"));
    return traj_cfg;
}

inline TrajectoryRun trajectory_run_from_json(const nlohmann::json& cfg)
{
    const auto traj_cfg = stationary_trajectory_config_from_json(cfg);
    return {.initial_position_e_m = traj_cfg.p_e,
            .truth_samples = sim::TrajectoryGenerator::stationary(traj_cfg)};
}

} // namespace navkit::app_support
