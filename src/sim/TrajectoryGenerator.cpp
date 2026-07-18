// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/TrajectoryGenerator.hpp"

namespace navkit::sim
{

std::vector<TruthSample> TrajectoryGenerator::stationary(const StationaryTrajectoryConfig& cfg)
{
    std::vector<TruthSample> samples;
    const int n = static_cast<int>(cfg.duration_s / cfg.dt_s) + 1;
    samples.reserve(static_cast<std::size_t>(n));

    for (int k = 0; k < n; ++k) {
        TruthSample s;
        s.time = static_cast<Time_t>(k) * cfg.dt_s;
        s.p_e = cfg.p_e;
        s.v_e = cfg.v_e;
        s.q_b2e = cfg.q_b2e;
        s.w_ib_b_radps = cfg.w_ib_b_radps;
        samples.push_back(s);
    }

    return samples;
}

} // namespace navkit::sim
