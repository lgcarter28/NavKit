// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/TrajectoryGenerator.hpp"

#include "navkit/core/time/Duration.hpp"

#include <cstdint>
#include <limits>

namespace navkit::sim
{

std::vector<TruthSample> TrajectoryGenerator::stationary(const StationaryTrajectoryConfig& cfg)
{
    std::vector<TruthSample> samples;
    if (!core::timestamp_is_valid(cfg.t_epoch) || !core::rational_rate_is_valid(cfg.rate) ||
        cfg.duration_s < 0.0) {
        return samples;
    }

    core::SampleIndex sample_index = 0U;
    while (true) {
        core::Timestamp t{};
        if (!core::timestamp_at_sample_index(cfg.t_epoch, cfg.rate, sample_index, t)) {
            return {};
        }
        core::Duration elapsed{};
        if (!core::elapsed_time(t, cfg.t_epoch, elapsed) ||
            core::duration_seconds(elapsed) > cfg.duration_s) {
            break;
        }
        TruthSample s;
        s.t = t;
        s.p_e = cfg.p_e;
        s.v_e = cfg.v_e;
        s.q_b2e = cfg.q_b2e;
        s.w_ib_b_radps = cfg.w_ib_b_radps;
        samples.push_back(s);
        if (sample_index == std::numeric_limits<core::SampleIndex>::max()) {
            return {};
        }
        ++sample_index;
    }

    return samples;
}

} // namespace navkit::sim
