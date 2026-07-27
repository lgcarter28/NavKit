// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/StationaryTrajectorySource.hpp"

#include "navkit/core/time/Duration.hpp"

namespace navkit::sim
{

StationaryTrajectorySource::StationaryTrajectorySource(const StationaryTrajectoryConfig& cfg)
    : m_cfg(cfg)
    , m_valid(core::timestamp_is_valid(cfg.t_epoch) && core::rational_rate_is_valid(cfg.rate) &&
              cfg.duration_s >= 0.0)
{}

bool StationaryTrajectorySource::advance_to(const core::Timestamp& t)
{
    if (!m_valid || !timestamp_is_in_source_range(t)) {
        if (m_valid && core::timestamp_less(t_end(), t)) {
            m_complete = true;
        }
        return false;
    }

    m_t_available = t;
    m_initialized = true;
    core::Duration elapsed{};
    if (!core::elapsed_time(t, m_cfg.t_epoch, elapsed)) {
        return false;
    }
    m_complete = core::duration_seconds(elapsed) >= m_cfg.duration_s;
    return true;
}

bool StationaryTrajectorySource::query(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    if (!m_initialized || !timestamp_is_in_source_range(t) ||
        core::timestamp_less(m_t_available, t)) {
        return false;
    }

    sample.t = t;
    sample.p_e = m_cfg.p_e;
    sample.v_e = m_cfg.v_e;
    sample.q_b2e = m_cfg.q_b2e;
    sample.w_ib_b_radps = m_cfg.w_ib_b_radps;
    return true;
}

core::Timestamp StationaryTrajectorySource::t_start() const
{
    return m_cfg.t_epoch;
}

core::Timestamp StationaryTrajectorySource::t_end() const
{
    if (!m_valid) {
        return {};
    }

    core::Timestamp t{};
    if (!core::timestamp_from_seconds(
            core::timestamp_seconds(m_cfg.t_epoch) + m_cfg.duration_s, m_cfg.t_epoch.scale, t)) {
        return {};
    }
    return t;
}

bool StationaryTrajectorySource::is_complete() const
{
    return m_complete;
}

bool StationaryTrajectorySource::timestamp_is_in_source_range(const core::Timestamp& t) const
{
    return core::timestamp_is_valid(t) && t.scale == m_cfg.t_epoch.scale &&
           !core::timestamp_less(t, m_cfg.t_epoch) && !core::timestamp_less(t_end(), t);
}

} // namespace navkit::sim
