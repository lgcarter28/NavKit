// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/TabulatedTrajectorySource.hpp"

#include "navkit/core/time/Timestamp.hpp"

#include <utility>

namespace navkit::sim
{

TabulatedTrajectorySource::TabulatedTrajectorySource(TruthTrajectory trajectory)
    : m_trajectory(std::move(trajectory))
{}

bool TabulatedTrajectorySource::advance_to(const core::Timestamp& t)
{
    if (m_trajectory.empty() || !core::timestamp_is_valid(t) ||
        t.scale != m_trajectory.first().t.scale ||
        core::timestamp_less(t, m_trajectory.first().t)) {
        return false;
    }
    if (core::timestamp_less(m_trajectory.last().t, t)) {
        m_complete = true;
        return false;
    }

    m_t_available = t;
    m_initialized = true;
    m_complete = (t == m_trajectory.last().t);
    return true;
}

bool TabulatedTrajectorySource::query(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    if (!m_initialized || core::timestamp_less(m_t_available, t)) {
        return false;
    }
    return m_trajectory.sample_at(t, sample);
}

bool TabulatedTrajectorySource::query_diagnostics(const core::Timestamp& t,
                                                  TrajectoryDiagnostics& diagnostics) const
{
    diagnostics = {};
    if (!m_initialized || core::timestamp_less(m_t_available, t)) {
        return false;
    }
    return m_trajectory.diagnostics_at(t, diagnostics);
}

core::Timestamp TabulatedTrajectorySource::t_start() const
{
    return m_trajectory.empty() ? core::Timestamp{} : m_trajectory.first().t;
}

core::Timestamp TabulatedTrajectorySource::t_end() const
{
    return m_trajectory.empty() ? core::Timestamp{} : m_trajectory.last().t;
}

bool TabulatedTrajectorySource::is_complete() const
{
    return m_complete;
}

} // namespace navkit::sim
