// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/TruthTrajectory.hpp"

#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"

#include <algorithm>
#include <utility>

namespace navkit::sim
{

TruthTrajectory::TruthTrajectory(std::vector<TruthSample> samples)
    : m_samples(std::move(samples))
{}

bool TruthTrajectory::empty() const
{
    return m_samples.empty();
}

std::size_t TruthTrajectory::size() const
{
    return m_samples.size();
}

const TruthSample& TruthTrajectory::first() const
{
    return m_samples.front();
}

const TruthSample& TruthTrajectory::last() const
{
    return m_samples.back();
}

const std::vector<TruthSample>& TruthTrajectory::samples() const
{
    return m_samples;
}

bool TruthTrajectory::sample_at(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    if (m_samples.empty() || !core::timestamp_is_valid(t) || t.scale != m_samples.front().t.scale ||
        core::timestamp_less(t, m_samples.front().t) ||
        core::timestamp_less(m_samples.back().t, t)) {
        return false;
    }

    const std::vector<TruthSample>::const_iterator upper =
        std::lower_bound(m_samples.begin(),
                         m_samples.end(),
                         t,
                         [](const TruthSample& candidate, const core::Timestamp& query) {
                             return core::timestamp_less(candidate.t, query);
                         });
    if (upper == m_samples.end()) {
        return false;
    }
    if (upper->t == t) {
        sample = *upper;
        return true;
    }
    if (upper == m_samples.begin()) {
        return false;
    }

    const TruthSample& previous = *std::prev(upper);
    const TruthSample& current = *upper;
    core::Duration full_duration{};
    core::Duration query_duration{};
    if (!core::elapsed_time(current.t, previous.t, full_duration) ||
        !core::elapsed_time(t, previous.t, query_duration)) {
        return false;
    }
    const core::Time_t full_dt_s = core::duration_seconds(full_duration);
    if (full_dt_s <= 0.0) {
        return false;
    }
    const core::Scalar_t alpha = core::duration_seconds(query_duration) / full_dt_s;

    sample.t = t;
    sample.p_e = previous.p_e + (alpha * (current.p_e - previous.p_e));
    sample.v_e = previous.v_e + (alpha * (current.v_e - previous.v_e));
    sample.q_b2e =
        core::math::normalized_with_positive_scalar(previous.q_b2e.slerp(alpha, current.q_b2e));
    sample.w_ib_b_radps =
        previous.w_ib_b_radps + (alpha * (current.w_ib_b_radps - previous.w_ib_b_radps));
    return true;
}

} // namespace navkit::sim
