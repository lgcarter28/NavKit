// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/RationalRate.hpp"

#include <limits>

namespace navkit::core
{

/** Produces exact planned timestamps at a configured rational cadence. */
class RationalTimeline
{
public:
    constexpr RationalTimeline() = default;

    constexpr RationalTimeline(const Timestamp t_epoch, const RationalRate rate)
        : m_t_epoch(t_epoch)
        , m_rate(rate)
        , m_initialized(rational_cadence_is_valid(t_epoch, rate))
    {}

    /** Starts an uninitialized timeline at its phase reference timestamp. */
    [[nodiscard]] constexpr bool initialize(const Timestamp t_epoch, const RationalRate rate)
    {
        if (m_initialized || !rational_cadence_is_valid(t_epoch, rate)) {
            return false;
        }
        m_t_epoch = t_epoch;
        m_rate = rate;
        m_next_sample_index = 1U;
        m_initialized = true;
        return true;
    }

    /** Explicitly restarts a timeline with a new phase reference or cadence. */
    [[nodiscard]] constexpr bool reset(const Timestamp t_epoch, const RationalRate rate)
    {
        if (!rational_cadence_is_valid(t_epoch, rate)) {
            return false;
        }
        m_t_epoch = t_epoch;
        m_rate = rate;
        m_next_sample_index = 1U;
        m_initialized = true;
        return true;
    }

    /** Returns whether the timeline has an initialized cadence and epoch. */
    [[nodiscard]] constexpr bool is_initialized() const
    {
        return m_initialized;
    }

    /** Produces the next exact cadence timestamp strictly after the epoch. */
    [[nodiscard]] constexpr bool next(Timestamp& t)
    {
        t = {};
        if (!m_initialized || !rational_cadence_is_valid(m_t_epoch, m_rate) ||
            m_next_sample_index == 0U) {
            return false;
        }
        if (!timestamp_at_sample_index(m_t_epoch, m_rate, m_next_sample_index, t)) {
            return false;
        }
        if (m_next_sample_index == std::numeric_limits<SampleIndex>::max()) {
            t = {};
            return false;
        }
        ++m_next_sample_index;
        return true;
    }

private:
    Timestamp m_t_epoch{};
    RationalRate m_rate{};
    SampleIndex m_next_sample_index{1U};
    bool m_initialized{false};
};

} // namespace navkit::core
