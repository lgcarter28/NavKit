// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/RationalRate.hpp"

#include <limits>

namespace navkit::core
{

/**
 * Tracks exact rational cadence for consumer-side due-time gating.
 * Consumer polling with due() consumes skipped periods without accumulating phase error.
 */
class RationalSchedule
{
public:
    constexpr RationalSchedule() = default;

    constexpr RationalSchedule(const Timestamp t_epoch, const RationalRate rate)
        : m_t_epoch(t_epoch)
        , m_rate(rate)
        , m_initialized(rational_cadence_is_valid(t_epoch, rate))
    {}

    /** Returns whether a phase reference and cadence form a valid schedule. */
    [[nodiscard]] static constexpr bool is_valid(const Timestamp& t_epoch, const RationalRate& rate)
    {
        return rational_cadence_is_valid(t_epoch, rate);
    }

    /** Starts an uninitialized schedule at its phase reference timestamp. */
    [[nodiscard]] constexpr bool initialize(const Timestamp t_epoch, const RationalRate rate)
    {
        if (m_initialized || !is_valid(t_epoch, rate)) {
            return false;
        }
        m_t_epoch = t_epoch;
        m_rate = rate;
        m_next_due_sample_index = 0U;
        m_initialized = true;
        return true;
    }

    /** Explicitly restarts a schedule with a new phase reference or cadence. */
    [[nodiscard]] constexpr bool reset(const Timestamp t_epoch, const RationalRate rate)
    {
        if (!is_valid(t_epoch, rate)) {
            return false;
        }
        m_t_epoch = t_epoch;
        m_rate = rate;
        m_next_due_sample_index = 0U;
        m_initialized = true;
        return true;
    }

    /** Returns whether a configured schedule is currently active. */
    [[nodiscard]] constexpr bool is_initialized() const
    {
        return m_initialized;
    }

    /** Consumes every cadence event due at the supplied timestamp. */
    [[nodiscard]] constexpr bool due(const Timestamp& t)
    {
        if (!m_initialized || !is_valid(m_t_epoch, m_rate)) {
            return false;
        }
        if (!timestamp_is_valid(t) || t.scale != m_t_epoch.scale || timestamp_less(t, m_t_epoch)) {
            return false;
        }

        Timestamp t_due{};
        if (!timestamp_at_sample_index(m_t_epoch, m_rate, m_next_due_sample_index, t_due)) {
            return false;
        }
        if (timestamp_less(t, t_due)) {
            return false;
        }

        do {
            if (m_next_due_sample_index == std::numeric_limits<SampleIndex>::max()) {
                return false;
            }
            ++m_next_due_sample_index;
            if (!timestamp_at_sample_index(m_t_epoch, m_rate, m_next_due_sample_index, t_due)) {
                return false;
            }
        } while (!timestamp_less(t, t_due));
        return true;
    }

private:
    Timestamp m_t_epoch{};
    RationalRate m_rate{};
    SampleIndex m_next_due_sample_index{};
    bool m_initialized{false};
};

} // namespace navkit::core
