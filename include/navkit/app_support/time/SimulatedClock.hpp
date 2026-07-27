// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/time/Clock.hpp"

namespace navkit::app_support
{

/** Immediately adopts planned time for deterministic desktop simulation. */
class SimulatedClock final : public Clock
{
public:
    [[nodiscard]] bool initialize(const core::Timestamp& t_epoch) override
    {
        if (!core::timestamp_is_valid(t_epoch)) {
            return false;
        }
        m_t = t_epoch;
        m_initialized = true;
        return true;
    }

    [[nodiscard]] bool wait_until(const core::Timestamp& t) override
    {
        if (!m_initialized || !core::timestamp_is_valid(t) || t.scale != m_t.scale ||
            core::timestamp_less(t, m_t)) {
            return false;
        }
        m_t = t;
        return true;
    }

    [[nodiscard]] core::Timestamp now() const override
    {
        return m_t;
    }

private:
    core::Timestamp m_t{};
    bool m_initialized{false};
};

} // namespace navkit::app_support
