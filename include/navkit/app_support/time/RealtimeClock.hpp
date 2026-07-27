// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/time/Clock.hpp"
#include "navkit/core/time/Duration.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <thread>

namespace navkit::app_support
{

/** Maps planned monotonic time to steady-clock deadlines for future HWIL use. */
class RealtimeClock final : public Clock
{
public:
    [[nodiscard]] bool initialize(const core::Timestamp& t_epoch) override
    {
        if (!core::timestamp_is_valid(t_epoch)) {
            return false;
        }
        m_t_epoch = t_epoch;
        m_t = t_epoch;
        m_wall_epoch = std::chrono::steady_clock::now();
        m_initialized = true;
        return true;
    }

    [[nodiscard]] bool wait_until(const core::Timestamp& t) override
    {
        if (!m_initialized || !core::timestamp_is_valid(t) || t.scale != m_t_epoch.scale ||
            core::timestamp_less(t, m_t)) {
            return false;
        }

        core::Duration elapsed{};
        if (!core::elapsed_time(t, m_t_epoch, elapsed) ||
            elapsed.s > static_cast<core::Seconds>(std::numeric_limits<std::int64_t>::max() /
                                                   core::nanoseconds_per_second)) {
            return false;
        }
        const std::int64_t elapsed_nanoseconds =
            (static_cast<std::int64_t>(elapsed.s) * core::nanoseconds_per_second) + elapsed.ns;
        std::this_thread::sleep_until(m_wall_epoch + std::chrono::nanoseconds{elapsed_nanoseconds});
        m_t = t;
        return true;
    }

    [[nodiscard]] core::Timestamp now() const override
    {
        return m_t;
    }

private:
    core::Timestamp m_t_epoch{};
    core::Timestamp m_t{};
    std::chrono::steady_clock::time_point m_wall_epoch{};
    bool m_initialized{false};
};

} // namespace navkit::app_support
