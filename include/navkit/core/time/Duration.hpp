// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"

namespace navkit::core
{

/** A non-negative elapsed interval with exact integral-second and nanosecond fields. */
struct Duration
{
    Seconds s{};
    Nanoseconds ns{};

    [[nodiscard]] constexpr bool operator==(const Duration&) const = default;
};

[[nodiscard]] constexpr bool duration_is_valid(const Duration& dt)
{
    return dt.ns < nanoseconds_per_second;
}

/**
 * Calculates a validated non-negative interval without floating-point subtraction.
 *
 * Returns false for unsupported timestamp versions, invalid nanoseconds, differing
 * time scales, or non-monotonic input. `dt` is cleared on failure.
 */
[[nodiscard]] constexpr bool
elapsed_time(const Timestamp& t_end, const Timestamp& t_start, Duration& dt)
{
    dt = {};
    if (!timestamp_is_valid(t_end) || !timestamp_is_valid(t_start) ||
        t_end.scale != t_start.scale || timestamp_less(t_end, t_start)) {
        return false;
    }

    dt.s = t_end.s - t_start.s;
    if (t_end.ns >= t_start.ns) {
        dt.ns = t_end.ns - t_start.ns;
        return true;
    }
    if (dt.s == 0U) {
        return false;
    }
    --dt.s;
    dt.ns = nanoseconds_per_second + t_end.ns - t_start.ns;
    return true;
}

/** Converts an exact non-negative interval to the scalar-seconds math representation. */
[[nodiscard]] constexpr Time_t duration_seconds(const Duration& dt)
{
    return static_cast<Time_t>(dt.s) +
           (static_cast<Time_t>(dt.ns) / static_cast<Time_t>(nanoseconds_per_second));
}

} // namespace navkit::core
