// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/time/TimeTypes.hpp"

#include <cmath>
#include <limits>

namespace navkit::core
{

inline constexpr std::uint32_t timestamp_version = 1U;
inline constexpr Nanoseconds nanoseconds_per_second = 1'000'000'000U;

/** Declares the epoch convention used by a timestamp source. */
enum class TimeScale
{
    Monotonic,
    Utc,
    Gps,
    Tai,
};

/**
 * A versioned absolute time point suitable for public sensor and telemetry APIs.
 *
 * `s` and `ns` are serialized as distinct fields; callers must not serialize the
 * native in-memory representation directly.
 */
struct Timestamp
{
    std::uint32_t version{timestamp_version};
    TimeScale scale{TimeScale::Monotonic};
    Seconds s{};
    Nanoseconds ns{};

    [[nodiscard]] constexpr bool operator==(const Timestamp&) const = default;
};

[[nodiscard]] constexpr bool timestamp_is_valid(const Timestamp& t)
{
    return t.version == timestamp_version && t.ns < nanoseconds_per_second;
}

[[nodiscard]] constexpr bool timestamp_less(const Timestamp& lhs, const Timestamp& rhs)
{
    if (lhs.scale != rhs.scale) {
        return false;
    }
    return (lhs.s < rhs.s) || ((lhs.s == rhs.s) && (lhs.ns < rhs.ns));
}

/**
 * Converts an absolute timestamp to scalar seconds for presentation or
 * simulation-only logging. Navigation algorithms should calculate an elapsed
 * duration first instead of subtracting two converted absolute timestamps.
 */
[[nodiscard]] constexpr Time_t timestamp_seconds(const Timestamp& t)
{
    return static_cast<Time_t>(t.s) +
           (static_cast<Time_t>(t.ns) / static_cast<Time_t>(nanoseconds_per_second));
}

/**
 * Converts non-negative scalar seconds at a configuration or presentation boundary
 * into a versioned timestamp. Hot-path algorithms should receive timestamps directly.
 */
[[nodiscard]] inline bool
timestamp_from_seconds(const Time_t seconds, const TimeScale scale, Timestamp& t)
{
    t = {};
    if (!std::isfinite(seconds) || seconds < 0.0 ||
        seconds > static_cast<Time_t>(std::numeric_limits<Seconds>::max())) {
        return false;
    }

    const Time_t whole_seconds = std::floor(seconds);
    const Time_t fractional_seconds = seconds - whole_seconds;
    const Time_t rounded_nanoseconds =
        std::round(fractional_seconds * static_cast<Time_t>(nanoseconds_per_second));
    t.scale = scale;
    t.s = static_cast<Seconds>(whole_seconds);
    t.ns = static_cast<Nanoseconds>(rounded_nanoseconds);
    if (t.ns >= nanoseconds_per_second) {
        if (t.s == std::numeric_limits<Seconds>::max()) {
            t = {};
            return false;
        }
        ++t.s;
        t.ns = 0U;
    }
    return true;
}

} // namespace navkit::core
