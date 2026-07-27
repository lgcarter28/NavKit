// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"

#include <array>
#include <limits>
#include <numeric>

namespace navkit::core
{

/** A positive sample rate represented as samples per second. */
struct RationalRate
{
    Samples samples{};
    Seconds s{1U};

    [[nodiscard]] constexpr bool operator==(const RationalRate&) const = default;
};

[[nodiscard]] constexpr bool rational_rate_is_valid(const RationalRate& rate)
{
    return rate.samples > 0U && rate.s > 0U;
}

/** Returns whether a timestamp epoch and rational cadence form a valid timebase. */
[[nodiscard]] constexpr bool rational_cadence_is_valid(const Timestamp& t_epoch,
                                                       const RationalRate& rate)
{
    return timestamp_is_valid(t_epoch) && rational_rate_is_valid(rate);
}

/** Returns whether faster_rate's cadence exactly contains every slower-rate event. */
[[nodiscard]] constexpr bool rational_rate_is_integer_multiple(const RationalRate& faster_rate,
                                                               const RationalRate& slower_rate)
{
    if (!rational_rate_is_valid(faster_rate) || !rational_rate_is_valid(slower_rate)) {
        return false;
    }

    std::array<std::uint64_t, 2U> numerator{
        faster_rate.samples,
        slower_rate.s,
    };
    std::array<std::uint64_t, 2U> denominator{
        faster_rate.s,
        slower_rate.samples,
    };
    for (std::uint64_t& denominator_factor : denominator) {
        for (std::uint64_t& numerator_factor : numerator) {
            const std::uint64_t divisor = std::gcd(numerator_factor, denominator_factor);
            numerator_factor /= divisor;
            denominator_factor /= divisor;
        }
    }
    return denominator.at(0U) == 1U && denominator.at(1U) == 1U;
}

/**
 * Materializes a timestamp at a rational-rate sample index without repeatedly adding
 * an approximate floating-point period.
 */
[[nodiscard]] constexpr bool timestamp_at_sample_index(const Timestamp& t_epoch,
                                                       const RationalRate& rate,
                                                       const SampleIndex sample_index,
                                                       Timestamp& t)
{
    t = {};
    if (!timestamp_is_valid(t_epoch) || !rational_rate_is_valid(rate)) {
        return false;
    }

    const SampleIndex complete_rate_cycles = sample_index / rate.samples;
    const SampleIndex remaining_samples = sample_index % rate.samples;
    if (complete_rate_cycles > ((std::numeric_limits<Seconds>::max() - t_epoch.s) / rate.s)) {
        return false;
    }

    const Seconds cycle_seconds = complete_rate_cycles * rate.s;
    if (remaining_samples > (std::numeric_limits<Seconds>::max() / rate.s)) {
        return false;
    }
    const Seconds remainder_numerator = remaining_samples * rate.s;
    const Seconds remainder_seconds = remainder_numerator / rate.samples;
    const Seconds remainder_fraction = remainder_numerator % rate.samples;
    const Seconds fractional_nanoseconds =
        ((remainder_fraction * nanoseconds_per_second) + (rate.samples / 2U)) / rate.samples;

    if (cycle_seconds > (std::numeric_limits<Seconds>::max() - t_epoch.s) ||
        remainder_seconds > (std::numeric_limits<Seconds>::max() - t_epoch.s - cycle_seconds)) {
        return false;
    }

    t = t_epoch;
    t.s += cycle_seconds + remainder_seconds;
    t.ns += static_cast<Nanoseconds>(fractional_nanoseconds);
    if (t.ns >= nanoseconds_per_second) {
        if (t.s == std::numeric_limits<Seconds>::max()) {
            t = {};
            return false;
        }
        ++t.s;
        t.ns -= nanoseconds_per_second;
    }
    return true;
}

} // namespace navkit::core
