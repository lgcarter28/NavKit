// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/time/RationalRate.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <string>
#include <string_view>

namespace navkit::app_support
{

inline void validate_runtime_rate(const nlohmann::json& cfg, std::string_view object_name)
{
    detail::require_optional_positive_number(cfg, "dt_s");
    detail::require_optional_positive_number(cfg, "rate_hz");
    if (cfg.contains("dt_s") && cfg.contains("rate_hz")) {
        detail::throw_runtime_config_error(std::string{object_name} +
                                           " must specify only one of 'dt_s' or 'rate_hz'");
    }
}

[[nodiscard]] inline core::Time_t dt_s_from_required_runtime_rate(const nlohmann::json& cfg,
                                                                  std::string_view object_name)
{
    validate_runtime_rate(cfg, object_name);
    if (cfg.contains("dt_s")) {
        return cfg.at("dt_s").get<core::Time_t>();
    }
    if (cfg.contains("rate_hz")) {
        return 1.0 / cfg.at("rate_hz").get<core::Time_t>();
    }
    detail::throw_runtime_config_error(std::string{object_name} +
                                       " must specify one of 'dt_s' or 'rate_hz'");
}

namespace detail
{

struct RationalValue
{
    std::uint64_t numerator{};
    std::uint64_t denominator{1U};
};

[[nodiscard]] inline RationalValue rational_value_from_positive_scalar(const core::Time_t value,
                                                                       std::string_view object_name)
{
    constexpr std::uint64_t max_denominator = 1'000'000U;
    constexpr int max_iterations = 32;
    constexpr core::Time_t tolerance = 1.0e-12;

    if (!std::isfinite(value) || value <= 0.0) {
        throw_runtime_config_error(std::string{object_name} + " must be finite and positive");
    }

    core::Time_t remainder = value;
    std::uint64_t numerator_previous = 0U;
    std::uint64_t denominator_previous = 1U;
    std::uint64_t numerator_current = 1U;
    std::uint64_t denominator_current = 0U;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        const core::Time_t integer_part_scalar = std::floor(remainder);
        if (integer_part_scalar >=
            static_cast<core::Time_t>(std::numeric_limits<std::uint64_t>::max())) {
            break;
        }
        const std::uint64_t integer_part = static_cast<std::uint64_t>(integer_part_scalar);
        if ((integer_part > 0U &&
             numerator_current > ((std::numeric_limits<std::uint64_t>::max() - numerator_previous) /
                                  integer_part)) ||
            (integer_part > 0U &&
             denominator_current >
                 ((std::numeric_limits<std::uint64_t>::max() - denominator_previous) /
                  integer_part))) {
            break;
        }
        const std::uint64_t numerator_next =
            (integer_part * numerator_current) + numerator_previous;
        const std::uint64_t denominator_next =
            (integer_part * denominator_current) + denominator_previous;
        if (denominator_next == 0U || denominator_next > max_denominator) {
            break;
        }

        const core::Time_t approximation =
            static_cast<core::Time_t>(numerator_next) / static_cast<core::Time_t>(denominator_next);
        numerator_previous = numerator_current;
        denominator_previous = denominator_current;
        numerator_current = numerator_next;
        denominator_current = denominator_next;
        if (std::abs(approximation - value) <= (tolerance * std::max(value, 1.0))) {
            break;
        }

        const core::Time_t fractional_part = remainder - integer_part_scalar;
        if (fractional_part <= tolerance) {
            break;
        }
        remainder = 1.0 / fractional_part;
    }

    if (numerator_current == 0U || denominator_current == 0U) {
        throw_runtime_config_error(std::string{object_name} + " cannot be represented as a rate");
    }

    const std::uint64_t divisor = std::gcd(numerator_current, denominator_current);
    return {.numerator = numerator_current / divisor, .denominator = denominator_current / divisor};
}

[[nodiscard]] inline core::Samples samples_from_rational_value(const std::uint64_t value,
                                                               std::string_view object_name)
{
    if (value > std::numeric_limits<core::Samples>::max()) {
        throw_runtime_config_error(std::string{object_name} +
                                   " is too large to represent as samples");
    }
    return static_cast<core::Samples>(value);
}

} // namespace detail

/**
 * Parses a runtime rate into its canonical integer samples-per-second form.
 *
 * Prefer `rate_hz` for rates with non-terminating decimal periods. `dt_s` remains
 * supported for compatibility and is converted once during configuration parsing.
 */
[[nodiscard]] inline core::RationalRate
rational_rate_from_required_runtime_rate(const nlohmann::json& cfg, std::string_view object_name)
{
    validate_runtime_rate(cfg, object_name);
    if (cfg.contains("rate_hz")) {
        const detail::RationalValue rate = detail::rational_value_from_positive_scalar(
            cfg.at("rate_hz").get<core::Time_t>(), std::string{object_name} + ".rate_hz");
        return {.samples = detail::samples_from_rational_value(rate.numerator, object_name),
                .s = rate.denominator};
    }
    if (cfg.contains("dt_s")) {
        const detail::RationalValue period = detail::rational_value_from_positive_scalar(
            cfg.at("dt_s").get<core::Time_t>(), std::string{object_name} + ".dt_s");
        return {.samples = detail::samples_from_rational_value(period.denominator, object_name),
                .s = period.numerator};
    }
    detail::throw_runtime_config_error(std::string{object_name} +
                                       " must specify one of 'dt_s' or 'rate_hz'");
}

} // namespace navkit::app_support
