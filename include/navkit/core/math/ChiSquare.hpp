// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace navkit::core::math
{

namespace detail
{

inline constexpr std::size_t gamma_max_iterations = 256U;
inline constexpr std::size_t quantile_bracket_max_iterations = 64U;
inline constexpr std::size_t quantile_bisection_max_iterations = 192U;

/** Evaluate the regularized lower incomplete gamma function P(a, x). */
[[nodiscard]] inline bool
regularized_gamma_p(const Scalar_t a, const Scalar_t x, Scalar_t& probability)
{
    if (!std::isfinite(a) || !std::isfinite(x) || a <= 0.0 || x < 0.0) {
        return false;
    }
    if (x == 0.0) {
        probability = 0.0;
        return true;
    }

    constexpr Scalar_t convergence_tolerance = 32.0 * std::numeric_limits<Scalar_t>::epsilon();
    constexpr Scalar_t minimum_denominator =
        std::numeric_limits<Scalar_t>::min() / convergence_tolerance;
    const Scalar_t logarithmic_scale = (a * std::log(x)) - x - std::lgamma(a);
    if (!std::isfinite(logarithmic_scale)) {
        return false;
    }

    Scalar_t result{};
    if (x < a + 1.0) {
        Scalar_t series_term = 1.0 / a;
        Scalar_t series_sum = series_term;
        Scalar_t denominator = a;
        bool converged = false;
        for (std::size_t iteration = 1U; iteration <= gamma_max_iterations; ++iteration) {
            denominator += 1.0;
            series_term *= x / denominator;
            series_sum += series_term;
            if (std::abs(series_term) <= std::abs(series_sum) * convergence_tolerance) {
                converged = true;
                break;
            }
        }
        if (!converged) {
            return false;
        }
        result = series_sum * std::exp(logarithmic_scale);
    }
    else {
        Scalar_t continued_fraction_b = x + 1.0 - a;
        if (std::abs(continued_fraction_b) < minimum_denominator) {
            continued_fraction_b = minimum_denominator;
        }
        Scalar_t continued_fraction_c = 1.0 / minimum_denominator;
        Scalar_t continued_fraction_d = 1.0 / continued_fraction_b;
        Scalar_t continued_fraction = continued_fraction_d;
        bool converged = false;
        for (std::size_t iteration = 1U; iteration <= gamma_max_iterations; ++iteration) {
            const Scalar_t iteration_value = static_cast<Scalar_t>(iteration);
            const Scalar_t numerator = -iteration_value * (iteration_value - a);
            continued_fraction_b += 2.0;
            continued_fraction_d = (numerator * continued_fraction_d) + continued_fraction_b;
            if (std::abs(continued_fraction_d) < minimum_denominator) {
                continued_fraction_d = minimum_denominator;
            }
            continued_fraction_c = continued_fraction_b + (numerator / continued_fraction_c);
            if (std::abs(continued_fraction_c) < minimum_denominator) {
                continued_fraction_c = minimum_denominator;
            }
            continued_fraction_d = 1.0 / continued_fraction_d;
            const Scalar_t multiplier = continued_fraction_d * continued_fraction_c;
            continued_fraction *= multiplier;
            if (std::abs(multiplier - 1.0) <= convergence_tolerance) {
                converged = true;
                break;
            }
        }
        if (!converged) {
            return false;
        }
        const Scalar_t upper_tail = std::exp(logarithmic_scale) * continued_fraction;
        result = 1.0 - upper_tail;
    }

    if (!std::isfinite(result)) {
        return false;
    }
    probability = std::clamp(result, Scalar_t{0.0}, Scalar_t{1.0});
    return true;
}

} // namespace detail

/**
 * Evaluate the chi-square cumulative distribution function.
 *
 * @param statistic Nonnegative chi-square statistic.
 * @param degrees_of_freedom Positive integer number of degrees of freedom.
 * @param probability Output cumulative probability in [0, 1]. Unchanged on failure.
 * @return True when the inputs and bounded numerical evaluation are valid.
 */
[[nodiscard]] inline bool chi_square_cdf(const Scalar_t statistic,
                                         const std::uint32_t degrees_of_freedom,
                                         Scalar_t& probability)
{
    if (!std::isfinite(statistic) || statistic < 0.0 || degrees_of_freedom == 0U) {
        return false;
    }

    Scalar_t result{};
    if (!detail::regularized_gamma_p(
            0.5 * static_cast<Scalar_t>(degrees_of_freedom), 0.5 * statistic, result)) {
        return false;
    }
    probability = result;
    return true;
}

/**
 * Invert the chi-square CDF using deterministic bracketing and bisection.
 *
 * @param probability Cumulative probability in [0, 1). A zero probability maps to zero.
 * @param degrees_of_freedom Positive integer number of degrees of freedom.
 * @param statistic Output nonnegative chi-square statistic. Unchanged on failure.
 * @return True when the inputs are valid and the bounded inversion converges.
 */
[[nodiscard]] inline bool chi_square_quantile(const Scalar_t probability,
                                              const std::uint32_t degrees_of_freedom,
                                              Scalar_t& statistic)
{
    if (!std::isfinite(probability) || probability < 0.0 || probability >= 1.0 ||
        degrees_of_freedom == 0U) {
        return false;
    }
    if (probability == 0.0) {
        statistic = 0.0;
        return true;
    }

    Scalar_t lower_bound = 0.0;
    Scalar_t upper_bound = std::max(Scalar_t{1.0}, static_cast<Scalar_t>(degrees_of_freedom));
    Scalar_t upper_probability{};
    bool bracketed = false;
    for (std::size_t iteration = 0U; iteration < detail::quantile_bracket_max_iterations;
         ++iteration) {
        if (!chi_square_cdf(upper_bound, degrees_of_freedom, upper_probability)) {
            return false;
        }
        if (upper_probability >= probability) {
            bracketed = true;
            break;
        }
        if (upper_bound > std::numeric_limits<Scalar_t>::max() / 2.0) {
            return false;
        }
        upper_bound *= 2.0;
    }
    if (!bracketed) {
        return false;
    }

    constexpr Scalar_t interval_tolerance = 64.0 * std::numeric_limits<Scalar_t>::epsilon();
    for (std::size_t iteration = 0U; iteration < detail::quantile_bisection_max_iterations;
         ++iteration) {
        const Scalar_t midpoint = lower_bound + (0.5 * (upper_bound - lower_bound));
        Scalar_t midpoint_probability{};
        if (!chi_square_cdf(midpoint, degrees_of_freedom, midpoint_probability)) {
            return false;
        }
        if (midpoint_probability < probability) {
            lower_bound = midpoint;
        }
        else {
            upper_bound = midpoint;
        }

        const Scalar_t scale = std::max(std::abs(midpoint), std::numeric_limits<Scalar_t>::min());
        if ((upper_bound - lower_bound) <= interval_tolerance * scale) {
            statistic = lower_bound + (0.5 * (upper_bound - lower_bound));
            return true;
        }
    }
    return false;
}

} // namespace navkit::core::math
