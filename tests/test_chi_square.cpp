// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/math/ChiSquare.hpp"
#include "test_main.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace navkit::core::math::test
{

TEST_CASE("Chi-square CDF handles boundaries and an analytic two-DOF case")
{
    Scalar_t probability = -1.0;
    REQUIRE(chi_square_cdf(0.0, 1U, probability));
    CHECK(probability == doctest::Approx(0.0));

    REQUIRE(chi_square_cdf(2.0, 2U, probability));
    CHECK(probability == doctest::Approx(1.0 - std::exp(-1.0)).epsilon(1.0e-13));

    REQUIRE(chi_square_cdf(1000.0, 2U, probability));
    CHECK(probability == doctest::Approx(1.0));
}

TEST_CASE("Chi-square quantile agrees with published reference values")
{
    struct ReferenceValue
    {
        std::uint32_t degrees_of_freedom;
        Scalar_t probability;
        Scalar_t statistic;
    };
    constexpr std::array<ReferenceValue, 4> references{{
        {.degrees_of_freedom = 1U, .probability = 0.95, .statistic = 3.841458820694124},
        {.degrees_of_freedom = 2U, .probability = 0.95, .statistic = 5.991464547107979},
        {.degrees_of_freedom = 3U, .probability = 0.95, .statistic = 7.814727903251179},
        {.degrees_of_freedom = 10U, .probability = 0.99, .statistic = 23.20925115895436},
    }};

    for (const ReferenceValue& reference : references) {
        Scalar_t statistic{};
        REQUIRE(
            chi_square_quantile(reference.probability, reference.degrees_of_freedom, statistic));
        CHECK(statistic == doctest::Approx(reference.statistic).epsilon(1.0e-11));
    }
}

TEST_CASE("Chi-square CDF and quantile round trip across dimensions and tails")
{
    constexpr std::array<std::uint32_t, 5> dimensions{1U, 2U, 3U, 15U, 64U};
    constexpr std::array<Scalar_t, 5> probabilities{0.001, 0.1, 0.5, 0.95, 0.999};

    for (const std::uint32_t degrees_of_freedom : dimensions) {
        for (const Scalar_t expected_probability : probabilities) {
            Scalar_t statistic{};
            REQUIRE(chi_square_quantile(expected_probability, degrees_of_freedom, statistic));
            Scalar_t actual_probability{};
            REQUIRE(chi_square_cdf(statistic, degrees_of_freedom, actual_probability));
            CHECK(actual_probability == doctest::Approx(expected_probability).epsilon(1.0e-10));
        }
    }
}

TEST_CASE("Chi-square quantile resolves a small lower-tail probability")
{
    constexpr Scalar_t probability = 1.0e-9;
    constexpr Scalar_t expected_statistic = 1.5707963267948967e-18;

    Scalar_t statistic{};
    REQUIRE(chi_square_quantile(probability, 1U, statistic));
    CHECK(statistic == doctest::Approx(expected_statistic).epsilon(1.0e-8));

    Scalar_t recovered_probability{};
    REQUIRE(chi_square_cdf(statistic, 1U, recovered_probability));
    CHECK(recovered_probability == doctest::Approx(probability).epsilon(1.0e-10));
}

TEST_CASE("Chi-square utilities reject invalid inputs without changing outputs")
{
    constexpr Scalar_t sentinel = 42.0;
    Scalar_t output = sentinel;

    CHECK_FALSE(chi_square_cdf(-1.0, 1U, output));
    CHECK(output == sentinel);
    CHECK_FALSE(chi_square_cdf(1.0, 0U, output));
    CHECK(output == sentinel);
    CHECK_FALSE(chi_square_cdf(std::numeric_limits<Scalar_t>::quiet_NaN(), 1U, output));
    CHECK(output == sentinel);

    CHECK_FALSE(chi_square_quantile(-0.1, 1U, output));
    CHECK(output == sentinel);
    CHECK_FALSE(chi_square_quantile(1.0, 1U, output));
    CHECK(output == sentinel);
    CHECK_FALSE(chi_square_quantile(0.5, 0U, output));
    CHECK(output == sentinel);
    CHECK_FALSE(chi_square_quantile(std::numeric_limits<Scalar_t>::quiet_NaN(), 1U, output));
    CHECK(output == sentinel);

    REQUIRE(chi_square_quantile(0.0, 1U, output));
    CHECK(output == doctest::Approx(0.0));
}

} // namespace navkit::core::math::test
