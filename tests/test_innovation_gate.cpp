// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/sensor/InnovationGate.hpp"
#include "test_main.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace navkit::core::estimation::test
{

TEST_CASE("Innovation gate rejects invalid probabilities without changing configuration")
{
    InnovationGate<3> gate{};
    REQUIRE(gate.configure_probability(0.95));
    const Scalar_t original_probability = gate.probability();
    const Scalar_t original_threshold = gate.threshold();

    constexpr std::array<Scalar_t, 4> invalid_finite_probabilities{-0.1, 0.0, 1.0, 1.1};
    for (const Scalar_t probability : invalid_finite_probabilities) {
        CHECK_FALSE(gate.configure_probability(probability));
        CHECK(gate.enabled());
        CHECK(gate.probability() == original_probability);
        CHECK(gate.threshold() == original_threshold);
    }

    CHECK_FALSE(gate.configure_probability(std::numeric_limits<Scalar_t>::quiet_NaN()));
    CHECK_FALSE(gate.configure_probability(std::numeric_limits<Scalar_t>::infinity()));
    CHECK(gate.enabled());
    CHECK(gate.probability() == original_probability);
    CHECK(gate.threshold() == original_threshold);
}

TEST_CASE("Disabled innovation gate accepts finite nonnegative statistics only")
{
    InnovationGate<3> gate{};
    CHECK_FALSE(gate.enabled());
    CHECK(gate.accepts(0.0));
    CHECK(gate.accepts(std::numeric_limits<Scalar_t>::max()));
    CHECK_FALSE(gate.accepts(-1.0));
    CHECK_FALSE(gate.accepts(std::numeric_limits<Scalar_t>::quiet_NaN()));
    CHECK_FALSE(gate.accepts(std::numeric_limits<Scalar_t>::infinity()));
    CHECK_FALSE(gate.accepts(-std::numeric_limits<Scalar_t>::infinity()));
}

TEST_CASE("Enabled innovation gate includes its threshold and rejects values above it")
{
    InnovationGate<3> gate{};
    REQUIRE(gate.configure_probability(0.95));
    REQUIRE(gate.enabled());

    const Scalar_t threshold = gate.threshold();
    CHECK(std::isfinite(threshold));
    CHECK(gate.accepts(threshold));
    CHECK_FALSE(gate.accepts(std::nextafter(threshold, std::numeric_limits<Scalar_t>::infinity())));
    CHECK_FALSE(gate.accepts(std::numeric_limits<Scalar_t>::quiet_NaN()));
    CHECK_FALSE(gate.accepts(std::numeric_limits<Scalar_t>::infinity()));
    CHECK_FALSE(gate.accepts(-std::numeric_limits<Scalar_t>::infinity()));
}

} // namespace navkit::core::estimation::test
