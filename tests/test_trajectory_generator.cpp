// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/TrajectoryGenerator.hpp"
#include "test_main.hpp"

namespace navkit::sim::test
{

TEST_CASE("Stationary trajectory includes both endpoints at fixed sample spacing")
{
    StationaryTrajectoryConfig config;
    config.duration_s = 2.0;
    config.rate = RationalRate{.samples = 2U, .s = 1U};
    config.p_e << 1.0, 2.0, 3.0;

    const auto samples = TrajectoryGenerator::stationary(config);

    REQUIRE(samples.size() == 5U);
    CHECK(timestamp_seconds(samples.front().t) == doctest::Approx(0.0));
    CHECK(timestamp_seconds(samples.back().t) == doctest::Approx(2.0));

    for (std::size_t index = 0; index < samples.size(); ++index) {
        const TruthSample& sample = samples.at(index);
        CHECK(timestamp_seconds(sample.t) == doctest::Approx(0.5 * static_cast<double>(index)));
        CHECK(sample.p_e.isApprox(config.p_e));
        CHECK(sample.v_e.isZero());
        CHECK(sample.q_b2e.isApprox(Eigen::Quaterniond::Identity()));
    }
}

} // namespace navkit::sim::test
