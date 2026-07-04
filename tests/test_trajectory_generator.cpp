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
    config.dt_s = 0.5;
    config.p_e << 1.0, 2.0, 3.0;

    const auto samples = TrajectoryGenerator::stationary(config);

    REQUIRE(samples.size() == 5U);
    CHECK(samples.front().time == doctest::Approx(0.0));
    CHECK(samples.back().time == doctest::Approx(2.0));

    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples.at(index);
        CHECK(sample.time == doctest::Approx(static_cast<double>(index) * config.dt_s));
        CHECK(sample.p_e.isApprox(config.p_e));
        CHECK(sample.v_e.isZero());
        CHECK(sample.a_e.isZero());
        CHECK(sample.q_eb.isApprox(Eigen::Quaterniond::Identity()));
        CHECK(sample.w_ib_b.isZero());
    }
}

} // namespace navkit::sim::test
