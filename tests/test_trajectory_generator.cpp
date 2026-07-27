// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/sim/StationaryTrajectorySource.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>

namespace navkit::sim::test
{

using navkit::core::RationalRate;
using navkit::core::Timestamp;
using navkit::core::timestamp_at_sample_index;
using navkit::core::timestamp_seconds;
using navkit::core::Vec3;

TEST_CASE("Stationary trajectory source makes planned truth available without eager samples")
{
    StationaryTrajectoryConfig config;
    config.duration_s = 2.0;
    config.rate = RationalRate{.samples = 2U, .s = 1U};
    config.p_e << 1.0, 2.0, 3.0;

    StationaryTrajectorySource trajectory(config);
    TruthSample sample{};

    CHECK(timestamp_seconds(trajectory.t_start()) == doctest::Approx(0.0));
    CHECK(timestamp_seconds(trajectory.t_end()) == doctest::Approx(2.0));
    CHECK_FALSE(trajectory.query(Timestamp{}, sample));

    REQUIRE(trajectory.advance_to(Timestamp{}));
    REQUIRE(trajectory.query(Timestamp{}, sample));
    CHECK(sample.p_e.isApprox(config.p_e));
    CHECK(sample.v_e.isZero());
    CHECK(sample.q_b2e.isApprox(Eigen::Quaterniond::Identity()));

    const Timestamp half_second{.ns = 500'000'000U};
    REQUIRE(trajectory.advance_to(half_second));
    REQUIRE(trajectory.query(half_second, sample));
    CHECK(timestamp_seconds(sample.t) == doctest::Approx(0.5));
    CHECK_FALSE(trajectory.query(Timestamp{.s = 1U}, sample));
    CHECK_FALSE(trajectory.is_complete());

    REQUIRE(trajectory.advance_to(Timestamp{.s = 2U}));
    CHECK(trajectory.is_complete());
    CHECK_FALSE(trajectory.advance_to(Timestamp{.s = 3U}));
}

TEST_CASE(
    "Truth trajectory interpolates ECEF states and quaternion attitude at arbitrary timestamps")
{
    TruthSample first{};
    first.t = Timestamp{};
    first.p_e = Vec3{1.0, 2.0, 3.0};
    first.v_e = Vec3{4.0, 5.0, 6.0};
    first.q_b2e = Eigen::Quaterniond::Identity();
    first.w_ib_b_radps = Vec3{1.0, 2.0, 3.0};

    TruthSample second{};
    second.t = Timestamp{.s = 2U};
    second.p_e = Vec3{5.0, 6.0, 7.0};
    second.v_e = Vec3{8.0, 9.0, 10.0};
    second.q_b2e =
        core::math::quaternion_from_rpy_rad(Vec3{0.0, 0.0, core::Scalar_t{3.14159265358979323846}});
    second.w_ib_b_radps = Vec3{5.0, 6.0, 7.0};

    TruthTrajectory trajectory{std::vector<TruthSample>{first, second}};
    TruthSample midpoint{};
    REQUIRE(trajectory.sample_at(Timestamp{.s = 1U}, midpoint));
    CHECK(midpoint.p_e.isApprox(Vec3{3.0, 4.0, 5.0}));
    CHECK(midpoint.v_e.isApprox(Vec3{6.0, 7.0, 8.0}));
    CHECK(midpoint.w_ib_b_radps.isApprox(Vec3{3.0, 4.0, 5.0}));
    CHECK(core::math::rpy_rad_from_quaternion(midpoint.q_b2e).z() ==
          doctest::Approx(1.5707963267948966));

    Timestamp imu_timestamp{};
    REQUIRE(core::timestamp_at_sample_index(
        Timestamp{}, core::RationalRate{.samples = 600U, .s = 1U}, 300U, imu_timestamp));
    TruthSample imu_truth{};
    REQUIRE(trajectory.sample_at(imu_timestamp, imu_truth));
    CHECK(core::timestamp_seconds(imu_truth.t) == doctest::Approx(0.5));
    CHECK(imu_truth.p_e.x() == doctest::Approx(2.0));

    TruthSample out_of_range{};
    CHECK_FALSE(trajectory.sample_at(Timestamp{.s = 3U}, out_of_range));
    CHECK_FALSE(
        trajectory.sample_at(Timestamp{.scale = core::TimeScale::Gps, .s = 1U}, out_of_range));
}

TEST_CASE("CSV trajectory source uses the shared truth trajectory contract")
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "navkit_truth_trajectory_source.csv";
    {
        std::ofstream stream(path);
        REQUIRE(stream.good());
        stream << "time_s,p_e_x_m,p_e_y_m,p_e_z_m,v_e_x_mps,v_e_y_mps,v_e_z_mps,q_b2e_w,q_b2e_x,q_"
                  "b2e_y,"
                  "q_b2e_z\n";
        stream << "0,6378137,0,0,0,0,0,1,0,0,0\n";
        stream << "2,6378139,0,0,2,0,0,1,0,0,0\n";
    }

    const nlohmann::json cfg{
        {"trajectory", {{"type", "csv"}, {"csv_path", path.filename().string()}}}};
    app_support::TrajectoryRun run = app_support::trajectory_run_from_json(cfg, path.parent_path());
    REQUIRE(run.source);
    REQUIRE(run.source->advance_to(Timestamp{.s = 1U}));
    TruthSample midpoint{};
    REQUIRE(run.source->query(Timestamp{.s = 1U}, midpoint));
    CHECK(midpoint.p_e.x() == doctest::Approx(6378138.0));
    CHECK(midpoint.v_e.x() == doctest::Approx(1.0));
    CHECK_FALSE(run.source->query(Timestamp{.s = 2U}, midpoint));

    std::filesystem::remove(path);
}

} // namespace navkit::sim::test
