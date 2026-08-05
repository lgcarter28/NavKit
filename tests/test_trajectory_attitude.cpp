// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/trajectory/TrajectoryAttitudeJson.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Timestamp.hpp"
#include "test_main.hpp"

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

[[nodiscard]] nlohmann::json quaternion_json(const Eigen::Quaternion<core::Scalar_t>& q)
{
    return nlohmann::json{q.w(), q.x(), q.y(), q.z()};
}

[[nodiscard]] nlohmann::json dcm_json(const core::Mat3& dcm)
{
    return nlohmann::json{
        dcm(0, 0),
        dcm(0, 1),
        dcm(0, 2),
        dcm(1, 0),
        dcm(1, 1),
        dcm(1, 2),
        dcm(2, 0),
        dcm(2, 1),
        dcm(2, 2),
    };
}

[[nodiscard]] nlohmann::json rpy_json(const Eigen::Quaternion<core::Scalar_t>& q)
{
    const core::Vec3 rpy_rad = core::math::rpy_rad_from_quaternion(q);
    const core::Vec3 rpy_deg = degrees_from_radians(rpy_rad);
    return nlohmann::json{rpy_deg.x(), rpy_deg.y(), rpy_deg.z()};
}

void check_same_rotation(const Eigen::Quaternion<core::Scalar_t>& actual,
                         const Eigen::Quaternion<core::Scalar_t>& expected)
{
    CHECK(std::abs(actual.dot(expected)) == doctest::Approx(1.0).epsilon(1.0e-11));
}

} // namespace

TEST_CASE("Every supported trajectory attitude form maps to canonical body-to-ECEF")
{
    core::Vec3 p_e_m{};
    REQUIRE(core::frames::lla_deg_m_to_ecef_m(core::Vec3{35.0, -106.0, 1500.0}, p_e_m));

    core::Timestamp t_epoch{};
    core::Timestamp t{};
    REQUIRE(core::timestamp_from_seconds(100.0, core::TimeScale::Monotonic, t_epoch));
    REQUIRE(core::timestamp_from_seconds(137.25, core::TimeScale::Monotonic, t));

    const Eigen::Quaternion<core::Scalar_t> q_b2e =
        core::math::quaternion_from_rpy_rad(core::Vec3{0.2, -0.3, 0.4});

    core::Mat3 dcm_n2e{};
    core::Mat3 dcm_i2e{};
    REQUIRE(core::frames::ned_to_ecef_matrix(p_e_m, dcm_n2e));
    REQUIRE(core::frames::inertial_to_fixed_matrix<core::environment::Wgs84>(t, t_epoch, dcm_i2e));
    const Eigen::Quaternion<core::Scalar_t> q_n2e{dcm_n2e};
    const Eigen::Quaternion<core::Scalar_t> q_i2e{dcm_i2e};
    const Eigen::Quaternion<core::Scalar_t> q_b2n =
        core::math::normalized_with_positive_scalar(q_n2e.conjugate() * q_b2e);
    const Eigen::Quaternion<core::Scalar_t> q_b2i =
        core::math::normalized_with_positive_scalar(q_i2e.conjugate() * q_b2e);
    const Eigen::Quaternion<core::Scalar_t> q_e2b{q_b2e.conjugate()};
    const Eigen::Quaternion<core::Scalar_t> q_i2b{q_b2i.conjugate()};
    const Eigen::Quaternion<core::Scalar_t> q_n2b{q_b2n.conjugate()};

    const std::array<std::pair<std::string, Eigen::Quaternion<core::Scalar_t>>, 6> frame_pairs{{
        {"b2e", q_b2e},
        {"e2b", q_e2b},
        {"b2i", q_b2i},
        {"i2b", q_i2b},
        {"b2n", q_b2n},
        {"n2b", q_n2b},
    }};

    for (const std::pair<std::string, Eigen::Quaternion<core::Scalar_t>>& frame_pair :
         frame_pairs) {
        const std::string& direction = frame_pair.first;
        const Eigen::Quaternion<core::Scalar_t>& q_start2end = frame_pair.second;
        const std::array<std::pair<std::string, nlohmann::json>, 3> payloads{{
            {"q_" + direction, quaternion_json(q_start2end)},
            {"dcm_" + direction, dcm_json(q_start2end.toRotationMatrix())},
            {"rpy_" + direction + "_deg", rpy_json(q_start2end)},
        }};

        for (const std::pair<std::string, nlohmann::json>& payload : payloads) {
            const nlohmann::json trajectory{{payload.first, payload.second}};
            const Eigen::Quaternion<core::Scalar_t> actual =
                detail::trajectory_attitude_b2e_from_json(trajectory, p_e_m, t, t_epoch);
            check_same_rotation(actual, q_b2e);
        }
    }
}

TEST_CASE("Trajectory attitude input rejects ambiguity and invalid rotations")
{
    const core::Timestamp t{};
    const core::Vec3 p_e_m{core::environment::Wgs84::a_m, 0.0, 0.0};

    CHECK_THROWS_AS(detail::validate_trajectory_attitude_json(nlohmann::json::object()),
                    std::runtime_error);
    CHECK_THROWS_AS(detail::validate_trajectory_attitude_json(nlohmann::json{
                        {"q_b2e", {1.0, 0.0, 0.0, 0.0}}, {"rpy_b2e_deg", {0.0, 0.0, 0.0}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(
        detail::validate_trajectory_attitude_json(nlohmann::json{{"q_b2e", {0.0, 0.0, 0.0, 0.0}}}),
        std::runtime_error);
    CHECK_THROWS_AS(detail::validate_trajectory_attitude_json(nlohmann::json{
                        {"dcm_b2e", {-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}}}),
                    std::runtime_error);

    CHECK_THROWS_AS((void)detail::trajectory_attitude_b2e_from_json(
                        nlohmann::json{{"q_b2n", {1.0, 0.0, 0.0, 0.0}}}, core::Vec3::Zero(), t, t),
                    std::runtime_error);

    core::Timestamp gps_time{};
    REQUIRE(core::timestamp_from_seconds(0.0, core::TimeScale::Gps, gps_time));
    CHECK_THROWS_AS((void)detail::trajectory_attitude_b2e_from_json(
                        nlohmann::json{{"q_b2i", {1.0, 0.0, 0.0, 0.0}}}, p_e_m, gps_time, t),
                    std::runtime_error);
}

TEST_CASE("RPY extraction selects the conventional aerospace principal branch")
{
    const std::array<core::Vec3, 4> cases{
        core::Vec3{-0.35, 0.0, 0.0},
        core::Vec3{0.35, -0.2, 1.1},
        core::Vec3{-0.4, 0.3, -2.7},
        core::Vec3{0.2, 1.2, 2.9},
    };

    for (const core::Vec3& expected_rpy_rad : cases) {
        const Eigen::Quaternion<core::Scalar_t> q =
            core::math::quaternion_from_rpy_rad(expected_rpy_rad);
        const core::Vec3 actual_rpy_rad = core::math::rpy_rad_from_quaternion(q);
        CHECK(actual_rpy_rad.isApprox(expected_rpy_rad, 1.0e-12));
    }
}

} // namespace navkit::app_support::test
