// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Frames.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/units/Units.hpp"
#include "test_main.hpp"

#include <numbers>

TEST_CASE("Unit cast feet to meters")
{
    using namespace navkit::core::units;
    using Frame = navkit::core::frames::Ecef;
    Vec<Foot, Frame, 3> p_ft;
    p_ft.v << 1.0, 2.0, 3.0;
    auto p_m = unit_cast<Foot, Meter>(p_ft);
    CHECK(p_m.v(0) == doctest::Approx(0.3048));
}

TEST_CASE("Frame DCM multiply compiles")
{
    navkit::core::frames::Dcm<navkit::core::frames::Body, navkit::core::frames::Ecef> C_eb;
    Eigen::Vector3d f_b;
    f_b << 1.0, 0.0, 0.0;
    auto f_e = C_eb * f_b;
    CHECK(f_e(0) == doctest::Approx(1.0));
}

TEST_CASE("WGS84 geodetic and ECEF conversions round trip")
{
    using navkit::core::Vec3;
    using navkit::core::environment::Wgs84;

    Vec3 p_e_m{};
    REQUIRE(navkit::core::frames::lla_deg_m_to_ecef_m(Vec3{0.0, 0.0, 0.0}, p_e_m));
    CHECK(p_e_m.x() == doctest::Approx(Wgs84::a_m));
    CHECK(p_e_m.y() == doctest::Approx(0.0).scale(1.0));
    CHECK(p_e_m.z() == doctest::Approx(0.0).scale(1.0));

    REQUIRE(navkit::core::frames::lla_deg_m_to_ecef_m(Vec3{90.0, 0.0, 0.0}, p_e_m));
    CHECK(p_e_m.x() == doctest::Approx(0.0).scale(1.0).epsilon(1.0e-8));
    CHECK(p_e_m.z() == doctest::Approx(Wgs84::b_m).epsilon(1.0e-12));

    const Vec3 expected_lla_deg_m{35.0, -106.0, 1500.0};
    REQUIRE(navkit::core::frames::lla_deg_m_to_ecef_m(expected_lla_deg_m, p_e_m));
    Vec3 actual_lla_deg_m{};
    REQUIRE(navkit::core::frames::ecef_m_to_lla_deg_m(p_e_m, actual_lla_deg_m));
    CHECK(actual_lla_deg_m.x() == doctest::Approx(expected_lla_deg_m.x()).epsilon(1.0e-10));
    CHECK(actual_lla_deg_m.y() == doctest::Approx(expected_lla_deg_m.y()).epsilon(1.0e-10));
    CHECK(actual_lla_deg_m.z() == doctest::Approx(expected_lla_deg_m.z()).epsilon(1.0e-8));

    CHECK_FALSE(navkit::core::frames::ecef_m_to_lla_deg_m(Vec3::Zero(), actual_lla_deg_m));
}

TEST_CASE("ECEF and NED frame matrices are proper inverses")
{
    using navkit::core::Mat3;
    using navkit::core::Vec3;

    Vec3 p_e_m{};
    REQUIRE(navkit::core::frames::lla_deg_m_to_ecef_m(Vec3{35.0, -106.0, 1500.0}, p_e_m));
    Mat3 dcm_e2n{};
    Mat3 dcm_n2e{};
    REQUIRE(navkit::core::frames::ecef_to_ned_matrix(p_e_m, dcm_e2n));
    REQUIRE(navkit::core::frames::ned_to_ecef_matrix(p_e_m, dcm_n2e));
    CHECK((dcm_e2n * dcm_n2e).isApprox(Mat3::Identity(), 1.0e-12));
    CHECK(dcm_e2n.determinant() == doctest::Approx(1.0).epsilon(1.0e-12));
}

TEST_CASE("Uniform rotating-frame transform preserves expected direction")
{
    using navkit::core::Mat3;
    using navkit::core::Scalar_t;
    using navkit::core::Vec3;
    using navkit::core::environment::Wgs84;

    const Scalar_t quarter_turn_s = std::numbers::pi_v<Scalar_t> / (2.0 * Wgs84::omega_rad_s);
    Mat3 dcm_e2i{};
    REQUIRE(navkit::core::frames::fixed_to_inertial_matrix<Wgs84>(quarter_turn_s, dcm_e2i));
    const Vec3 p_i_m = dcm_e2i * Vec3::UnitX();
    CHECK(p_i_m.x() == doctest::Approx(0.0).scale(1.0).epsilon(1.0e-10));
    CHECK(p_i_m.y() == doctest::Approx(1.0).epsilon(1.0e-12));
    CHECK((dcm_e2i.transpose() * dcm_e2i).isApprox(Mat3::Identity(), 1.0e-12));
}

TEST_CASE("Rotating-frame position velocity and acceleration conversions round trip")
{
    using navkit::core::Vec3;
    using navkit::core::environment::Wgs84;

    const navkit::core::Time_t elapsed_s = 123.5;
    const Vec3 p_e_m{Wgs84::a_m + 1000.0, 250.0, -400.0};
    const Vec3 v_e_mps{75.0, -20.0, 5.0};
    const Vec3 a_e_mps2{1.0, 0.5, -0.25};
    Vec3 p_i_m{};
    Vec3 v_i_mps{};
    Vec3 a_i_mps2{};
    REQUIRE(navkit::core::frames::fixed_to_inertial_position<Wgs84>(p_e_m, elapsed_s, p_i_m));
    REQUIRE(navkit::core::frames::fixed_to_inertial_velocity<Wgs84>(
        p_e_m, v_e_mps, elapsed_s, v_i_mps));
    REQUIRE(navkit::core::frames::fixed_to_inertial_acceleration<Wgs84>(
        p_e_m, v_e_mps, a_e_mps2, elapsed_s, a_i_mps2));

    Vec3 recovered_p_e_m{};
    Vec3 recovered_v_e_mps{};
    Vec3 recovered_a_e_mps2{};
    REQUIRE(
        navkit::core::frames::inertial_to_fixed_position<Wgs84>(p_i_m, elapsed_s, recovered_p_e_m));
    REQUIRE(navkit::core::frames::inertial_to_fixed_velocity<Wgs84>(
        p_i_m, v_i_mps, elapsed_s, recovered_v_e_mps));
    REQUIRE(navkit::core::frames::inertial_to_fixed_acceleration<Wgs84>(
        p_i_m, v_i_mps, a_i_mps2, elapsed_s, recovered_a_e_mps2));

    CHECK(recovered_p_e_m.isApprox(p_e_m, 1.0e-8));
    CHECK(recovered_v_e_mps.isApprox(v_e_mps, 1.0e-10));
    CHECK(recovered_a_e_mps2.isApprox(a_e_mps2, 1.0e-12));
}
