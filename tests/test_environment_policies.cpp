// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/GravityPolicy.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/gravity/Spherical.hpp"
#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Frames.hpp"

#include <Eigen/Dense>
#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::core::environment::test
{

TEST_CASE("WGS84 satisfies planet policy concepts")
{
    static_assert(PlanetPolicy<Wgs84>);
    static_assert(RotatingPlanetPolicy<Wgs84>);
    static_assert(EllipsoidPlanetPolicy<Wgs84>);
    static_assert(J2PlanetPolicy<Wgs84>);

    CHECK(Wgs84::a_m > 0.0);
    CHECK(Wgs84::b_m > 0.0);
    CHECK(Wgs84::a_m > Wgs84::b_m);
    CHECK(Wgs84::mu_m3_s2 > 0.0);
    CHECK(Wgs84::omega_rad_s > 0.0);
    CHECK(Wgs84::J2 > 0.0);
}

TEST_CASE("WGS84 exposes expected frame aliases")
{
    CHECK(std::is_same_v<Wgs84::InertialFrame, navkit::core::frames::EarthCenteredInertial>);
    CHECK(std::is_same_v<Wgs84::FixedFrame, navkit::core::frames::EarthCenteredEarthFixed>);
}

TEST_CASE("Gravity policies satisfy gravity policy concept")
{
    static_assert(GravityPolicy<Spherical<Wgs84>>);
    static_assert(GravityPolicy<J2<Wgs84>>);
}

TEST_CASE("Spherical WGS84 gravity points toward the planet center")
{
    const Eigen::Vector3d p_p{Wgs84::a_m, 0.0, 0.0};

    const auto g_p = Spherical<Wgs84>::acceleration(p_p);

    CHECK(g_p.x() < 0.0);
    CHECK(g_p.y() == doctest::Approx(0.0));
    CHECK(g_p.z() == doctest::Approx(0.0));

    const double expected_mag = Wgs84::mu_m3_s2 / (Wgs84::a_m * Wgs84::a_m);
    CHECK(g_p.norm() == doctest::Approx(expected_mag).epsilon(1.0e-12));
}

TEST_CASE("J2 WGS84 gravity produces finite acceleration")
{
    const Eigen::Vector3d p_p{Wgs84::a_m, 0.0, 0.0};

    const auto g_p = J2<Wgs84>::acceleration(p_p);

    CHECK(g_p.allFinite());
    CHECK(g_p.x() < 0.0);
}

TEST_CASE("Rotating-planet centrifugal acceleration is outward with an exact Jacobian")
{
    const Eigen::Vector3d p_p{Wgs84::a_m, 0.0, 100.0};
    const Eigen::Vector3d centrifugal = centrifugal_acceleration_fixed_mps2<Wgs84>(p_p);
    const Eigen::Matrix3d gradient = centrifugal_acceleration_gradient_fixed_1ps2<Wgs84>();

    CHECK(centrifugal.x() > 0.0);
    CHECK(centrifugal.y() == doctest::Approx(0.0));
    CHECK(centrifugal.z() == doctest::Approx(0.0));
    CHECK(centrifugal.isApprox(gradient * p_p, 1.0e-15));
    CHECK(gradient(0, 0) == doctest::Approx(Wgs84::omega_rad_s * Wgs84::omega_rad_s));
    CHECK(gradient(1, 1) == doctest::Approx(Wgs84::omega_rad_s * Wgs84::omega_rad_s));
    CHECK(gradient(2, 2) == doctest::Approx(0.0));
}

} // namespace navkit::core::environment::test
