// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/frames/Frames.hpp"
#include "navkit/gravity/GravityPolicy.hpp"
#include "navkit/gravity/J2.hpp"
#include "navkit/gravity/Spherical.hpp"
#include "navkit/planet/PlanetPolicy.hpp"
#include "navkit/planet/Wgs84.hpp"

#include <Eigen/Dense>
#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::test
{

TEST_CASE("WGS84 satisfies planet policy concepts")
{
    static_assert(planet::PlanetPolicy<planet::Wgs84>);
    static_assert(planet::RotatingPlanetPolicy<planet::Wgs84>);
    static_assert(planet::EllipsoidPlanetPolicy<planet::Wgs84>);
    static_assert(planet::J2PlanetPolicy<planet::Wgs84>);

    CHECK(planet::Wgs84::a_m > 0.0);
    CHECK(planet::Wgs84::b_m > 0.0);
    CHECK(planet::Wgs84::a_m > planet::Wgs84::b_m);
    CHECK(planet::Wgs84::mu_m3_s2 > 0.0);
    CHECK(planet::Wgs84::omega_rad_s > 0.0);
    CHECK(planet::Wgs84::J2 > 0.0);
}

TEST_CASE("WGS84 exposes expected frame aliases")
{
    CHECK(std::is_same_v<planet::Wgs84::InertialFrame, frames::EarthCenteredInertial>);
    CHECK(std::is_same_v<planet::Wgs84::FixedFrame, frames::EarthCenteredEarthFixed>);
}

TEST_CASE("Gravity policies satisfy gravity policy concept")
{
    static_assert(gravity::GravityPolicy<gravity::Spherical<planet::Wgs84>>);
    static_assert(gravity::GravityPolicy<gravity::J2<planet::Wgs84>>);
}

TEST_CASE("Spherical WGS84 gravity points toward the planet center")
{
    const Eigen::Vector3d p_p{planet::Wgs84::a_m, 0.0, 0.0};

    const auto g_p = gravity::Spherical<planet::Wgs84>::acceleration(p_p);

    CHECK(g_p.x() < 0.0);
    CHECK(g_p.y() == doctest::Approx(0.0));
    CHECK(g_p.z() == doctest::Approx(0.0));

    const double expected_mag = planet::Wgs84::mu_m3_s2 / (planet::Wgs84::a_m * planet::Wgs84::a_m);
    CHECK(g_p.norm() == doctest::Approx(expected_mag).epsilon(1.0e-12));
}

TEST_CASE("J2 WGS84 gravity produces finite acceleration")
{
    const Eigen::Vector3d p_p{planet::Wgs84::a_m, 0.0, 0.0};

    const auto g_p = gravity::J2<planet::Wgs84>::acceleration(p_p);

    CHECK(g_p.allFinite());
    CHECK(g_p.x() < 0.0);
}

} // namespace navkit::test
