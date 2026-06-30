// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/gravity/GravityPolicy.hpp"
#include "navkit/gravity/J2.hpp"
#include "navkit/gravity/Spherical.hpp"
#include "navkit/planet/Wgs84.hpp"
#include "test_main.hpp"

TEST_CASE("Wgs84 satisfies planet policy capabilities")
{
    static_assert(navkit::planet::PlanetPolicy<navkit::planet::Wgs84>);
    static_assert(navkit::planet::RotatingPlanetPolicy<navkit::planet::Wgs84>);
    static_assert(navkit::planet::EllipsoidPlanetPolicy<navkit::planet::Wgs84>);
    static_assert(navkit::planet::J2PlanetPolicy<navkit::planet::Wgs84>);

    CHECK(navkit::planet::Wgs84::a_m == doctest::Approx(6378137.0));
    CHECK(navkit::planet::Wgs84::omega_rad_s == doctest::Approx(7.2921150e-5));
}

TEST_CASE("Wgs84 spherical gravity policy compiles and returns inward acceleration")
{
    using Gravity = navkit::gravity::Spherical<navkit::planet::Wgs84>;
    static_assert(navkit::gravity::GravityPolicy<Gravity>);

    Eigen::Vector3d p_e{navkit::planet::Wgs84::a_m, 0.0, 0.0};
    const Eigen::Vector3d g_e = Gravity::acceleration(p_e);

    CHECK(g_e.x() < 0.0);
    CHECK(g_e.y() == doctest::Approx(0.0));
    CHECK(g_e.z() == doctest::Approx(0.0));
}

TEST_CASE("Wgs84 J2 gravity policy compiles and returns finite acceleration")
{
    using Gravity = navkit::gravity::J2<navkit::planet::Wgs84>;
    static_assert(navkit::gravity::GravityPolicy<Gravity>);

    Eigen::Vector3d p_e{navkit::planet::Wgs84::a_m, 0.0, 0.0};
    const Eigen::Vector3d g_e = Gravity::acceleration(p_e);

    CHECK(g_e.allFinite());
    CHECK(g_e.x() < 0.0);
}
