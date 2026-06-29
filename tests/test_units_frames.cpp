// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/nav/Frames.hpp"
#include "navkit/nav/Units.hpp"
#include "test_main.hpp"

TEST_CASE("Unit cast feet to meters")
{
    using namespace navkit::units;
    using Frame = navkit::frames::Ecef;
    Vec<Foot, Frame, 3> p_ft;
    p_ft.v << 1.0, 2.0, 3.0;
    auto p_m = unit_cast<Foot, Meter>(p_ft);
    CHECK(p_m.v(0) == doctest::Approx(0.3048));
}

TEST_CASE("Frame DCM multiply compiles")
{
    navkit::frames::Dcm<navkit::frames::Body, navkit::frames::Ecef> C_eb;
    Eigen::Vector3d f_b;
    f_b << 1.0, 0.0, 0.0;
    auto f_e = C_eb * f_b;
    CHECK(f_e(0) == doctest::Approx(1.0));
}
