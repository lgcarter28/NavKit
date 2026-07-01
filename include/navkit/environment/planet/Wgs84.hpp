// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/environment/planet/PlanetPolicy.hpp"
#include "navkit/environment/planet/PlanetPolicyBase.hpp"
#include "navkit/frames/Frames.hpp"

namespace navkit::planet
{

struct Wgs84 : PlanetPolicyBase<Wgs84>
{
    using InertialFrame = frames::EarthCenteredInertial;
    using FixedFrame = frames::EarthCenteredEarthFixed;

    static inline constexpr Scalar_t a_m = 6378137.0;
    static inline constexpr Scalar_t f = 1.0 / 298.257223563;
    static inline constexpr Scalar_t b_m = a_m * (1.0 - f);
    static inline constexpr Scalar_t e2 = f * (2.0 - f);

    static inline constexpr Scalar_t mu_m3_s2 = 3.986004418e14;
    static inline constexpr Scalar_t omega_rad_s = 7.2921150e-5;
    static inline constexpr Scalar_t J2 = 1.08262668e-3;
};

static_assert(PlanetPolicy<Wgs84>);
static_assert(RotatingPlanetPolicy<Wgs84>);
static_assert(EllipsoidPlanetPolicy<Wgs84>);
static_assert(J2PlanetPolicy<Wgs84>);

} // namespace navkit::planet
