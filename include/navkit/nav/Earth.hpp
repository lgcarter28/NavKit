// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/planet/Wgs84.hpp"

namespace navkit::earth
{

// Compatibility aliases for existing Earth-centric code.
// New code should prefer navkit::planet::Wgs84 directly.
using Wgs84 = navkit::planet::Wgs84;

inline constexpr Scalar_t WGS84_A_M = Wgs84::a_m;
inline constexpr Scalar_t WGS84_B_M = Wgs84::b_m;
inline constexpr Scalar_t WGS84_F = Wgs84::f;
inline constexpr Scalar_t WGS84_E2 = Wgs84::e2;
inline constexpr Scalar_t MU_M3_S2 = Wgs84::mu_m3_s2;
inline constexpr Scalar_t OMEGA_IE_RADPS = Wgs84::omega_rad_s;
inline constexpr Scalar_t J2 = Wgs84::J2;

inline constexpr Scalar_t wgs84_a_m = Wgs84::a_m;
inline constexpr Scalar_t wgs84_b_m = Wgs84::b_m;
inline constexpr Scalar_t wgs84_f = Wgs84::f;
inline constexpr Scalar_t wgs84_e2 = Wgs84::e2;
inline constexpr Scalar_t mu_m3_s2 = Wgs84::mu_m3_s2;
inline constexpr Scalar_t omega_ie_rad_s = Wgs84::omega_rad_s;
inline constexpr Scalar_t j2 = Wgs84::J2;

} // namespace navkit::earth
