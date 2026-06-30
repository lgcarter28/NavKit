// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/Config.hpp"
#include "navkit/gravity/J2.hpp"
#include "navkit/gravity/Spherical.hpp"
#include "navkit/planet/Wgs84.hpp"

#include <Eigen/Dense>

namespace navkit::gravity
{

using Wgs84Spherical = Spherical<planet::Wgs84>;
using Wgs84J2 = J2<planet::Wgs84>;

// Compatibility wrapper for existing Earth-centric code.
// New code should prefer gravity::Spherical<planet::Wgs84>::acceleration(...)
// or a selected gravity policy.
[[nodiscard]] Eigen::Matrix<Scalar_t, 3, 1>
simple_gravity_ecef(const Eigen::Matrix<Scalar_t, 3, 1>& p_e);

} // namespace navkit::gravity
