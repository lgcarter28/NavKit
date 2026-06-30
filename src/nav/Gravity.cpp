// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/nav/Gravity.hpp"

namespace navkit::gravity
{

Eigen::Matrix<Scalar_t, 3, 1> simple_gravity_ecef(const Eigen::Matrix<Scalar_t, 3, 1>& p_e)
{
    return Wgs84Spherical::acceleration(p_e);
}

} // namespace navkit::gravity
