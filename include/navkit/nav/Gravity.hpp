// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/Config.hpp"

#include <Eigen/Dense>

namespace navkit::gravity
{

Eigen::Matrix<Scalar_t, 3, 1> simple_gravity_ecef(const Eigen::Matrix<Scalar_t, 3, 1>& p_e);

} // namespace navkit::gravity
