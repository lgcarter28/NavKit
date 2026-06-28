#pragma once

#include <Eigen/Dense>
#include "navkit/core/Config.hpp"

namespace navkit::gravity {

Eigen::Matrix<Scalar_t, 3, 1> simple_gravity_ecef(const Eigen::Matrix<Scalar_t, 3, 1>& p_e);

} // namespace navkit::gravity
