#pragma once

#include <Eigen/Dense>
#include "navkit/core/Config.hpp"

namespace navkit {

template <typename StateDef>
using State = Eigen::Matrix<Scalar_t, StateDef::N, 1>;

template <typename StateDef>
using StateCov = Eigen::Matrix<Scalar_t, StateDef::N, StateDef::N>;

} // namespace navkit
