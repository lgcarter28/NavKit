// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/Config.hpp"
#include "navkit/core/StateDefPolicy.hpp"

#include <Eigen/Dense>

namespace navkit
{

template<StateDefPolicy StateDef>
using State = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, 1>;

template<StateDefPolicy StateDef>
using StateCov = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, StateDef::N>;

} // namespace navkit
