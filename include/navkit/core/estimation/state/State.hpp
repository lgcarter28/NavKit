// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<StateDefPolicy StateDef>
using State = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, 1>;

template<StateDefPolicy StateDef>
using StateCov = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, StateDef::N>;

template<StateSpaceDefPolicy StateSpaceDef>
using NominalState = State<typename StateSpaceDef::Nominal>;

template<StateSpaceDefPolicy StateSpaceDef>
using ErrorState = State<typename StateSpaceDef::Error>;

template<StateSpaceDefPolicy StateSpaceDef>
using ErrorStateCov = StateCov<typename StateSpaceDef::Error>;

} // namespace navkit::core::estimation
