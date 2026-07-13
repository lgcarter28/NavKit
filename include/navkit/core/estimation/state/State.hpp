// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <Eigen/Dense>
#include <type_traits>

namespace navkit::core::estimation
{

template<StateDefPolicy StateDef>
using State = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, 1>;

template<StateDefPolicy StateDef>
using StateCov = Eigen::Matrix<typename StateDef::Scalar_t, StateDef::N, StateDef::N>;

namespace detail
{

template<typename StateDef, typename = void>
struct NominalStateDimension
{
    static constexpr int value = StateDef::N;
};

template<typename StateDef>
struct NominalStateDimension<StateDef, std::void_t<decltype(StateDef::NominalN)>>
{
    static constexpr int value = StateDef::NominalN;
};

} // namespace detail

template<StateDefPolicy StateDef>
inline constexpr int NominalStateDimension_v = detail::NominalStateDimension<StateDef>::value;

template<StateDefPolicy StateDef>
using NominalState =
    Eigen::Matrix<typename StateDef::Scalar_t, NominalStateDimension_v<StateDef>, 1>;

} // namespace navkit::core::estimation
