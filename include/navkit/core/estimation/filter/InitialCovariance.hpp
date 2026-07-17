// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <array>
#include <cstddef>

namespace navkit::core::estimation
{

template<StateSpaceDefPolicy StateDef>
using InitialCovariance = ErrorStateCov<StateDef>;

template<StateSpaceDefPolicy StateDef>
struct InitialCovarianceDiagonal
{
    std::array<Scalar_t, StateDef::Error::N> values{};
};

template<StateSpaceDefPolicy StateDef>
[[nodiscard]] inline InitialCovariance<StateDef>
diagonal_initial_covariance(const InitialCovarianceDiagonal<StateDef>& diagonal)
{
    InitialCovariance<StateDef> covariance = InitialCovariance<StateDef>::Zero();
    for (std::size_t i = 0U; i < diagonal.values.size(); ++i) {
        covariance(static_cast<int>(i), static_cast<int>(i)) = diagonal.values.at(i);
    }
    return covariance;
}

} // namespace navkit::core::estimation
