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
using CovarianceFloor = ErrorStateCov<StateDef>;

template<StateSpaceDefPolicy StateDef>
struct CovarianceFloorDiagonal
{
    std::array<Scalar_t, StateDef::Error::N> values{};
};

template<StateSpaceDefPolicy StateDef>
[[nodiscard]] inline CovarianceFloor<StateDef>
diagonal_covariance_floor(const CovarianceFloorDiagonal<StateDef>& diagonal)
{
    CovarianceFloor<StateDef> floor = CovarianceFloor<StateDef>::Zero();
    for (std::size_t i = 0U; i < diagonal.values.size(); ++i) {
        floor(static_cast<int>(i), static_cast<int>(i)) = diagonal.values.at(i);
    }
    return floor;
}

template<StateSpaceDefPolicy StateDef>
inline void apply_diagonal_covariance_floor(ErrorStateCov<StateDef>& covariance,
                                            const CovarianceFloor<StateDef>& floor)
{
    for (int i = 0; i < StateDef::Error::N; ++i) {
        if (floor(i, i) > 0.0 && covariance(i, i) < floor(i, i)) {
            covariance(i, i) = floor(i, i);
        }
    }
    covariance = 0.5 * (covariance + covariance.transpose());
}

} // namespace navkit::core::estimation
