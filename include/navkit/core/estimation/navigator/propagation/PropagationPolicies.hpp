// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/estimation/state/State.hpp"

#include <cstddef>

namespace navkit::core::estimation
{

struct NoOpPropagation
{
    static constexpr std::size_t imu_buffer_capacity = 1U; // NOLINT(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz = 100.0;
    static constexpr std::size_t covariance_history_capacity =
        1U; // NOLINT(readability-identifier-naming)
    static constexpr bool apply_coning_sculling_compensation =
        false; // NOLINT(readability-identifier-naming)

    template<StateSpaceDefPolicy StateDef>
    static bool process_imu_increment(const ImuIncrement& increment, NominalState<StateDef>&)
    {
        return increment.dt_s >= 0.0;
    }

    template<StateSpaceDefPolicy StateDef>
    static bool process_imu_increment_pair(const ImuIncrement& first,
                                           const ImuIncrement& second,
                                           NominalState<StateDef>& state)
    {
        return process_imu_increment<StateDef>(first, state) &&
               process_imu_increment<StateDef>(second, state);
    }

    template<StateSpaceDefPolicy StateDef>
    static bool covariance_step_from_increment(const NominalState<StateDef>&,
                                               const ImuIncrement& increment,
                                               ErrorStateCov<StateDef>& phi,
                                               ErrorStateCov<StateDef>& qd)
    {
        phi.setIdentity();
        qd.setZero();
        return increment.dt_s >= 0.0;
    }

    template<StateSpaceDefPolicy StateDef>
    static bool covariance_step_from_increment_pair(const NominalState<StateDef>& state,
                                                    const ImuIncrement& first,
                                                    const ImuIncrement& second,
                                                    ErrorStateCov<StateDef>& phi,
                                                    ErrorStateCov<StateDef>& qd)
    {
        return covariance_step_from_increment<StateDef>(state, first, phi, qd) &&
               covariance_step_from_increment<StateDef>(state, second, phi, qd);
    }
};

} // namespace navkit::core::estimation
