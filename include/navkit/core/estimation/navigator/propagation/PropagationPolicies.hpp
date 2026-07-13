// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/estimation/state/State.hpp"

#include <cstddef>

namespace navkit::core::estimation
{

struct NoOpPropagation
{
    static constexpr std::size_t imu_buffer_capacity = 1U; // NOLINT(readability-identifier-naming)

    template<StateDefPolicy StateDef>
    static bool process_imu_increment(State<StateDef>&, const ImuIncrement& increment)
    {
        return increment.dt_s >= 0.0;
    }

    template<StateDefPolicy StateDef>
    static bool process_imu_increment_pair(State<StateDef>& state,
                                           const ImuIncrement& first,
                                           const ImuIncrement& second)
    {
        return process_imu_increment<StateDef>(state, first) &&
               process_imu_increment<StateDef>(state, second);
    }

    template<StateDefPolicy StateDef>
    static bool covariance_step_from_increment(const State<StateDef>&,
                                               const ImuIncrement& increment,
                                               StateCov<StateDef>& phi,
                                               StateCov<StateDef>& qd)
    {
        phi.setIdentity();
        qd.setZero();
        return increment.dt_s >= 0.0;
    }

    template<StateDefPolicy StateDef>
    static bool covariance_step_from_increment_pair(const State<StateDef>& state,
                                                    const ImuIncrement& first,
                                                    const ImuIncrement& second,
                                                    StateCov<StateDef>& phi,
                                                    StateCov<StateDef>& qd)
    {
        return covariance_step_from_increment<StateDef>(state, first, phi, qd) &&
               covariance_step_from_increment<StateDef>(state, second, phi, qd);
    }
};

} // namespace navkit::core::estimation
