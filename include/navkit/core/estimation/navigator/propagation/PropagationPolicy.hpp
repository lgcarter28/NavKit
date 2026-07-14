// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <concepts>
#include <cstddef>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept PropagationPolicy =
    StateSpaceDefPolicy<StateDef> && requires(NominalState<StateDef>& state,
                                              const NominalState<StateDef>& const_state,
                                              ErrorStateCov<StateDef>& phi,
                                              ErrorStateCov<StateDef>& qd) {
        { Candidate::imu_buffer_capacity } -> std::convertible_to<std::size_t>;
        { Candidate::covariance_history_capacity } -> std::convertible_to<std::size_t>;
        { Candidate::covariance_update_rate_hz } -> std::convertible_to<Time_t>;
        {
            Candidate::template process_imu_increment<StateDef>(ImuIncrement{}, state)
        } -> std::same_as<bool>;
        {
            Candidate::template process_imu_increment_pair<StateDef>(
                ImuIncrement{}, ImuIncrement{}, state)
        } -> std::same_as<bool>;
        {
            Candidate::template covariance_step_from_increment<StateDef>(
                const_state, ImuIncrement{}, phi, qd)
        } -> std::same_as<bool>;
        {
            Candidate::template covariance_step_from_increment_pair<StateDef>(
                const_state, ImuIncrement{}, ImuIncrement{}, phi, qd)
        } -> std::same_as<bool>;
    };

} // namespace navkit::core::estimation
