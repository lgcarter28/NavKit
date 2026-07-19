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
    StateSpaceDefPolicy<StateDef> &&
    requires(Candidate& propagation,
             const Candidate& const_propagation,
             NominalState<StateDef>& state,
             const NominalState<StateDef>& const_state,
             const typename Candidate::RuntimeConfig_t& runtime_config,
             ErrorStateCov<StateDef>& phi,
             ErrorStateCov<StateDef>& qd) {
        typename Candidate::RuntimeConfig_t;
        { Candidate::imu_buffer_capacity } -> std::convertible_to<std::size_t>;
        { Candidate::covariance_history_capacity } -> std::convertible_to<std::size_t>;
        { Candidate::covariance_update_rate_hz } -> std::convertible_to<Time_t>;
        { Candidate::apply_coning_sculling_compensation } -> std::convertible_to<bool>;
        { Candidate::runtime_config } -> std::convertible_to<typename Candidate::RuntimeConfig_t>;
        { propagation.set_runtime_config(runtime_config) } -> std::same_as<void>;
        {
            const_propagation.runtime_config_value()
        } -> std::same_as<const typename Candidate::RuntimeConfig_t&>;
        {
            propagation.template process_imu_increment<StateDef>(ImuIncrement{}, state)
        } -> std::same_as<bool>;
        {
            propagation.template process_imu_increment_pair<StateDef>(
                ImuIncrement{}, ImuIncrement{}, state)
        } -> std::same_as<bool>;
        {
            const_propagation.template covariance_step_from_increment<StateDef>(
                const_state, ImuIncrement{}, phi, qd)
        } -> std::same_as<bool>;
        {
            const_propagation.template covariance_step_from_increment_pair<StateDef>(
                const_state, ImuIncrement{}, ImuIncrement{}, phi, qd)
        } -> std::same_as<bool>;
    };

} // namespace navkit::core::estimation
