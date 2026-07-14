// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorPolicy.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate>
concept FilterCorrectionPolicy = requires(Candidate& filter) {
    { filter.inject() } -> std::same_as<void>;
    { filter.reset() } -> std::same_as<void>;
};

template<typename Candidate>
concept FilterPolicy =
    FilterCorrectionPolicy<Candidate> && requires(Candidate& filter,
                                                  const Candidate& const_filter,
                                                  const typename Candidate::State_t& state,
                                                  const typename Candidate::P_t& covariance) {
        typename Candidate::State_t;
        typename Candidate::ErrorState_t;
        typename Candidate::P_t;
        typename Candidate::StateDef_t;

        requires StateSpaceDefPolicy<typename Candidate::StateDef_t>;

        { filter.state() } -> std::same_as<typename Candidate::State_t&>;
        { const_filter.state() } -> std::same_as<const typename Candidate::State_t&>;
        { filter.covariance() } -> std::same_as<typename Candidate::P_t&>;
        { const_filter.covariance() } -> std::same_as<const typename Candidate::P_t&>;
        { filter.set_state(state) } -> std::same_as<void>;
        { filter.set_covariance(covariance) } -> std::same_as<void>;
        { filter.propagate_covariance(covariance, covariance) } -> std::same_as<void>;
    };

template<typename Candidate, typename Sensor>
concept FilterSensorPolicy =
    FilterPolicy<Candidate> && SensorPolicy<Sensor> && requires(Candidate& filter, Sensor& sensor) {
        { filter.process_sensor(sensor) } -> std::same_as<void>;
    };

} // namespace navkit::core::estimation
