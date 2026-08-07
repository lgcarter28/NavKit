// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Candidate, typename Filter, typename Sensor>
concept UpdatePolicy =
    FilterPolicy<Filter> && SensorPolicy<Sensor> && requires(Filter& filter, Sensor& sensor) {
        {
            Candidate::template sensor_update<std::remove_reference_t<Sensor>>(filter, sensor)
        } -> std::same_as<typename Filter::AppliedCorrection_t>;
        { Candidate::filter_update(filter) } -> std::same_as<typename Filter::AppliedCorrection_t>;
    };

} // namespace navkit::core::estimation
