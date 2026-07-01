// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>
#include <type_traits>

namespace navkit
{

template<typename Candidate, typename Filter, typename Sensor>
concept UpdatePolicy = requires(Filter& filter, Sensor& sensor) {
    {
        Candidate::template sensor_update<std::remove_reference_t<Sensor>>(filter, sensor)
    } -> std::same_as<void>;
    { Candidate::filter_update(filter) } -> std::same_as<void>;
};

} // namespace navkit
