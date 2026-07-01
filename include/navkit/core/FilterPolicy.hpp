// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit
{

template<typename Candidate, typename Sensor>
concept FilterPolicy = requires(Candidate& filter, Sensor& sensor) {
    { filter.process_sensor(sensor) } -> std::same_as<void>;
};

} // namespace navkit
