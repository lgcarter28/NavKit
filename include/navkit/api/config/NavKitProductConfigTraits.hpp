// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::api::config::detail
{

template<typename StatisticsTuple, typename SensorTuple>
struct measurement_statistics_sources_configured;

template<typename SensorTuple, typename... Statistics>
struct measurement_statistics_sources_configured<std::tuple<Statistics...>, SensorTuple>
    : std::bool_constant<(navkit::core::estimation::
                              sensor_type_contains_v<typename Statistics::Sensor_t, SensorTuple> &&
                          ...)>
{};

template<typename StatisticsTuple, typename SensorTuple>
inline constexpr bool measurement_statistics_sources_configured_v =
    measurement_statistics_sources_configured<StatisticsTuple, SensorTuple>::value;

} // namespace navkit::api::config::detail
