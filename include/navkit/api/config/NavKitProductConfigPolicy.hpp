// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTuple.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::api::config
{

namespace detail
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

} // namespace detail

template<typename Candidate>
concept NavKitProductConfigPolicy = requires {
    typename Candidate::Numeric;
    typename Candidate::StateDef;
    typename Candidate::Sensors;
    typename Candidate::MeasurementStatisticsConfigs;
    typename Candidate::Profiler;
    typename Candidate::Filter;
    typename Candidate::Navigator;

    requires navkit::core::config::ConfigPolicy<Candidate>;
    requires navkit::core::estimation::StateDefPolicy<typename Candidate::StateDef>;
    requires navkit::core::estimation::SensorCollectionPolicy<typename Candidate::Sensors>;
    requires navkit::core::estimation::sensor_ids_unique_v<typename Candidate::Sensors>;
    requires navkit::core::estimation::MeasurementStatisticsCollectionPolicy<
        typename Candidate::MeasurementStatisticsConfigs>;
    requires detail::measurement_statistics_sources_configured_v<
        typename Candidate::MeasurementStatisticsConfigs,
        typename Candidate::Sensors>;
    requires navkit::core::profiling::ProfilerPolicy<typename Candidate::Profiler>;
};

} // namespace navkit::api::config
