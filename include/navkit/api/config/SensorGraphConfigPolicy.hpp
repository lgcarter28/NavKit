// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::api::config
{

template<typename SensorTuple>
struct MeasurementModelsFromSensors;

template<typename... Sensors>
struct MeasurementModelsFromSensors<std::tuple<Sensors...>>
{
    using type = std::tuple<typename Sensors::Model_t...>;
};

template<typename SensorTuple>
using MeasurementModelsFromSensors_t = typename MeasurementModelsFromSensors<SensorTuple>::type;

template<typename SensorTuple>
struct sensor_ids_unique;

template<>
struct sensor_ids_unique<std::tuple<>> : std::true_type
{};

template<typename First, typename... Rest>
struct sensor_ids_unique<std::tuple<First, Rest...>>
    : std::bool_constant<((First::Id != Rest::Id) && ...) &&
                         sensor_ids_unique<std::tuple<Rest...>>::value>
{};

template<typename SensorTuple>
inline constexpr bool sensor_ids_unique_v = sensor_ids_unique<SensorTuple>::value;

template<core::estimation::SensorId IdValue, typename SensorTuple>
struct sensor_id_count;

template<core::estimation::SensorId IdValue>
struct sensor_id_count<IdValue, std::tuple<>> : std::integral_constant<std::size_t, 0U>
{};

template<core::estimation::SensorId IdValue, typename First, typename... Rest>
struct sensor_id_count<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t,
                             (First::Id == IdValue ? 1U : 0U) +
                                 sensor_id_count<IdValue, std::tuple<Rest...>>::value>
{};

template<core::estimation::SensorId IdValue, typename SensorTuple>
inline constexpr std::size_t sensor_id_count_v = sensor_id_count<IdValue, SensorTuple>::value;

template<core::estimation::SensorId IdValue, typename SensorTuple>
inline constexpr bool sensor_id_exists_v = sensor_id_count_v<IdValue, SensorTuple> > 0U;

template<typename Sensor, typename SensorTuple>
struct sensor_type_contains;

template<typename Sensor>
struct sensor_type_contains<Sensor, std::tuple<>> : std::false_type
{};

template<typename Sensor, typename... Rest>
struct sensor_type_contains<Sensor, std::tuple<Sensor, Rest...>> : std::true_type
{};

template<typename Sensor, typename First, typename... Rest>
struct sensor_type_contains<Sensor, std::tuple<First, Rest...>>
    : sensor_type_contains<Sensor, std::tuple<Rest...>>
{};

template<typename Sensor, typename SensorTuple>
inline constexpr bool sensor_type_contains_v = sensor_type_contains<Sensor, SensorTuple>::value;

template<core::estimation::SensorId IdValue, typename SensorTuple>
struct sensor_index_from_id;

template<core::estimation::SensorId IdValue, typename First, typename... Rest>
    requires(First::Id == IdValue)
struct sensor_index_from_id<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t, 0U>
{};

template<core::estimation::SensorId IdValue, typename First, typename... Rest>
    requires(First::Id != IdValue)
struct sensor_index_from_id<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t,
                             1U + sensor_index_from_id<IdValue, std::tuple<Rest...>>::value>
{};

template<core::estimation::SensorId IdValue, typename SensorTuple>
inline constexpr std::size_t SensorIndexFromId_v =
    sensor_index_from_id<IdValue, SensorTuple>::value;

template<core::estimation::SensorId IdValue, typename SensorTuple>
using SensorFromId_t = std::tuple_element_t<SensorIndexFromId_v<IdValue, SensorTuple>, SensorTuple>;

template<typename Candidate>
concept SensorGraphConfigPolicy = requires {
    typename Candidate::Sensors;
    typename Candidate::MeasurementModels;

    requires navkit::core::estimation::SensorCollectionPolicy<typename Candidate::Sensors>;
    requires sensor_ids_unique_v<typename Candidate::Sensors>;
    requires std::same_as<typename Candidate::MeasurementModels,
                          MeasurementModelsFromSensors_t<typename Candidate::Sensors>>;
};

} // namespace navkit::api::config
