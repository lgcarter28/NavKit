// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/core/estimation/sensor/SensorTuple.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::app_support
{

template<typename BindingTuple>
struct emulator_binding_ids_unique;

template<>
struct emulator_binding_ids_unique<std::tuple<>> : std::true_type
{};

template<typename First, typename... Rest>
struct emulator_binding_ids_unique<std::tuple<First, Rest...>>
    : std::bool_constant<((First::Id != Rest::Id) && ...) &&
                         emulator_binding_ids_unique<std::tuple<Rest...>>::value>
{};

template<typename BindingTuple>
inline constexpr bool emulator_binding_ids_unique_v =
    emulator_binding_ids_unique<BindingTuple>::value;

template<SensorId IdValue, typename BindingTuple>
struct binding_id_count;

template<SensorId IdValue>
struct binding_id_count<IdValue, std::tuple<>> : std::integral_constant<std::size_t, 0U>
{};

template<SensorId IdValue, typename First, typename... Rest>
struct binding_id_count<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t,
                             (First::Id == IdValue ? 1U : 0U) +
                                 binding_id_count<IdValue, std::tuple<Rest...>>::value>
{};

template<SensorId IdValue, typename BindingTuple>
inline constexpr std::size_t binding_id_count_v = binding_id_count<IdValue, BindingTuple>::value;

template<SensorId IdValue, typename BindingTuple>
inline constexpr bool binding_id_exists_v = binding_id_count_v<IdValue, BindingTuple> > 0U;

template<SensorId IdValue, typename BindingTuple>
struct binding_index_from_id;

template<SensorId IdValue, typename First, typename... Rest>
    requires(First::Id == IdValue)
struct binding_index_from_id<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t, 0U>
{};

template<SensorId IdValue, typename First, typename... Rest>
    requires(First::Id != IdValue)
struct binding_index_from_id<IdValue, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t,
                             1U + binding_index_from_id<IdValue, std::tuple<Rest...>>::value>
{};

template<SensorId IdValue, typename BindingTuple>
inline constexpr std::size_t BindingIndexFromId_v =
    binding_index_from_id<IdValue, BindingTuple>::value;

template<SensorId IdValue, typename BindingTuple>
using BindingFromId_t =
    std::tuple_element_t<BindingIndexFromId_v<IdValue, BindingTuple>, BindingTuple>;

template<SensorId IdValue, typename BindingTuple>
using EmulatorFromId_t = typename BindingFromId_t<IdValue, BindingTuple>::Emulator_t;

template<typename BindingTuple, typename SensorTuple>
struct emulator_binding_sensors_valid;

template<typename SensorTuple, typename... Bindings>
struct emulator_binding_sensors_valid<std::tuple<Bindings...>, SensorTuple>
    : std::bool_constant<(
          (navkit::core::estimation::sensor_type_contains_v<typename Bindings::Sensor_t,
                                                            SensorTuple> &&
           navkit::core::estimation::sensor_id_exists_v<Bindings::Id, SensorTuple> &&
           (Bindings::Id == Bindings::Sensor_t::Id)) &&
          ...)>
{};

template<typename BindingTuple, typename SensorTuple>
inline constexpr bool emulator_binding_sensors_valid_v =
    emulator_binding_sensors_valid<BindingTuple, SensorTuple>::value;

} // namespace navkit::app_support
