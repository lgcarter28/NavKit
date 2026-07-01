// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <Eigen/Dense>
#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Model>
struct MeasurementStatistics
{
    using O_t = typename Model::O_t;
    using R_t = typename Model::R_t;
    using H_t = typename Model::H_t;
    using K_t = typename Model::K_t;

    bool valid{false};
    bool accepted{false};
    Time_t time{0.0};

    O_t innovation{O_t::Zero()};
    R_t innovation_covariance{R_t::Zero()};
    R_t measurement_covariance{R_t::Zero()};
    H_t jacobian_h{H_t::Zero()};
    K_t kalman_gain{K_t::Zero()};

    Scalar_t nis{0.0};
};

template<typename ModelTuple>
struct MeasurementStatisticsTuple;

template<typename... Models>
struct MeasurementStatisticsTuple<std::tuple<Models...>>
{
    using type = std::tuple<MeasurementStatistics<Models>...>;
};

template<typename ModelTuple>
using MeasurementStatisticsTuple_t = typename MeasurementStatisticsTuple<ModelTuple>::type;

template<typename T, typename Tuple>
struct tuple_contains;

template<typename T>
struct tuple_contains<T, std::tuple<>> : std::false_type
{};

template<typename T, typename... Rest>
struct tuple_contains<T, std::tuple<T, Rest...>> : std::true_type
{};

template<typename T, typename First, typename... Rest>
struct tuple_contains<T, std::tuple<First, Rest...>> : tuple_contains<T, std::tuple<Rest...>>
{};

template<typename T, typename Tuple>
inline constexpr bool tuple_contains_v = tuple_contains<T, Tuple>::value;

template<typename T, typename Tuple>
struct tuple_index;

template<typename T, typename... Rest>
struct tuple_index<T, std::tuple<T, Rest...>> : std::integral_constant<std::size_t, 0>
{};

template<typename T, typename First, typename... Rest>
struct tuple_index<T, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t, 1 + tuple_index<T, std::tuple<Rest...>>::value>
{};

template<typename T, typename Tuple>
inline constexpr std::size_t tuple_index_v = tuple_index<T, Tuple>::value;

} // namespace navkit::core::estimation
