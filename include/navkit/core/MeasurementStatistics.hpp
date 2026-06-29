#pragma once

#include "navkit/core/Config.hpp"

#include <limits>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace navkit
{

template<typename Model>
struct MeasurementStatistics
{
    using O_t = typename Model::O_t;
    using H_t = typename Model::H_t;
    using R_t = typename Model::R_t;
    using K_t = typename Model::K_t;

    static constexpr int M = Model::M;
    static constexpr std::string_view ModelName = Model::Name;

    bool valid{false};
    bool accepted{false};
    Time_t time{0.0};

    O_t innovation{O_t::Zero()};
    R_t innovation_covariance{R_t::Zero()};
    R_t measurement_covariance{R_t::Zero()};
    H_t jacobian_h{H_t::Zero()};
    K_t kalman_gain{K_t::Zero()};
    Scalar_t nis{std::numeric_limits<Scalar_t>::quiet_NaN()};

    void reset()
    {
        valid = false;
        accepted = false;
        time = 0.0;
        innovation.setZero();
        innovation_covariance.setZero();
        measurement_covariance.setZero();
        jacobian_h.setZero();
        kalman_gain.setZero();
        nis = std::numeric_limits<Scalar_t>::quiet_NaN();
    }
};

template<typename T, typename Tuple>
struct tuple_contains;

template<typename T>
struct tuple_contains<T, std::tuple<>> : std::false_type
{};

template<typename T, typename Head, typename... Tail>
struct tuple_contains<T, std::tuple<Head, Tail...>>
    : std::conditional_t<std::is_same_v<T, Head>,
                         std::true_type,
                         tuple_contains<T, std::tuple<Tail...>>>
{};

template<typename T, typename Tuple>
inline constexpr bool tuple_contains_v = tuple_contains<T, Tuple>::value;

template<typename T, typename Tuple>
struct tuple_index;

template<typename T, typename... Tail>
struct tuple_index<T, std::tuple<T, Tail...>> : std::integral_constant<std::size_t, 0>
{};

template<typename T, typename Head, typename... Tail>
struct tuple_index<T, std::tuple<Head, Tail...>>
    : std::integral_constant<std::size_t, 1 + tuple_index<T, std::tuple<Tail...>>::value>
{};

template<typename T, typename Tuple>
inline constexpr std::size_t tuple_index_v = tuple_index<T, Tuple>::value;

template<typename ModelTuple>
struct MeasurementStatisticsTuple;

template<typename... Models>
struct MeasurementStatisticsTuple<std::tuple<Models...>>
{
    using type = std::tuple<MeasurementStatistics<Models>...>;
};

template<typename ModelTuple>
using MeasurementStatisticsTuple_t = typename MeasurementStatisticsTuple<ModelTuple>::type;

} // namespace navkit
