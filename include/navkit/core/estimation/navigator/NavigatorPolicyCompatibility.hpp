// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicy.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation::detail
{

template<typename Filter, typename Update, typename SensorTuple, typename Indices>
struct NavigatorPolicyCompatibility;

template<typename Filter, typename Update, typename SensorTuple, std::size_t... Is>
struct NavigatorPolicyCompatibility<Filter, Update, SensorTuple, std::index_sequence<Is...>>
{
    using Tuple = std::remove_cvref_t<SensorTuple>;

    static constexpr bool value =
        ((SensorFilterPolicy<Filter, std::tuple_element_t<Is, Tuple>> &&
          UpdatePolicy<Update, Filter, std::tuple_element_t<Is, Tuple>>) &&
         ...);
};

template<typename Filter, typename Update, SensorCollectionPolicy SensorTuple>
inline constexpr bool navigator_policy_compatible_v = NavigatorPolicyCompatibility<
    Filter,
    Update,
    SensorTuple,
    std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<SensorTuple>>>>::value;

} // namespace navkit::core::estimation::detail
