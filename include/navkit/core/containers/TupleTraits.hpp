// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::core::containers
{

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
struct tuple_index<T, std::tuple<T, Rest...>> : std::integral_constant<std::size_t, 0U>
{};

template<typename T, typename First, typename... Rest>
struct tuple_index<T, std::tuple<First, Rest...>>
    : std::integral_constant<std::size_t, 1U + tuple_index<T, std::tuple<Rest...>>::value>
{};

template<typename T, typename Tuple>
inline constexpr std::size_t tuple_index_v = tuple_index<T, Tuple>::value;

template<template<typename> typename Predicate, typename Tuple>
struct tuple_all;

template<template<typename> typename Predicate>
struct tuple_all<Predicate, std::tuple<>> : std::true_type
{};

template<template<typename> typename Predicate, typename First, typename... Rest>
struct tuple_all<Predicate, std::tuple<First, Rest...>>
    : std::bool_constant<Predicate<First>::value &&
                         tuple_all<Predicate, std::tuple<Rest...>>::value>
{};

template<template<typename> typename Predicate, typename Tuple>
inline constexpr bool tuple_all_v = tuple_all<Predicate, Tuple>::value;

} // namespace navkit::core::containers
