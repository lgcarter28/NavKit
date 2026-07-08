// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTraits.hpp"

#include <tuple>
#include <type_traits>
#include <utility>

namespace navkit::app_support
{

namespace detail
{

template<typename BindingTuple, typename Logger, std::size_t... Is>
inline constexpr bool emulator_bindings_satisfy_logger_impl(std::index_sequence<Is...>)
{
    return (EmulatorBindingPolicy<std::tuple_element_t<Is, BindingTuple>, Logger> && ...);
}

template<typename BindingTuple, typename Logger>
inline constexpr bool emulator_bindings_satisfy_logger_v =
    emulator_bindings_satisfy_logger_impl<BindingTuple, Logger>(
        std::make_index_sequence<std::tuple_size_v<BindingTuple>>{});

} // namespace detail

template<typename BindingTuple, typename SensorTuple, typename Logger>
concept EmulatorBindingTuplePolicy =
    requires { typename std::tuple_size<std::remove_cvref_t<BindingTuple>>::type; } &&
    detail::emulator_bindings_satisfy_logger_v<std::remove_cvref_t<BindingTuple>, Logger> &&
    emulator_binding_ids_unique_v<std::remove_cvref_t<BindingTuple>> &&
    emulator_binding_sensors_valid_v<std::remove_cvref_t<BindingTuple>, SensorTuple>;

} // namespace navkit::app_support
