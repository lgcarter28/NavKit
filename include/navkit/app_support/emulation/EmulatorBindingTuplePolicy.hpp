// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTraits.hpp"
#include "navkit/core/containers/TupleTraits.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::app_support
{

namespace detail
{

template<typename Candidate>
struct is_emulator_binding : std::bool_constant<EmulatorBindingPolicy<Candidate>>
{};

} // namespace detail

template<typename BindingTuple, typename SensorTuple>
concept EmulatorBindingTuplePolicy =
    requires { typename std::tuple_size<std::remove_cvref_t<BindingTuple>>::type; } &&
    navkit::core::containers::tuple_all_v<detail::is_emulator_binding,
                                          std::remove_cvref_t<BindingTuple>> &&
    emulator_binding_ids_unique_v<std::remove_cvref_t<BindingTuple>> &&
    emulator_binding_sensors_valid_v<std::remove_cvref_t<BindingTuple>, SensorTuple>;

} // namespace navkit::app_support
