// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/app_support/EmulatorBindingTraits.hpp"

#include <concepts>
#include <tuple>

namespace navkit::app_support
{

template<typename BindingTuple, typename SensorTuple>
concept EmulatorBindingPolicy = requires { typename std::tuple_size<BindingTuple>::type; } &&
                                emulator_binding_ids_unique_v<BindingTuple> &&
                                emulator_binding_sensors_valid_v<BindingTuple, SensorTuple>;

} // namespace navkit::app_support
