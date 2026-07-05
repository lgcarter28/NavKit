// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/TupleTraits.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

namespace detail
{

template<typename Candidate>
struct is_sensor : std::bool_constant<SensorPolicy<Candidate>>
{};

} // namespace detail

template<typename Candidate>
concept SensorTuplePolicy = requires {
    typename std::tuple_size<std::remove_cvref_t<Candidate>>::type;
} && navkit::core::containers::tuple_all_v<detail::is_sensor, std::remove_cvref_t<Candidate>>;

} // namespace navkit::core::estimation
