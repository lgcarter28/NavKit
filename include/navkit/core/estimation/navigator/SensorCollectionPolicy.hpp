// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Candidate>
concept SensorCollectionPolicy =
    requires { typename std::tuple_size<std::remove_cvref_t<Candidate>>::type; };

} // namespace navkit::core::estimation
