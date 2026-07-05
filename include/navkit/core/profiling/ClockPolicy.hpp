// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::profiling
{

template<typename Candidate>
concept ClockPolicy = requires(typename Candidate::Tick start, typename Candidate::Tick end) {
    typename Candidate::Tick;

    { Candidate::now() } -> std::same_as<typename Candidate::Tick>;
    { end - start } -> std::same_as<typename Candidate::Tick>;
};

} // namespace navkit::core::profiling
