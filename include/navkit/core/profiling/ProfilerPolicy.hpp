// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileScopePolicy.hpp"

#include <concepts>

namespace navkit::core::profiling
{

template<typename Candidate>
concept ProfilerPolicy = requires {
    typename Candidate::Scope;

    requires ProfileScopePolicy<typename Candidate::Scope>;
    { Candidate::profile(ProfilePoint{}) } -> std::same_as<typename Candidate::Scope>;
};

} // namespace navkit::core::profiling
