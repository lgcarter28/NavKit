// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileRecord.hpp"

#include <concepts>

namespace navkit::core::profiling
{

template<typename Candidate>
concept ClockPolicy = requires(typename Candidate::Tick start, typename Candidate::Tick end) {
    typename Candidate::Tick;

    { Candidate::now() } -> std::same_as<typename Candidate::Tick>;
    { end - start } -> std::same_as<typename Candidate::Tick>;
};

template<typename Candidate, typename Clock>
concept ProfileSinkPolicy =
    ClockPolicy<Clock> && requires(ProfileRecord<typename Clock::Tick> record) {
        { Candidate::record(record) } -> std::same_as<void>;
    };

template<typename Candidate>
concept ProfileScopePolicy = std::destructible<Candidate>;

template<typename Candidate>
concept ProfilerPolicy = requires {
    typename Candidate::Scope;

    requires ProfileScopePolicy<typename Candidate::Scope>;
    { Candidate::profile(ProfilePoint{}) } -> std::same_as<typename Candidate::Scope>;
};

} // namespace navkit::core::profiling
