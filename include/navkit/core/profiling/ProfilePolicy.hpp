// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileRecord.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::profiling
{

template<typename Sink, typename Clock, typename = void>
struct ProfileSinkRecord
{
    using type = ProfileRecord<typename Clock::Tick>;
};

template<typename Sink, typename Clock>
struct ProfileSinkRecord<Sink, Clock, std::void_t<typename Sink::Record>>
{
    using type = typename Sink::Record;
};

template<typename Sink, typename Clock>
using ProfileSinkRecord_t = typename ProfileSinkRecord<Sink, Clock>::type;

template<typename Candidate>
concept ClockPolicy = requires(typename Candidate::Tick start, typename Candidate::Tick end) {
    typename Candidate::Tick;

    { Candidate::now() } -> std::same_as<typename Candidate::Tick>;
    { end - start } -> std::same_as<typename Candidate::Tick>;
};

template<typename Candidate, typename Clock>
concept ProfileSinkPolicy =
    ClockPolicy<Clock> && requires(ProfileSinkRecord_t<Candidate, Clock> record) {
        typename ProfileSinkRecord_t<Candidate, Clock>::Tick_t;

        requires std::same_as<typename ProfileSinkRecord_t<Candidate, Clock>::Tick_t,
                              typename Clock::Tick>;
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
