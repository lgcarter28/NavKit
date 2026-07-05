// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ClockPolicy.hpp"
#include "navkit/core/profiling/ProfileSinkRecord.hpp"

#include <concepts>

namespace navkit::core::profiling
{

template<typename Candidate, typename Clock>
concept ProfileSinkPolicy =
    ClockPolicy<Clock> && requires(ProfileSinkRecord_t<Candidate, Clock> record) {
        typename ProfileSinkRecord_t<Candidate, Clock>::Tick_t;

        requires std::same_as<typename ProfileSinkRecord_t<Candidate, Clock>::Tick_t,
                              typename Clock::Tick>;
        { Candidate::record(record) } -> std::same_as<void>;
    };

} // namespace navkit::core::profiling
