// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ClockPolicy.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileScope.hpp"
#include "navkit/core/profiling/ProfileSinkPolicy.hpp"

namespace navkit::core::profiling
{

template<ClockPolicy Clock, ProfileSinkPolicy<Clock> Sink>
struct ScopedProfiler
{
    using Scope = ProfileScope<Clock, Sink>;

    [[nodiscard]] static Scope profile(ProfilePoint point)
    {
        return Scope{point};
    }
};

} // namespace navkit::core::profiling
