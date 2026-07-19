// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"

#include <chrono>
#include <cstdint>
#include <string_view>

namespace navkit::config::navkit::products::components
{

struct NullProfiling
{
    using Profiler = core::profiling::NullProfiler;
};

struct HostSteadyMicrosecondClock
{
    using Tick = std::uint64_t;

    static constexpr std::string_view source = "std::chrono::steady_clock";
    static constexpr double tick_period_us = 1.0;

    static Tick now()
    {
        const std::chrono::steady_clock::duration now =
            std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<Tick>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }
};

struct HostRingBufferProfiling
{
    using ProfileClock = HostSteadyMicrosecondClock;
    using ProfileTick = typename ProfileClock::Tick;
    using ProfileSink = core::profiling::RingBufferProfileSink<ProfileTick, 4096U>;
    using Profiler = core::profiling::ScopedProfiler<ProfileClock, ProfileSink>;
};

} // namespace navkit::config::navkit::products::components
