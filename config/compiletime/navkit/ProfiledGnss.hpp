// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace navkit::config::navkit
{

struct ProfiledGnssNumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct ProfiledGnssMeasurementStatisticsConfig
{
    static constexpr bool EnableMeasurementStatistics = true;
};

struct ProfiledGnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct HostSteadyMicrosecondClock
{
    using Tick = std::uint64_t;

    static Tick now()
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<Tick>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }
};

struct ProfiledGnssConfig
{
    static constexpr std::string_view ProfileClockSource = "std::chrono::steady_clock";
    static constexpr double ProfileTickPeriodUs = 1.0;

    using Numeric = ProfiledGnssNumericConfig;
    using MeasurementStatistics = ProfiledGnssMeasurementStatisticsConfig;
    using GnssBuffer = ProfiledGnssBufferConfig;

    using ProfileClock = HostSteadyMicrosecondClock;
    using ProfileTick = typename ProfileClock::Tick;

    using ProfileSink = core::profiling::RingBufferProfileSink<ProfileTick, 4096U>;
    using Profiler = core::profiling::ScopedProfiler<ProfileClock, ProfileSink>;
};

static_assert(core::config::NumericConfigPolicy<ProfiledGnssNumericConfig>);
static_assert(
    core::estimation::MeasurementStatisticsConfigPolicy<ProfiledGnssMeasurementStatisticsConfig>);
static_assert(core::estimation::BufferConfigPolicy<ProfiledGnssBufferConfig>);
static_assert(core::profiling::ClockPolicy<HostSteadyMicrosecondClock>);
static_assert(core::profiling::ProfileSinkPolicy<ProfiledGnssConfig::ProfileSink,
                                                 HostSteadyMicrosecondClock>);
static_assert(core::profiling::ProfilerPolicy<ProfiledGnssConfig::Profiler>);
static_assert(core::config::ConfigPolicy<ProfiledGnssConfig>);
static_assert(
    core::estimation::MeasurementStatisticsConfigPolicy<ProfiledGnssConfig::MeasurementStatistics>);
static_assert(core::estimation::BufferConfigPolicy<ProfiledGnssConfig::GnssBuffer>);

} // namespace navkit::config::navkit
