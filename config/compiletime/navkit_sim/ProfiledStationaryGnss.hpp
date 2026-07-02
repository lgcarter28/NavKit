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

namespace navkit::config::navkit_sim
{

struct ProfiledStationaryGnssNumericConfig
{
    using Scalar_t = navkit::core::Scalar_t;
    using Time_t = navkit::core::Time_t;
};

struct ProfiledStationaryGnssMeasurementStatisticsConfig
{
    static constexpr bool EnableMeasurementStatistics = true;
};

struct ProfiledStationaryGnssBufferConfig
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

struct ProfiledStationaryGnssConfig
{
    using Numeric = ProfiledStationaryGnssNumericConfig;
    using MeasurementStatistics = ProfiledStationaryGnssMeasurementStatisticsConfig;
    using GnssBuffer = ProfiledStationaryGnssBufferConfig;

    using ProfileClock = HostSteadyMicrosecondClock;
    using ProfileTick = typename ProfileClock::Tick;
    using ProfileSink = navkit::core::profiling::RingBufferProfileSink<ProfileTick, 4096U>;
    using Profiler = navkit::core::profiling::ScopedProfiler<ProfileClock, ProfileSink>;
};

} // namespace navkit::config::navkit_sim

namespace navkit::config
{

using SelectedConfig = navkit_sim::ProfiledStationaryGnssConfig;

} // namespace navkit::config

namespace navkit::config::navkit_sim
{

static_assert(navkit::core::config::NumericConfigPolicy<ProfiledStationaryGnssNumericConfig>);
static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
              ProfiledStationaryGnssMeasurementStatisticsConfig>);
static_assert(navkit::core::estimation::BufferConfigPolicy<ProfiledStationaryGnssBufferConfig>);
static_assert(navkit::core::profiling::ClockPolicy<HostSteadyMicrosecondClock>);
static_assert(navkit::core::profiling::ProfileSinkPolicy<ProfiledStationaryGnssConfig::ProfileSink,
                                                         HostSteadyMicrosecondClock>);
static_assert(navkit::core::profiling::ProfilerPolicy<ProfiledStationaryGnssConfig::Profiler>);
static_assert(navkit::core::config::ConfigPolicy<ProfiledStationaryGnssConfig>);
static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
              ProfiledStationaryGnssConfig::MeasurementStatistics>);
static_assert(
    navkit::core::estimation::BufferConfigPolicy<ProfiledStationaryGnssConfig::GnssBuffer>);

} // namespace navkit::config::navkit_sim
