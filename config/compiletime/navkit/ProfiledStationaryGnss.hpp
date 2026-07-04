// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicies.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>

namespace navkit::config::navkit
{

struct ProfiledStationaryGnssNumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
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
    static constexpr std::string_view ProfileClockSource = "std::chrono::steady_clock";
    static constexpr double ProfileTickPeriodUs = 1.0;

    using Numeric = ProfiledStationaryGnssNumericConfig;
    using GnssBuffer = ProfiledStationaryGnssBufferConfig;

    // Public product graph.
    using StateDef = core::estimation::InsStateDef;
    using PrimaryGnssModel = core::models::GnssPosModel<StateDef>;
    static constexpr core::estimation::SensorId PrimaryGnssSensorId = 0U;
    using PrimaryGnssSensor = core::estimation::Sensor<PrimaryGnssSensorId,
                                                       PrimaryGnssModel,
                                                       GnssBuffer::BufferSize,
                                                       core::estimation::GnssFixedNoisePolicy>;

    using Sensors = std::tuple<PrimaryGnssSensor>;
    using MeasurementStatisticsTuple =
        std::tuple<core::estimation::MeasurementStatistics<PrimaryGnssSensor>>;

    using ProfileClock = HostSteadyMicrosecondClock;
    using ProfileTick = typename ProfileClock::Tick;
    using ProfileSink = core::profiling::RingBufferProfileSink<ProfileTick, 4096U>;
    using Profiler = core::profiling::ScopedProfiler<ProfileClock, ProfileSink>;

    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       MeasurementStatisticsTuple,
                                       Profiler>;
    using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;
    using Navigator = core::estimation::Navigator<Filter, Sensors, NavigatorUpdate, Profiler>;
};

static_assert(api::config::NavKitProductConfigPolicy<ProfiledStationaryGnssConfig>);

} // namespace navkit::config::navkit
