// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>

namespace navkit::config::navkit::products::profiled_stationary_gnss
{

struct NumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct HostSteadyMicrosecondClock
{
    using Tick = std::uint64_t;

    static constexpr std::string_view source = "std::chrono::steady_clock";
    static constexpr double tick_period_us = 1.0;

    static Tick now()
    {
        const auto now = std::chrono::steady_clock::now().time_since_epoch();
        return static_cast<Tick>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }
};

struct ProductConfig
{
    using Numeric = NumericConfig;

    // Public product graph.
    using StateDef = core::estimation::InsStateDef;
    using PrimaryGnssMeasurementModel = core::models::GnssPosModel<StateDef>;
    static constexpr core::estimation::SensorId primary_gnss_sensor_id = 0U;
    static constexpr std::size_t primary_gnss_buffer_size = 16U;
    using PrimaryGnssDiagnostics = core::estimation::DefaultSensorDiagnostics;
    using PrimaryGnssSensor = core::estimation::Sensor<primary_gnss_sensor_id,
                                                       PrimaryGnssMeasurementModel,
                                                       primary_gnss_buffer_size,
                                                       core::estimation::GnssFixedNoisePolicy,
                                                       PrimaryGnssDiagnostics>;

    using Sensors = std::tuple<PrimaryGnssSensor>;

    using ProfileClock = HostSteadyMicrosecondClock;
    using ProfileTick = typename ProfileClock::Tick;
    using ProfileSink = core::profiling::RingBufferProfileSink<ProfileTick, 4096U>;
    using Profiler = core::profiling::ScopedProfiler<ProfileClock, ProfileSink>;

    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       Sensors,
                                       Profiler>;
    using Propagation = core::estimation::NoOpPropagation;
    using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;
    using Navigator =
        core::estimation::Navigator<Filter, Sensors, Propagation, NavigatorUpdate, Profiler>;
};

static_assert(api::config::NavKitProductConfigPolicy<ProductConfig>);

} // namespace navkit::config::navkit::products::profiled_stationary_gnss

namespace navkit::config::navkit
{

using ProfiledStationaryGnssConfig = products::profiled_stationary_gnss::ProductConfig;

} // namespace navkit::config::navkit
