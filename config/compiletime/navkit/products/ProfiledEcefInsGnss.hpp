// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>

namespace navkit::config::navkit::products::profiled_ecef_ins_gnss
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
    using StateDef = core::estimation::DefaultInsStateDef;
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
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;
    using InitialCovariance_t = core::estimation::InitialCovariance<StateDef>;
    inline static const InitialCovariance_t initial_covariance =
        core::estimation::diagonal_initial_covariance<StateDef>(
            core::estimation::InitialCovarianceDiagonal<StateDef>{
                .values =
                    {
                        // Pos
                        10000.0,
                        10000.0,
                        10000.0,
                        // Vel
                        100.0,
                        100.0,
                        100.0,
                        // AttRotVec
                        0.007615435494667714,
                        0.007615435494667714,
                        0.030461741978670857,
                        // GyroB
                        7.615435494667714e-7,
                        7.615435494667714e-7,
                        7.615435494667714e-7,
                        // AccB
                        9.61703842225e-7,
                        9.61703842225e-7,
                        9.61703842225e-7,
                    },
            });
    static constexpr std::size_t imu_buffer_capacity = 1024U;
    static constexpr std::size_t covariance_history_capacity = 256U;
    static constexpr core::Time_t covariance_update_rate_hz = 100.0;
    static constexpr bool apply_coning_sculling_compensation = true;
    using Propagation =
        core::estimation::EcefInsPropagation<Planet,
                                             Gravity,
                                             core::estimation::EcefInsZeroProcessNoise,
                                             imu_buffer_capacity,
                                             covariance_history_capacity,
                                             covariance_update_rate_hz,
                                             apply_coning_sculling_compensation>;

    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       Sensors,
                                       Profiler>;
    using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;
    using Navigator =
        core::estimation::Navigator<Filter, Sensors, Propagation, NavigatorUpdate, Profiler>;
};

static_assert(api::config::NavKitProductConfigPolicy<ProductConfig>);

} // namespace navkit::config::navkit::products::profiled_ecef_ins_gnss

namespace navkit::config::navkit
{

using ProfiledEcefInsGnssConfig = products::profiled_ecef_ins_gnss::ProductConfig;

} // namespace navkit::config::navkit
