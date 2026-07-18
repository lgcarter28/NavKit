// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"

#include <cstddef>
#include <tuple>

namespace navkit::config::navkit::products::ecef_ins_gnss
{

struct NumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct ProductConfig
{
    using Numeric = NumericConfig;

    // Public product graph.
    using StateDef = core::estimation::DefaultInsStateDef;
    using PrimaryGnssPositionMeasurementModel = core::models::GnssPosModel<StateDef>;
    using PrimaryGnssVelocityMeasurementModel = core::models::GnssVelModel<StateDef>;
    static constexpr core::estimation::SensorId primary_gnss_position_sensor_id = 0U;
    static constexpr core::estimation::SensorId primary_gnss_velocity_sensor_id = 1U;
    static constexpr std::size_t primary_gnss_buffer_size = 16U;
    using PrimaryGnssDiagnostics = core::estimation::DefaultSensorDiagnostics;
    using PrimaryGnssPositionSensor =
        core::estimation::Sensor<primary_gnss_position_sensor_id,
                                 PrimaryGnssPositionMeasurementModel,
                                 primary_gnss_buffer_size,
                                 core::estimation::GnssFixedNoisePolicy,
                                 PrimaryGnssDiagnostics>;
    using PrimaryGnssVelocitySensor =
        core::estimation::Sensor<primary_gnss_velocity_sensor_id,
                                 PrimaryGnssVelocityMeasurementModel,
                                 primary_gnss_buffer_size,
                                 core::estimation::GnssFixedNoisePolicy,
                                 PrimaryGnssDiagnostics>;

    using Sensors = std::tuple<PrimaryGnssPositionSensor, PrimaryGnssVelocitySensor>;

    using Profiler = core::profiling::NullProfiler;
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

} // namespace navkit::config::navkit::products::ecef_ins_gnss

namespace navkit::config::navkit
{

using EcefInsGnssConfig = products::ecef_ins_gnss::ProductConfig;

} // namespace navkit::config::navkit
