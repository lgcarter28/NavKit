// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"

#include <cstddef>
#include <tuple>

namespace navkit::config::navkit::products::stationary_gnss
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

    using Profiler = core::profiling::NullProfiler;
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;
    static constexpr std::size_t imu_buffer_capacity = 1024U;
    using Propagation =
        core::estimation::EcefInsPropagation<Planet,
                                             Gravity,
                                             core::estimation::EcefInsZeroProcessNoise,
                                             imu_buffer_capacity>;

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

} // namespace navkit::config::navkit::products::stationary_gnss

namespace navkit::config::navkit
{

using StationaryGnssConfig = products::stationary_gnss::ProductConfig;

} // namespace navkit::config::navkit
