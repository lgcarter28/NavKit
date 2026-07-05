// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
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
    using PrimaryGnssModel = core::models::GnssPosModel<StateDef>;
    static constexpr core::estimation::SensorId primary_gnss_sensor_id = 0U;
    static constexpr std::size_t primary_gnss_buffer_size = 16U;
    using PrimaryGnssSensor = core::estimation::Sensor<primary_gnss_sensor_id,
                                                       PrimaryGnssModel,
                                                       primary_gnss_buffer_size,
                                                       core::estimation::GnssFixedNoisePolicy>;

    using Sensors = std::tuple<PrimaryGnssSensor>;
    using MeasurementStatisticsTuple =
        std::tuple<core::estimation::MeasurementStatistics<PrimaryGnssSensor>>;

    using Profiler = core::profiling::NullProfiler;
    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       MeasurementStatisticsTuple,
                                       Profiler>;
    using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;
    using Navigator = core::estimation::Navigator<Filter, Sensors, NavigatorUpdate, Profiler>;
};

static_assert(api::config::NavKitProductConfigPolicy<ProductConfig>);

} // namespace navkit::config::navkit::products::stationary_gnss

namespace navkit::config::navkit
{

using StationaryGnssConfig = products::stationary_gnss::ProductConfig;

} // namespace navkit::config::navkit
