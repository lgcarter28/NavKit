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
#include "navkit/core/profiling/NullProfiler.hpp"

#include <cstddef>
#include <tuple>

namespace navkit::config::navkit
{

struct StationaryGnssNumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct StationaryGnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct StationaryGnssConfig
{
    using Numeric = StationaryGnssNumericConfig;
    using GnssBuffer = StationaryGnssBufferConfig;

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

    using Profiler = core::profiling::NullProfiler;
    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       MeasurementStatisticsTuple,
                                       Profiler>;
    using Navigator =
        core::estimation::Navigator<Filter, Sensors, core::estimation::UpdatePostFilter, Profiler>;
};

static_assert(core::config::NumericConfigPolicy<StationaryGnssNumericConfig>);
static_assert(core::estimation::BufferConfigPolicy<StationaryGnssBufferConfig>);
static_assert(core::config::ConfigPolicy<StationaryGnssConfig>);
static_assert(core::estimation::MeasurementStatisticsCollectionPolicy<
              StationaryGnssConfig::MeasurementStatisticsTuple>);
static_assert(core::estimation::BufferConfigPolicy<StationaryGnssConfig::GnssBuffer>);
static_assert(api::config::NavKitProductConfigPolicy<StationaryGnssConfig>);

} // namespace navkit::config::navkit
