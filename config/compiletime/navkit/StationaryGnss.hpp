// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"

#include <cstddef>

namespace navkit::config::navkit
{

struct StationaryGnssNumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct StationaryGnssMeasurementStatisticsConfig
{
    static constexpr bool EnableMeasurementStatistics = true;
};

struct StationaryGnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct StationaryGnssConfig
{
    using Numeric = StationaryGnssNumericConfig;
    using MeasurementStatistics = StationaryGnssMeasurementStatisticsConfig;
    using GnssBuffer = StationaryGnssBufferConfig;
};

static_assert(core::config::NumericConfigPolicy<StationaryGnssNumericConfig>);
static_assert(
    core::estimation::MeasurementStatisticsConfigPolicy<StationaryGnssMeasurementStatisticsConfig>);
static_assert(core::estimation::BufferConfigPolicy<StationaryGnssBufferConfig>);
static_assert(core::config::ConfigPolicy<StationaryGnssConfig>);
static_assert(core::estimation::MeasurementStatisticsConfigPolicy<
              StationaryGnssConfig::MeasurementStatistics>);
static_assert(core::estimation::BufferConfigPolicy<StationaryGnssConfig::GnssBuffer>);

} // namespace navkit::config::navkit
