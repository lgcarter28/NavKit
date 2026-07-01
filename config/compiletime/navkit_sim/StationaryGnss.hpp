// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"

#include <cstddef>

namespace navkit::config::navkit_sim
{

struct StationaryGnssNumericConfig
{
    using Scalar_t = navkit::core::Scalar_t;
    using Time_t = navkit::core::Time_t;
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

} // namespace navkit::config::navkit_sim

namespace navkit::config
{

using SelectedConfig = navkit_sim::StationaryGnssConfig;

} // namespace navkit::config

namespace navkit::config::navkit_sim
{

static_assert(navkit::core::config::NumericConfigPolicy<StationaryGnssNumericConfig>);
static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
              StationaryGnssMeasurementStatisticsConfig>);
static_assert(navkit::core::estimation::BufferConfigPolicy<StationaryGnssBufferConfig>);
static_assert(navkit::core::config::ConfigPolicy<StationaryGnssConfig>);
static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
              StationaryGnssConfig::MeasurementStatistics>);
static_assert(navkit::core::estimation::BufferConfigPolicy<StationaryGnssConfig::GnssBuffer>);

} // namespace navkit::config::navkit_sim
