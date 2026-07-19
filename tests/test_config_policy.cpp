// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/SelectedConfig.hpp"
#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/MeasurementStatisticsStorage.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/products/MinimalConfig.hpp"

#include <cstddef>
#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::core::config::test
{

struct MissingScalarConfig
{
    using Time_t = double;
};

struct GnssOnlyBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct MissingBufferSizeConfig
{};

struct ZeroCapacityBufferConfig
{
    static constexpr std::size_t BufferSize = 0;
};

struct MinimalConfig
{
    using Numeric = navkit::config::navkit::MinimalConfig::Numeric;
};

struct MissingNumericConfig
{
    using GnssBuffer = navkit::config::navkit::MinimalConfig::GnssBuffer;
};

struct NumericOnlyConfig
{
    using Numeric = navkit::config::navkit::MinimalConfig::Numeric;
};

TEST_CASE("Concrete config slices satisfy narrow configuration concepts")
{
    using ExampleConfig = navkit::config::navkit::MinimalConfig;
    using SelectedAppConfig = navkit::selected_config::Config;
    using SimConfig = navkit::app_support::NavKitConfig_t<navkit::selected_config::Config>;

    static_assert(NumericConfigPolicy<ExampleConfig::Numeric>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<ExampleConfig::GnssBuffer>);
    static_assert(ConfigPolicy<ExampleConfig>);
    static_assert(ConfigPolicy<SimConfig>);
    static_assert(ConfigPolicy<MinimalConfig>);
    static_assert(navkit::api::config::NavKitProductConfigPolicy<SimConfig>);
    static_assert(navkit::core::estimation::sensor_ids_unique_v<SimConfig::Sensors>);
    static_assert(std::is_same_v<navkit::core::estimation::SensorFromId_t<
                                     SimConfig::SensorGraph::primary_gnss_position_sensor_id,
                                     SimConfig::Sensors>,
                                 SimConfig::PrimaryGnssPositionSensor>);
    static_assert(std::is_same_v<navkit::core::estimation::SensorFromId_t<
                                     SimConfig::SensorGraph::primary_gnss_velocity_sensor_id,
                                     SimConfig::Sensors>,
                                 SimConfig::PrimaryGnssVelocitySensor>);
    static_assert(std::is_same_v<navkit::app_support::LoggerConfig_t<SelectedAppConfig>,
                                 SelectedAppConfig::Logger>);
    static_assert(std::is_same_v<
                  SimConfig::Filter::MeasurementStatisticsTuple_t,
                  navkit::core::estimation::MeasurementStatisticsStorage_t<SimConfig::Sensors>>);
    static_assert(std::is_same_v<SimConfig::PrimaryGnssPositionSensor::Diagnostics_t,
                                 SimConfig::PrimaryGnssDiagnostics>);

    static_assert(std::is_same_v<ExampleConfig::Numeric::Scalar_t, navkit::core::Scalar_t>);
    static_assert(std::is_same_v<ExampleConfig::Numeric::Time_t, navkit::core::Time_t>);

    CHECK(ExampleConfig::GnssBuffer::BufferSize > 0);
    CHECK(SimConfig::SensorGraph::primary_gnss_buffer_size > 0U);
}

TEST_CASE("Narrow config concepts reject only their own missing capabilities")
{
    static_assert(!NumericConfigPolicy<MissingScalarConfig>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<GnssOnlyBufferConfig>);
    static_assert(!navkit::core::estimation::BufferConfigPolicy<MissingBufferSizeConfig>);
    static_assert(!navkit::core::estimation::BufferConfigPolicy<ZeroCapacityBufferConfig>);
    static_assert(!ConfigPolicy<MissingNumericConfig>);
    static_assert(ConfigPolicy<NumericOnlyConfig>);

    CHECK(true);
}

TEST_CASE("Concrete config composes at product-core sensor boundaries")
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    using SimConfig = navkit::app_support::NavKitConfig_t<navkit::selected_config::Config>;
    using Sensor = navkit::core::estimation::
        Sensor<0U, Model, SimConfig::SensorGraph::primary_gnss_buffer_size>;
    using GnssOnlySensor =
        navkit::core::estimation::Sensor<1U, Model, GnssOnlyBufferConfig::BufferSize>;

    static_assert(std::is_default_constructible_v<Sensor>);
    static_assert(std::is_default_constructible_v<GnssOnlySensor>);
    CHECK(SimConfig::SensorGraph::primary_gnss_buffer_size == 16U);
    CHECK(navkit::core::estimation::SensorIndexFromId_v<
              SimConfig::SensorGraph::primary_gnss_position_sensor_id,
              SimConfig::Sensors> == 0U);
    CHECK(navkit::core::estimation::SensorIndexFromId_v<
              SimConfig::SensorGraph::primary_gnss_velocity_sensor_id,
              SimConfig::Sensors> == 1U);
}

} // namespace navkit::core::config::test
