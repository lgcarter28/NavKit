// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/MinimalConfig.hpp"
#include "navkit/SelectedConfig.hpp"
#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/ConfigTraits.hpp"
#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"

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

struct MissingStatisticsFlagConfig
{};

struct MinimalConfig
{
    using Numeric = navkit::config::navkit::MinimalNumericConfig;
};

struct MissingNumericConfig
{
    using GnssBuffer = navkit::config::navkit::MinimalGnssBufferConfig;
};

struct NumericOnlyConfig
{
    using Numeric = navkit::config::navkit::MinimalNumericConfig;
};

TEST_CASE("Concrete config slices satisfy narrow configuration concepts")
{
    using ExampleConfig = navkit::config::navkit::MinimalConfig;
    using SimConfig = navkit::app_support::NavKitConfig_t<navkit::selected_config::Config>;

    static_assert(NumericConfigPolicy<navkit::config::navkit::MinimalNumericConfig>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<
                  navkit::config::navkit::MinimalGnssBufferConfig>);
    static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
                  SimConfig::MeasurementStatistics>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<SimConfig::GnssBuffer>);
    static_assert(ConfigPolicy<ExampleConfig>);
    static_assert(ConfigPolicy<SimConfig>);
    static_assert(ConfigPolicy<MinimalConfig>);
    static_assert(navkit::api::config::SensorGraphConfigPolicy<SimConfig>);
    static_assert(navkit::api::config::NavKitProductConfigPolicy<SimConfig>);
    static_assert(navkit::api::config::sensor_ids_unique_v<SimConfig::Sensors>);
    static_assert(
        std::is_same_v<
            navkit::api::config::SensorFromId_t<SimConfig::PrimaryGnssSensorId, SimConfig::Sensors>,
            SimConfig::PrimaryGnssSensor>);

    static_assert(std::is_same_v<ExampleConfig::Numeric::Scalar_t, navkit::core::Scalar_t>);
    static_assert(std::is_same_v<ExampleConfig::Numeric::Time_t, navkit::core::Time_t>);

    CHECK(ExampleConfig::GnssBuffer::BufferSize > 0);
    CHECK(SimConfig::GnssBuffer::BufferSize > 0);
}

TEST_CASE("Narrow config concepts reject only their own missing capabilities")
{
    static_assert(!NumericConfigPolicy<MissingScalarConfig>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<GnssOnlyBufferConfig>);
    static_assert(!navkit::core::estimation::BufferConfigPolicy<MissingBufferSizeConfig>);
    static_assert(!navkit::core::estimation::BufferConfigPolicy<ZeroCapacityBufferConfig>);
    static_assert(
        !navkit::core::estimation::MeasurementStatisticsConfigPolicy<MissingStatisticsFlagConfig>);
    static_assert(!ConfigPolicy<MissingNumericConfig>);
    static_assert(ConfigPolicy<NumericOnlyConfig>);

    CHECK(true);
}

TEST_CASE("Concrete config composes at product-core sensor boundaries")
{
    using StateDef = navkit::core::estimation::InsStateDef;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    using SimConfig = navkit::app_support::NavKitConfig_t<navkit::selected_config::Config>;
    using Sensor = navkit::core::estimation::Sensor<0U, Model, SimConfig::GnssBuffer::BufferSize>;
    using GnssOnlySensor =
        navkit::core::estimation::Sensor<1U, Model, GnssOnlyBufferConfig::BufferSize>;

    static_assert(std::is_default_constructible_v<Sensor>);
    static_assert(std::is_default_constructible_v<GnssOnlySensor>);
    CHECK(SimConfig::GnssBuffer::BufferSize == 16U);
    CHECK(navkit::api::config::SensorIndexFromId_v<SimConfig::PrimaryGnssSensorId,
                                                   SimConfig::Sensors> == 0U);
}

} // namespace navkit::core::config::test
