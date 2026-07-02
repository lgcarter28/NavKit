// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "examples/MinimalConfig.hpp"
#include "navkit/SelectedConfig.hpp"
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
    using Numeric = navkit::config::examples::MinimalNumericConfig;
};

struct MissingNumericConfig
{
    using GnssBuffer = navkit::config::examples::MinimalGnssBufferConfig;
};

struct NumericOnlyConfig
{
    using Numeric = navkit::config::examples::MinimalNumericConfig;
};

TEST_CASE("Concrete config slices satisfy narrow configuration concepts")
{
    using ExampleConfig = navkit::config::examples::MinimalConfig;
    using SimConfig = navkit::selected_config::Config;

    static_assert(NumericConfigPolicy<navkit::config::examples::MinimalNumericConfig>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<
                  navkit::config::examples::MinimalGnssBufferConfig>);
    static_assert(navkit::core::estimation::MeasurementStatisticsConfigPolicy<
                  SimConfig::MeasurementStatistics>);
    static_assert(navkit::core::estimation::BufferConfigPolicy<SimConfig::GnssBuffer>);
    static_assert(ConfigPolicy<ExampleConfig>);
    static_assert(ConfigPolicy<SimConfig>);
    static_assert(ConfigPolicy<MinimalConfig>);

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
    using SimConfig = navkit::selected_config::Config;
    using Sensor = navkit::core::estimation::Sensor<Model, SimConfig::GnssBuffer::BufferSize>;
    using GnssOnlySensor =
        navkit::core::estimation::Sensor<Model, GnssOnlyBufferConfig::BufferSize>;

    static_assert(std::is_default_constructible_v<Sensor>);
    static_assert(std::is_default_constructible_v<GnssOnlySensor>);
    CHECK(SimConfig::GnssBuffer::BufferSize == 16U);
}

} // namespace navkit::core::config::test
