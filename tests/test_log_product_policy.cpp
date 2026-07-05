// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/io/LogProductPolicy.hpp"
#include "navkit/io/RunLogProducts.hpp"
#include "navkit/sim/TruthSample.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <tuple>

namespace navkit::io::test
{

namespace
{

using StateDef = navkit::core::estimation::InsStateDef;
using Model = navkit::core::models::GnssPosModel<StateDef>;
using Sensor = navkit::core::estimation::Sensor<0U, Model, 4U>;
using Filter = navkit::core::estimation::KalmanFilter<
    StateDef,
    navkit::core::estimation::DefaultInjectionPolicy<StateDef>,
    navkit::core::estimation::DefaultResetPolicy<StateDef>,
    std::tuple<navkit::core::estimation::MeasurementStatistics<Sensor>>>;
using GnssMeasurement = navkit::core::estimation::Measurement<3>;
using Statistics = navkit::core::estimation::MeasurementStatistics<Sensor>;

struct MissingOpen
{
    void log(const GnssMeasurement&) {}
    void flush() {}
    nlohmann::json metadata()
    {
        return {};
    }
    static nlohmann::json manifest_entry()
    {
        return {};
    }
};

struct MissingPayloadLog
{
    void open(const std::filesystem::path&) {}
    void log() {}
    void flush() {}
    nlohmann::json metadata()
    {
        return {};
    }
    static nlohmann::json manifest_entry()
    {
        return {};
    }
};

struct MissingManifest
{
    void open(const std::filesystem::path&) {}
    void log(const GnssMeasurement&) {}
    void flush() {}
    nlohmann::json metadata()
    {
        return {};
    }
};

} // namespace

TEST_CASE("log product policies describe concrete payload boundaries")
{
    static_assert(LogProductPolicy<TruthLogProduct, navkit::sim::TruthSample>);
    static_assert(LogProductPolicy<GnssPositionLogProduct, GnssMeasurement>);
    static_assert(LogProductPolicy<NavEstimateLogProduct, NavEstimateLogPayload<StateDef, Filter>>);
    static_assert(LogProductPolicy<GnssPositionUpdateLogProduct,
                                   MeasurementStatisticsLogPayload<Statistics>>);

    static_assert(!LogProductPolicy<MissingOpen, GnssMeasurement>);
    static_assert(!LogProductPolicy<MissingPayloadLog, GnssMeasurement>);
    static_assert(!LogProductPolicy<MissingManifest, GnssMeasurement>);

    CHECK(true);
}

} // namespace navkit::io::test
