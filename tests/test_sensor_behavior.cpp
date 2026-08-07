// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

namespace navkit::core::estimation::test
{

namespace
{

using SensorTestModel = navkit::core::models::GnssPosModel<InsGyroAccelBiasStateDef>;
using SensorTestMeasurement = Measurement<SensorTestModel::M>;

struct WrongInnovationGateSensor : Sensor<0U, SensorTestModel, 1U>
{
    using InnovationGate_t = InnovationGate<2>;
};

static_assert(SensorPolicy<Sensor<0U, SensorTestModel, 1U>>);
static_assert(!SensorPolicy<WrongInnovationGateSensor>);

struct MeasurementDrivenNoisePolicy
{
    static void update(SensorTestModel::ObservationContext& context,
                       const SensorTestMeasurement& measurement)
    {
        context.R_e_m2(0, 0) = measurement.z(0);
        context.R_e_m2(2, 2) = measurement.z(1);
    }
};

SensorTestMeasurement make_measurement(const double time_s, const double x_m)
{
    SensorTestMeasurement measurement{};
    const bool timestamp_valid =
        timestamp_from_seconds(time_s, TimeScale::Monotonic, measurement.t);
    if (!timestamp_valid) {
        return {};
    }
    measurement.z << x_m, x_m + 1.0, x_m + 2.0;
    return measurement;
}

} // namespace

TEST_CASE("Sensor preserves FIFO measurement order")
{
    Sensor<0U, SensorTestModel, 2> sensor;
    CHECK_FALSE(sensor.has_measurement());

    CHECK(sensor.push(make_measurement(1.0, 10.0)));
    CHECK(sensor.push(make_measurement(2.0, 20.0)));
    CHECK_FALSE(sensor.push(make_measurement(3.0, 30.0)));

    SensorTestMeasurement out{};
    CHECK(sensor.has_measurement());
    CHECK(sensor.pop(out));
    CHECK(timestamp_seconds(out.t) == doctest::Approx(1.0));
    CHECK(out.z(0) == doctest::Approx(10.0));

    CHECK(sensor.pop(out));
    CHECK(timestamp_seconds(out.t) == doctest::Approx(2.0));
    CHECK(out.z(0) == doctest::Approx(20.0));

    CHECK_FALSE(sensor.pop(out));
    CHECK_FALSE(sensor.has_measurement());
}

TEST_CASE("Sensor noise policy can update context from the measurement sample")
{
    Sensor<0U, SensorTestModel, 1, MeasurementDrivenNoisePolicy> sensor;
    SensorTestMeasurement measurement = make_measurement(1.0, 10.0);

    sensor.update_observation_context(measurement);

    CHECK(sensor.observation_context().R_e_m2(0, 0) == doctest::Approx(10.0));
    CHECK(sensor.observation_context().R_e_m2(2, 2) == doctest::Approx(11.0));
}

} // namespace navkit::core::estimation::test
