// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

namespace navkit::core::estimation::test
{

namespace
{

using SensorTestModel = navkit::core::models::GnssPosModel<InsStateDef>;
using SensorTestMeasurement = Measurement<SensorTestModel::M>;

struct MeasurementDrivenNoisePolicy
{
    static void update(SensorTestModel::NoiseContext& context,
                       const SensorTestMeasurement& measurement)
    {
        context.sigma_h = measurement.z(0);
        context.sigma_v = measurement.z(1);
    }
};

SensorTestMeasurement make_measurement(const double time_s, const double x_m)
{
    SensorTestMeasurement measurement{};
    measurement.time = time_s;
    measurement.z << x_m, x_m + 1.0, x_m + 2.0;
    return measurement;
}

} // namespace

TEST_CASE("Sensor preserves FIFO measurement order")
{
    Sensor<SensorTestModel, 2> sensor;
    CHECK_FALSE(sensor.has_measurement());

    CHECK(sensor.push(make_measurement(1.0, 10.0)));
    CHECK(sensor.push(make_measurement(2.0, 20.0)));
    CHECK_FALSE(sensor.push(make_measurement(3.0, 30.0)));

    SensorTestMeasurement out{};
    CHECK(sensor.has_measurement());
    CHECK(sensor.pop(out));
    CHECK(out.time == doctest::Approx(1.0));
    CHECK(out.z(0) == doctest::Approx(10.0));

    CHECK(sensor.pop(out));
    CHECK(out.time == doctest::Approx(2.0));
    CHECK(out.z(0) == doctest::Approx(20.0));

    CHECK_FALSE(sensor.pop(out));
    CHECK_FALSE(sensor.has_measurement());
}

TEST_CASE("Sensor noise policy can update context from the measurement sample")
{
    Sensor<SensorTestModel, 1, MeasurementDrivenNoisePolicy> sensor;
    auto measurement = make_measurement(1.0, 10.0);

    sensor.update_noise_context(measurement);

    CHECK(sensor.noise_context().sigma_h == doctest::Approx(10.0));
    CHECK(sensor.noise_context().sigma_v == doctest::Approx(11.0));
}

} // namespace navkit::core::estimation::test
