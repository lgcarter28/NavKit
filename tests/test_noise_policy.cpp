// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicies.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"

#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::core::estimation::test
{

using NoiseTestModel = navkit::core::models::GnssPosModel<DefaultInsStateDef>;
using NoiseTestMeasurement = Measurement<NoiseTestModel::M>;

struct ExactNoisePolicy
{
    static void update(NoiseTestModel::ObservationContext& ctx,
                       const NoiseTestMeasurement& unused_measurement)
    {
        static_cast<void>(unused_measurement);
        ctx.R_e_m2(0, 0) = 4.0;
        ctx.R_e_m2(2, 2) = 6.0;
    }
};

struct MissingUpdate
{};

struct WrongObservationContext
{
    static void update(int& unused_context, const NoiseTestMeasurement& unused_measurement)
    {
        static_cast<void>(unused_context);
        static_cast<void>(unused_measurement);
    }
};

struct WrongMeasurementSample
{
    static void update(NoiseTestModel::ObservationContext& unused_context,
                       const Measurement<1>& unused_measurement)
    {
        static_cast<void>(unused_context);
        static_cast<void>(unused_measurement);
    }
};

struct ReturningUpdate
{
    static bool update(NoiseTestModel::ObservationContext& unused_context,
                       const NoiseTestMeasurement& unused_measurement)
    {
        static_cast<void>(unused_context);
        static_cast<void>(unused_measurement);
        return true;
    }
};

TEST_CASE("NoisePolicy accepts compatible noise policies")
{
    static_assert(NoisePolicy<DefaultNoisePolicy, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(NoisePolicy<GnssFixedNoisePolicy, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(NoisePolicy<ExactNoisePolicy, NoiseTestModel, NoiseTestMeasurement>);

    CHECK(true);
}

TEST_CASE("NoisePolicy rejects incompatible noise policies")
{
    static_assert(!NoisePolicy<MissingUpdate, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<WrongObservationContext, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<WrongMeasurementSample, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<ReturningUpdate, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<ExactNoisePolicy, NoiseTestModel, Measurement<1>>);

    CHECK(true);
}

TEST_CASE("Sensor accepts constrained noise policy and preserves fixed capacity")
{
    using TestSensor = Sensor<0U, NoiseTestModel, 2, ExactNoisePolicy>;
    static_assert(std::is_default_constructible_v<TestSensor>);

    TestSensor sensor;
    NoiseTestMeasurement meas{};
    meas.z << 1.0, 2.0, 3.0;

    CHECK(sensor.push(meas));
    CHECK(sensor.push(meas));
    CHECK_FALSE(sensor.push(meas));

    sensor.update_observation_context(meas);
    CHECK(sensor.observation_context().R_e_m2(0, 0) == doctest::Approx(4.0));
    CHECK(sensor.observation_context().R_e_m2(2, 2) == doctest::Approx(6.0));
}

} // namespace navkit::core::estimation::test
