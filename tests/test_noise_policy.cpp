// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/Measurement.hpp"
#include "navkit/core/NoisePolicy.hpp"
#include "navkit/core/Sensor.hpp"
#include "navkit/core/StateDefs.hpp"
#include "navkit/core/policies/NoisePolicies.hpp"
#include "navkit/models/GnssPosModel.hpp"

#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::test
{

using NoiseTestModel = GnssPosModel<InsStateDef>;
using NoiseTestMeasurement = Measurement<NoiseTestModel::M>;

struct ExactNoisePolicy
{
    static void update(NoiseTestModel::NoiseContext& ctx, const NoiseTestMeasurement&)
    {
        ctx.sigma_h = 4.0;
        ctx.sigma_v = 6.0;
    }
};

struct MissingUpdate
{};

struct WrongNoiseContext
{
    static void update(int&, const NoiseTestMeasurement&) {}
};

struct WrongMeasurementSample
{
    static void update(NoiseTestModel::NoiseContext&, const Measurement<1>&) {}
};

struct ReturningUpdate
{
    static bool update(NoiseTestModel::NoiseContext&, const NoiseTestMeasurement&)
    {
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
    static_assert(!NoisePolicy<WrongNoiseContext, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<WrongMeasurementSample, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<ReturningUpdate, NoiseTestModel, NoiseTestMeasurement>);
    static_assert(!NoisePolicy<ExactNoisePolicy, NoiseTestModel, Measurement<1>>);

    CHECK(true);
}

TEST_CASE("Sensor accepts constrained noise policy and preserves fixed capacity")
{
    using TestSensor = Sensor<NoiseTestModel, 2, ExactNoisePolicy>;
    static_assert(std::is_default_constructible_v<TestSensor>);

    TestSensor sensor;
    NoiseTestMeasurement meas{};
    meas.z << 1.0, 2.0, 3.0;

    CHECK(sensor.push(meas));
    CHECK(sensor.push(meas));
    CHECK_FALSE(sensor.push(meas));

    sensor.update_noise_context(meas);
    CHECK(sensor.noise_context().sigma_h == doctest::Approx(4.0));
    CHECK(sensor.noise_context().sigma_v == doctest::Approx(6.0));
}

} // namespace navkit::test
