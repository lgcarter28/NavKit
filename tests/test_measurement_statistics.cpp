// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>

namespace navkit::core::estimation::test
{

namespace
{

using StateDef = InsGyroAccelBiasStateDef;
using Nominal = StateDef::Nominal;
using Error = StateDef::Error;
using Model = navkit::core::models::GnssPosModel<StateDef>;
using Sensor = navkit::core::estimation::Sensor<0U, Model, 4U>;
struct DisabledStatisticsDiagnostics
{
    // This member name is part of SensorDiagnosticsPolicy.
    // NOLINTNEXTLINE(readability-identifier-naming)
    static constexpr bool enable_measurement_statistics = false;
};
using DisabledStatisticsSensor =
    navkit::core::estimation::Sensor<1U,
                                     Model,
                                     4U,
                                     navkit::core::estimation::DefaultNoisePolicy,
                                     DisabledStatisticsDiagnostics>;
using Sensors = std::tuple<Sensor>;
using Filter =
    KalmanFilter<StateDef, DefaultInjectionPolicy<StateDef>, DefaultResetPolicy<StateDef>, Sensors>;
using DisabledStatisticsFilter = KalmanFilter<StateDef,
                                              DefaultInjectionPolicy<StateDef>,
                                              DefaultResetPolicy<StateDef>,
                                              std::tuple<DisabledStatisticsSensor>>;

struct StatisticsFixture
{
    Filter filter;
    Filter::ErrorState_t initial_error_state{Filter::ErrorState_t::Zero()};
    ErrorStateCov<StateDef> initial_covariance{ErrorStateCov<StateDef>::Identity()};
    Model::O_t measurement{Model::O_t::Zero()};
    Model::ObservationContext noise{};
    Measurement<Model::M> sensor_measurement{};

    StatisticsFixture()
    {
        Filter::State_t x = Filter::State_t::Zero();
        x.template segment<3>(Nominal::Pos::i) << 10.0, -2.0, 5.0;
        filter.set_state(x);

        initial_covariance *= 100.0;
        filter.set_covariance(initial_covariance);

        noise.R_e_m2 = navkit::core::Mat3::Identity();

        sensor_measurement.t = Timestamp{};
        sensor_measurement.z = measurement;
    }
};

Model::O_t expected_innovation()
{
    Model::O_t innovation{};
    innovation << -10.0, 2.0, -5.0;
    return innovation;
}

Model::R_t expected_measurement_covariance()
{
    return Model::R_t::Identity();
}

Model::R_t expected_innovation_covariance()
{
    return 101.0 * Model::R_t::Identity();
}

Model::H_t expected_jacobian()
{
    Model::H_t H = Model::H_t::Zero();
    H.template block<3, 3>(0, Error::Pos::i) = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    return H;
}

Model::K_t expected_gain()
{
    Model::K_t K = Model::K_t::Zero();
    K.template block<3, 3>(Error::Pos::i, 0) =
        (100.0 / 101.0) * Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    return K;
}

Scalar_t expected_nis()
{
    return expected_innovation().squaredNorm() / 101.0;
}

void check_common_statistics(const MeasurementStatistics<Sensor>& stats,
                             const bool accepted,
                             const Time_t expected_time)
{
    CHECK(stats.valid);
    CHECK(stats.accepted == accepted);
    CHECK(timestamp_seconds(stats.t) == doctest::Approx(expected_time));
    CHECK(stats.innovation.isApprox(expected_innovation(), 1.0e-12));
    CHECK(stats.measurement_covariance.isApprox(expected_measurement_covariance(), 1.0e-12));
    CHECK(stats.innovation_covariance.isApprox(expected_innovation_covariance(), 1.0e-12));
    CHECK(stats.jacobian_h.isApprox(expected_jacobian(), 1.0e-12));
    CHECK(stats.kalman_gain.isApprox(expected_gain(), 1.0e-12));
    CHECK(stats.nis == doctest::Approx(expected_nis()));
}

} // namespace

TEST_CASE("accepted measurement update records statistics and updates filter state")
{
    StatisticsFixture fixture{};
    constexpr Time_t update_time = 12.5;
    Sensor sensor{};
    REQUIRE(
        timestamp_from_seconds(update_time, TimeScale::Monotonic, fixture.sensor_measurement.t));
    sensor.observation_context() = fixture.noise;
    CHECK(sensor.push(fixture.sensor_measurement));

    CHECK_FALSE(fixture.filter.measurement_statistics_available<Sensor>());

    fixture.filter.process_sensor(sensor);

    CHECK(fixture.filter.measurement_statistics_available<Sensor>());
    check_common_statistics(fixture.filter.measurement_statistics<Sensor>(), true, update_time);

    CHECK_FALSE(fixture.filter.error_state().isApprox(fixture.initial_error_state, 1.0e-12));
    CHECK_FALSE(fixture.filter.covariance().isApprox(fixture.initial_covariance, 1.0e-12));
}

TEST_CASE("direct model update does not record sensor-keyed statistics")
{
    StatisticsFixture fixture{};
    constexpr Time_t update_time = 23.0;

    Timestamp t{};
    REQUIRE(timestamp_from_seconds(update_time, TimeScale::Monotonic, t));
    fixture.filter.observation_update<Model>(fixture.measurement, t, fixture.noise, false);

    CHECK_FALSE(fixture.filter.measurement_statistics_available<Sensor>());

    CHECK(fixture.filter.error_state().isApprox(fixture.initial_error_state, 1.0e-12));
    CHECK(fixture.filter.covariance().isApprox(fixture.initial_covariance, 1.0e-12));
}

TEST_CASE("sensor diagnostics can disable measurement statistics storage")
{
    static_assert(SensorDiagnosticsPolicy<DisabledStatisticsDiagnostics>);

    DisabledStatisticsFilter filter{};
    DisabledStatisticsSensor sensor{};
    Measurement<Model::M> measurement{};
    Model::ObservationContext noise{};

    DisabledStatisticsFilter::State_t x = DisabledStatisticsFilter::State_t::Zero();
    x.template segment<3>(Nominal::Pos::i) << 10.0, -2.0, 5.0;
    filter.set_state(x);
    filter.set_covariance(ErrorStateCov<StateDef>::Identity() * 100.0);

    noise.R_e_m2 = navkit::core::Mat3::Identity();
    sensor.observation_context() = noise;
    CHECK(sensor.push(measurement));

    filter.process_sensor(sensor);

    CHECK_FALSE(filter.measurement_statistics_available<DisabledStatisticsSensor>());
}

} // namespace navkit::core::estimation::test
