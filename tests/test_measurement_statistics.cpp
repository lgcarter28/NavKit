// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/filter/KalmanFilter.hpp"
#include "navkit/core/state/StateDefs.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>

namespace navkit::test
{

namespace
{

using StateDef = InsStateDef;
using Model = GnssPosModel<StateDef>;
using Filter = KalmanFilter<StateDef,
                            DefaultInjectionPolicy<StateDef>,
                            DefaultResetPolicy<StateDef>,
                            std::tuple<Model>>;

struct StatisticsFixture
{
    Filter filter{};
    State<StateDef> initial_error_state{State<StateDef>::Zero()};
    StateCov<StateDef> initial_covariance{StateCov<StateDef>::Identity()};
    Model::O_t measurement{Model::O_t::Zero()};
    Model::NoiseContext noise{};

    StatisticsFixture()
    {
        State<StateDef> x = State<StateDef>::Zero();
        x.template segment<3>(StateDef::Pos::i) << 10.0, -2.0, 5.0;
        filter.set_state(x);

        initial_covariance *= 100.0;
        filter.set_covariance(initial_covariance);

        noise.sigma_h = 1.0;
        noise.sigma_v = 1.0;
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
    H.template block<3, 3>(0, StateDef::Pos::i) = -Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    return H;
}

Model::K_t expected_gain()
{
    Model::K_t K = Model::K_t::Zero();
    K.template block<3, 3>(StateDef::Pos::i, 0) =
        (-100.0 / 101.0) * Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    return K;
}

Scalar_t expected_nis()
{
    return expected_innovation().squaredNorm() / 101.0;
}

void check_common_statistics(const MeasurementStatistics<Model>& stats,
                             const bool accepted,
                             const Time_t expected_time)
{
    CHECK(stats.valid);
    CHECK(stats.accepted == accepted);
    CHECK(stats.time == doctest::Approx(expected_time));
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

    CHECK_FALSE(fixture.filter.has_measurement_statistics<Model>());

    fixture.filter.observation_update<Model>(fixture.measurement, update_time, fixture.noise, true);

    CHECK(fixture.filter.has_measurement_statistics<Model>());
    check_common_statistics(fixture.filter.measurement_statistics<Model>(), true, update_time);

    CHECK_FALSE(fixture.filter.error_state().isApprox(fixture.initial_error_state, 1.0e-12));
    CHECK_FALSE(fixture.filter.covariance().isApprox(fixture.initial_covariance, 1.0e-12));
}

TEST_CASE("rejected measurement update records statistics without updating filter state")
{
    StatisticsFixture fixture{};
    constexpr Time_t update_time = 23.0;

    fixture.filter.observation_update<Model>(
        fixture.measurement, update_time, fixture.noise, false);

    CHECK(fixture.filter.has_measurement_statistics<Model>());
    check_common_statistics(fixture.filter.measurement_statistics<Model>(), false, update_time);

    CHECK(fixture.filter.error_state().isApprox(fixture.initial_error_state, 1.0e-12));
    CHECK(fixture.filter.covariance().isApprox(fixture.initial_covariance, 1.0e-12));
}

} // namespace navkit::test
