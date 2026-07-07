// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/measurement/MeasurementModelPolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/BaroAltModel.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"

#include <doctest/doctest.h>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation::test
{

struct MeasurementModelPolicyTestStateDef
{
    using Scalar_t = double;
    static constexpr int N = 6;
};

struct ValidMeasurement
{
    static constexpr int M = 2;
    using State_t = State<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestStateDef::N, M>;

    struct NoiseContext
    {
        Scalar_t sigma{1.0};
    };

    static O_t obs(const State_t& unused_state)
    {
        static_cast<void>(unused_state);
        return O_t::Zero();
    }

    static H_t compute_h(const State_t& unused_state)
    {
        static_cast<void>(unused_state);
        return H_t::Zero();
    }

    static R_t compute_r(const NoiseContext& ctx)
    {
        return (ctx.sigma * ctx.sigma) * R_t::Identity();
    }
};

struct LocalInjection
{
    static void apply(State<MeasurementModelPolicyTestStateDef>& x,
                      const State<MeasurementModelPolicyTestStateDef>& dx)
    {
        x -= dx;
    }
};

struct LocalReset
{
    static void reset_covariance(State<MeasurementModelPolicyTestStateDef>& unused_state,
                                 const State<MeasurementModelPolicyTestStateDef>& unused_dx,
                                 StateCov<MeasurementModelPolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }

    static void reset_dx(State<MeasurementModelPolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingNoiseContext
{
    static constexpr int M = 2;
    using State_t = State<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestStateDef::N, M>;
};

struct MissingGainType
{
    static constexpr int M = 2;
    using State_t = State<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;

    struct NoiseContext
    {};

    static O_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

struct BadObservationReturn
{
    static constexpr int M = 2;
    using State_t = State<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestStateDef::N, M>;

    struct NoiseContext
    {};

    static R_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

struct WrongGainDimensions
{
    static constexpr int M = 2;
    using State_t = State<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestStateDef::N>;

    struct NoiseContext
    {};

    static O_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

TEST_CASE("MeasurementModelPolicy accepts compatible measurement models")
{
    static_assert(MeasurementModelPolicy<ValidMeasurement, MeasurementModelPolicyTestStateDef>);
    static_assert(
        MeasurementModelPolicy<navkit::core::models::GnssPosModel<InsStateDef>, InsStateDef>);
    static_assert(
        MeasurementModelPolicy<navkit::core::models::GnssVelModel<InsStateDef>, InsStateDef>);
    static_assert(
        MeasurementModelPolicy<navkit::core::models::BaroAltModel<InsStateDef>, InsStateDef>);

    CHECK(true);
}

TEST_CASE("MeasurementModelPolicy rejects incomplete or incompatible models")
{
    static_assert(!MeasurementModelPolicy<MissingNoiseContext, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<MissingGainType, MeasurementModelPolicyTestStateDef>);
    static_assert(
        !MeasurementModelPolicy<BadObservationReturn, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<WrongGainDimensions, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<navkit::core::models::GnssPosModel<InsStateDef>,
                                          MeasurementModelPolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("KalmanFilter accepts constrained measurement models at observation boundary")
{
    using Filter = KalmanFilter<MeasurementModelPolicyTestStateDef, LocalInjection, LocalReset>;
    static_assert(std::is_default_constructible_v<Filter>);

    Filter filter;
    ValidMeasurement::NoiseContext ctx{};
    filter.observation_update<ValidMeasurement>(ValidMeasurement::O_t::Zero(), ctx);

    CHECK(filter.error_state().isZero());
}

} // namespace navkit::core::estimation::test
