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

struct MeasurementModelPolicyTestNominalStateDef
{
    using Scalar_t = double;
    static constexpr int N = 6;
};

struct MeasurementModelPolicyTestErrorStateDef
{
    using Scalar_t = double;
    static constexpr int N = 6;
};

struct MeasurementModelPolicyTestStateDef
{
    using Nominal = MeasurementModelPolicyTestNominalStateDef;
    using Error = MeasurementModelPolicyTestErrorStateDef;
};

struct ValidMeasurement
{
    static constexpr int M = 2;
    using State_t = NominalState<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestErrorStateDef::N, M>;

    struct ObservationContext
    {
        Scalar_t sigma{1.0};
    };

    static O_t obs(const State_t& unused_state, const ObservationContext& unused_context)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_context);
        return O_t::Zero();
    }

    static H_t compute_h(const State_t& unused_state, const ObservationContext& unused_context)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_context);
        return H_t::Zero();
    }

    static R_t compute_r(const ObservationContext& ctx)
    {
        return (ctx.sigma * ctx.sigma) * R_t::Identity();
    }
};

struct LocalInjection
{
    static void apply(NominalState<MeasurementModelPolicyTestStateDef>& x,
                      const ErrorState<MeasurementModelPolicyTestStateDef>& dx)
    {
        x -= dx;
    }
};

struct LocalReset
{
    static void
    reset_covariance(NominalState<MeasurementModelPolicyTestStateDef>& unused_state,
                     ErrorState<MeasurementModelPolicyTestStateDef>& unused_dx,
                     ErrorStateCov<MeasurementModelPolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }

    static void reset_dx(ErrorState<MeasurementModelPolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingObservationContext
{
    static constexpr int M = 2;
    using State_t = NominalState<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestErrorStateDef::N, M>;
};

struct MissingGainType
{
    static constexpr int M = 2;
    using State_t = NominalState<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;

    struct ObservationContext
    {};

    static O_t obs(const State_t&, const ObservationContext&);
    static H_t compute_h(const State_t&, const ObservationContext&);
    static R_t compute_r(const ObservationContext&);
};

struct BadObservationReturn
{
    static constexpr int M = 2;
    using State_t = NominalState<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementModelPolicyTestErrorStateDef::N, M>;

    struct ObservationContext
    {};

    static R_t obs(const State_t&, const ObservationContext&);
    static H_t compute_h(const State_t&, const ObservationContext&);
    static R_t compute_r(const ObservationContext&);
};

struct WrongGainDimensions
{
    static constexpr int M = 2;
    using State_t = NominalState<MeasurementModelPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, M, MeasurementModelPolicyTestErrorStateDef::N>;

    struct ObservationContext
    {};

    static O_t obs(const State_t&, const ObservationContext&);
    static H_t compute_h(const State_t&, const ObservationContext&);
    static R_t compute_r(const ObservationContext&);
};

TEST_CASE("MeasurementModelPolicy accepts compatible measurement models")
{
    static_assert(MeasurementModelPolicy<ValidMeasurement, MeasurementModelPolicyTestStateDef>);
    static_assert(MeasurementModelPolicy<navkit::core::models::GnssPosModel<DefaultInsStateDef>,
                                         DefaultInsStateDef>);
    static_assert(MeasurementModelPolicy<navkit::core::models::GnssVelModel<DefaultInsStateDef>,
                                         DefaultInsStateDef>);
    static_assert(MeasurementModelPolicy<navkit::core::models::BaroAltModel<DefaultInsStateDef>,
                                         DefaultInsStateDef>);

    CHECK(true);
}

TEST_CASE("MeasurementModelPolicy rejects incomplete or incompatible models")
{
    static_assert(
        !MeasurementModelPolicy<MissingObservationContext, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<MissingGainType, MeasurementModelPolicyTestStateDef>);
    static_assert(
        !MeasurementModelPolicy<BadObservationReturn, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<WrongGainDimensions, MeasurementModelPolicyTestStateDef>);
    static_assert(!MeasurementModelPolicy<navkit::core::models::GnssPosModel<DefaultInsStateDef>,
                                          MeasurementModelPolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("KalmanFilter accepts constrained measurement models at observation boundary")
{
    using Filter = KalmanFilter<MeasurementModelPolicyTestStateDef, LocalInjection, LocalReset>;
    static_assert(std::is_default_constructible_v<Filter>);

    Filter filter;
    ValidMeasurement::ObservationContext ctx{};
    filter.observation_update<ValidMeasurement>(ValidMeasurement::O_t::Zero(), ctx);

    CHECK(filter.error_state().isZero());
}

} // namespace navkit::core::estimation::test
