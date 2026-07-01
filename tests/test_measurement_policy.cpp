// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/KalmanFilter.hpp"
#include "navkit/core/MeasurementPolicy.hpp"
#include "navkit/core/StateDefs.hpp"
#include "navkit/models/BaroAltModel.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "navkit/models/GnssVelModel.hpp"

#include <doctest/doctest.h>
#include <tuple>
#include <type_traits>

namespace navkit::test
{

struct MeasurementPolicyTestStateDef
{
    using Scalar_t = double;
    static constexpr int N = 6;
};

struct ValidMeasurement
{
    static constexpr int M = 2;
    using State_t = State<MeasurementPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementPolicyTestStateDef::N, M>;

    struct NoiseContext
    {
        Scalar_t sigma{1.0};
    };

    static O_t obs(const State_t&)
    {
        return O_t::Zero();
    }

    static H_t compute_h(const State_t&)
    {
        return H_t::Zero();
    }

    static R_t compute_r(const NoiseContext& ctx)
    {
        return (ctx.sigma * ctx.sigma) * R_t::Identity();
    }
};

struct LocalInjection
{
    static void apply(State<MeasurementPolicyTestStateDef>& x,
                      const State<MeasurementPolicyTestStateDef>& dx)
    {
        x -= dx;
    }
};

struct LocalReset
{
    static void reset_covariance(State<MeasurementPolicyTestStateDef>&,
                                 const State<MeasurementPolicyTestStateDef>&,
                                 StateCov<MeasurementPolicyTestStateDef>&)
    {}

    static void reset_dx(State<MeasurementPolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingNoiseContext
{
    static constexpr int M = 2;
    using State_t = State<MeasurementPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementPolicyTestStateDef::N, M>;
};

struct MissingGainType
{
    static constexpr int M = 2;
    using State_t = State<MeasurementPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;

    struct NoiseContext
    {};

    static O_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

struct BadObservationReturn
{
    static constexpr int M = 2;
    using State_t = State<MeasurementPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, MeasurementPolicyTestStateDef::N, M>;

    struct NoiseContext
    {};

    static R_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

struct WrongGainDimensions
{
    static constexpr int M = 2;
    using State_t = State<MeasurementPolicyTestStateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using H_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;
    using K_t = Eigen::Matrix<Scalar_t, M, MeasurementPolicyTestStateDef::N>;

    struct NoiseContext
    {};

    static O_t obs(const State_t&);
    static H_t compute_h(const State_t&);
    static R_t compute_r(const NoiseContext&);
};

TEST_CASE("MeasurementPolicy accepts compatible measurement models")
{
    static_assert(MeasurementPolicy<ValidMeasurement, MeasurementPolicyTestStateDef>);
    static_assert(MeasurementPolicy<GnssPosModel<InsStateDef>, InsStateDef>);
    static_assert(MeasurementPolicy<GnssVelModel<InsStateDef>, InsStateDef>);
    static_assert(MeasurementPolicy<BaroAltModel<InsStateDef>, InsStateDef>);

    CHECK(true);
}

TEST_CASE("MeasurementPolicy rejects incomplete or incompatible models")
{
    static_assert(!MeasurementPolicy<MissingNoiseContext, MeasurementPolicyTestStateDef>);
    static_assert(!MeasurementPolicy<MissingGainType, MeasurementPolicyTestStateDef>);
    static_assert(!MeasurementPolicy<BadObservationReturn, MeasurementPolicyTestStateDef>);
    static_assert(!MeasurementPolicy<WrongGainDimensions, MeasurementPolicyTestStateDef>);
    static_assert(!MeasurementPolicy<GnssPosModel<InsStateDef>, MeasurementPolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("KalmanFilter accepts constrained measurement models at observation boundary")
{
    using Filter = KalmanFilter<MeasurementPolicyTestStateDef,
                                LocalInjection,
                                LocalReset,
                                std::tuple<ValidMeasurement>>;
    static_assert(std::is_default_constructible_v<Filter>);

    Filter filter;
    ValidMeasurement::NoiseContext ctx{};
    filter.observation_update<ValidMeasurement>(ValidMeasurement::O_t::Zero(), ctx);

    CHECK(filter.has_measurement_statistics<ValidMeasurement>());
}

} // namespace navkit::test
