// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"
#include "test_main.hpp"

TEST_CASE("GNSS position update moves state toward measurement")
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using Nominal = StateDef::Nominal;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    navkit::core::estimation::KalmanFilter<StateDef> kf;

    decltype(kf)::State_t x = decltype(kf)::State_t::Zero();
    x.template segment<3>(Nominal::Pos::i) << 10.0, 0.0, 0.0;
    kf.set_state(x);

    navkit::core::estimation::ErrorStateCov<StateDef> P =
        navkit::core::estimation::ErrorStateCov<StateDef>::Identity();
    P *= 100.0;
    kf.set_covariance(P);

    Model::O_t z;
    z << 0.0, 0.0, 0.0;
    Model::NoiseContext ctx;
    ctx.sigma_h = 1.0;
    ctx.sigma_v = 1.0;

    kf.observation_update<Model>(z, ctx);
    kf.inject();
    kf.reset();

    CHECK(kf.state()(Nominal::Pos::i) < 10.0);
}

TEST_CASE("GNSS position Jacobian follows truth-minus-estimate error convention")
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssPosModel<StateDef>;

    const Model::State_t x = Model::State_t::Zero();
    const Model::H_t H = Model::compute_h(x);

    CHECK(H.template block<3, 3>(0, Error::Pos::i)
              .isApprox(Eigen::Matrix<navkit::core::Scalar_t, 3, 3>::Identity()));
}

TEST_CASE("GNSS velocity Jacobian follows truth-minus-estimate error convention")
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssVelModel<StateDef>;

    const Model::State_t x = Model::State_t::Zero();
    const Model::H_t H = Model::compute_h(x);

    CHECK(H.template block<3, 3>(0, Error::Vel::i)
              .isApprox(Eigen::Matrix<navkit::core::Scalar_t, 3, 3>::Identity()));
}
