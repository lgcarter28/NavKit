// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

TEST_CASE("GNSS position update moves state toward measurement")
{
    using StateDef = navkit::core::estimation::DefaultInsStateDef;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    navkit::core::estimation::KalmanFilter<StateDef> kf;

    decltype(kf)::State_t x = decltype(kf)::State_t::Zero();
    x.template segment<3>(StateDef::Pos::i) << 10.0, 0.0, 0.0;
    kf.set_state(x);

    navkit::core::estimation::StateCov<StateDef> P =
        navkit::core::estimation::StateCov<StateDef>::Identity();
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

    CHECK(kf.state()(StateDef::Pos::i) < 10.0);
}
