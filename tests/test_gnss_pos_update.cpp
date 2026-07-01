// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/filter/KalmanFilter.hpp"
#include "navkit/core/state/StateDefs.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "test_main.hpp"

TEST_CASE("GNSS position update moves state toward measurement")
{
    using StateDef = navkit::InsStateDef;
    using Model = navkit::GnssPosModel<StateDef>;
    navkit::KalmanFilter<StateDef> kf;

    navkit::State<StateDef> x = navkit::State<StateDef>::Zero();
    x.template segment<3>(StateDef::Pos::i) << 10.0, 0.0, 0.0;
    kf.set_state(x);

    navkit::StateCov<StateDef> P = navkit::StateCov<StateDef>::Identity();
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
