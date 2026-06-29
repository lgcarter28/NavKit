// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/Navigator.hpp"
#include "navkit/core/Sensor.hpp"
#include "navkit/core/StateDefs.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>

TEST_CASE("Navigator compiles and processes GNSS sensor")
{
    using StateDef = navkit::InsStateDef;
    using Model = navkit::GnssPosModel<StateDef>;
    using Sensor = navkit::Sensor<Model, 4>;
    using Filter = navkit::KalmanFilter<StateDef>;
    using Nav = navkit::Navigator<Filter, std::tuple<Sensor>>;

    Nav nav;
    Sensor::Measurement_t meas;
    meas.time = 0.0;
    meas.z << 1.0, 2.0, 3.0;
    CHECK(nav.template sensor<0>().push(meas));
    nav.process_measurements();
    CHECK(nav.filter().state()(StateDef::Pos::i) != doctest::Approx(0.0));
}
