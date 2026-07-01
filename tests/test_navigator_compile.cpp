// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "test_main.hpp"

#include <tuple>

TEST_CASE("Navigator compiles and processes GNSS sensor")
{
    using StateDef = navkit::core::estimation::InsStateDef;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    using Sensor = navkit::core::estimation::Sensor<Model, 4>;
    using Filter = navkit::core::estimation::KalmanFilter<StateDef>;
    using Nav = navkit::core::estimation::Navigator<Filter, std::tuple<Sensor>>;

    Nav nav;
    Sensor::Measurement_t meas;
    meas.time = 0.0;
    meas.z << 1.0, 2.0, 3.0;
    CHECK(nav.template sensor<0>().push(meas));
    nav.process_measurements();
    CHECK(nav.filter().state()(StateDef::Pos::i) != doctest::Approx(0.0));
}
