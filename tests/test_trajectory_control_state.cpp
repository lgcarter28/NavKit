// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/trajectory/TrajectoryControlState.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateAccessors.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "test_main.hpp"

namespace navkit::app_support::test
{

TEST_CASE("Truth and Navigator PVA map to the same source-agnostic ECI control state")
{
    using StateDef = core::estimation::InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;

    const core::Timestamp t_epoch{};
    sim::TruthSample truth{};
    truth.p_e = core::Vec3{core::environment::Wgs84::a_m, 0.0, 0.0};
    truth.v_e = core::Vec3{10.0, 20.0, 30.0};
    truth.q_b2e = Eigen::Quaternion<core::Scalar_t>::Identity();
    truth.w_ib_b_radps = core::Vec3{0.1, 0.2, 0.3};

    sim::TrajectoryControlState truth_state{};
    REQUIRE(trajectory_control_state_from_truth(truth, t_epoch, truth_state));

    core::estimation::NominalState<StateDef> navigation_state =
        core::estimation::NominalState<StateDef>::Zero();
    core::estimation::segment<Nominal::Pos>(navigation_state) = truth.p_e;
    core::estimation::segment<Nominal::Vel>(navigation_state) = truth.v_e;
    core::estimation::set_q_b2e<StateDef>(navigation_state, truth.q_b2e);

    sim::TrajectoryControlState navigation_control_state{};
    REQUIRE(trajectory_control_state_from_navigation<StateDef>(
        truth.t, t_epoch, navigation_state, navigation_control_state));

    const core::Vec3 expected_v_i_mps =
        truth.v_e +
        core::environment::planet_rate_fixed_radps<core::environment::Wgs84>().cross(truth.p_e);
    CHECK(truth_state.p_i_m.isApprox(truth.p_e));
    CHECK(truth_state.v_i_mps.isApprox(expected_v_i_mps));
    CHECK(truth_state.w_ib_b_radps.isApprox(truth.w_ib_b_radps));
    CHECK(navigation_control_state.p_i_m.isApprox(truth_state.p_i_m));
    CHECK(navigation_control_state.v_i_mps.isApprox(truth_state.v_i_mps));
    CHECK(navigation_control_state.q_b2i.coeffs().isApprox(truth_state.q_b2i.coeffs()));
}

} // namespace navkit::app_support::test
