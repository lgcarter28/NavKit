// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/autopilot/FirstOrderAutopilotModel.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"
#include "navkit/sim/trajectory/FirstOrderVehicleResponseModel.hpp"
#include "navkit/sim/trajectory/TrajectoryAttitude.hpp"
#include "navkit/sim/trajectory/TrajectoryIntegration.hpp"
#include "test_main.hpp"

#include <cmath>
#include <numbers>

namespace navkit::sim::test
{

namespace
{

[[nodiscard]] AutopilotState autopilot_state_from(const TrajectoryControlState& state,
                                                  const TrajectoryEnvironment& environment)
{
    return AutopilotState{
        .q_b2i = state.q_b2i,
        .w_ib_b_radps = state.w_ib_b_radps,
        .v_eb_n_mps = environment.v_eb_n_mps,
        .C_i2n = environment.C_i2n,
    };
}

} // namespace

TEST_CASE("Guidance command filter is per-channel, bypassable, and changes nominal tau later")
{
    GuidanceCommandFilterConfig initial_config{};
    initial_config.specific_force_time_constant_b_s = core::Vec3{1.0, 0.0, 2.0};
    initial_config.bank_time_constant_s = 1.0;
    GuidanceCommandFilter filter{};
    REQUIRE(filter.initialize(core::Vec3::Zero(), 0.0, initial_config));

    GuidanceCommandFilterConfig bypass_config{};
    GuidanceCommandFilterStateEntryConfig no_entry{};
    GuidanceCommandFilterOutput first{};
    REQUIRE(filter.advance(
        core::Vec3{1.0, 2.0, 3.0}, 2.0, initial_config, no_entry, false, 1.0, first));
    CHECK(first.specific_force_filtered_ib_b_mps2.x() == doctest::Approx(1.0 - std::exp(-1.0)));
    CHECK(first.specific_force_filtered_ib_b_mps2.y() == doctest::Approx(2.0));
    CHECK(first.specific_force_filtered_ib_b_mps2.z() ==
          doctest::Approx(3.0 * (1.0 - std::exp(-0.5))));
    CHECK(first.bank_filtered_n_rad == doctest::Approx(2.0 * (1.0 - std::exp(-1.0))));

    GuidanceCommandFilterOutput second{};
    REQUIRE(filter.advance(
        core::Vec3{4.0, 5.0, 6.0}, -1.0, bypass_config, no_entry, false, 1.0, second));
    CHECK(second.specific_force_filtered_ib_b_mps2.isApprox(core::Vec3{4.0, 5.0, 6.0}));
    CHECK(second.bank_filtered_n_rad == doctest::Approx(-1.0));

    GuidanceCommandFilterConfig invalid_config{};
    invalid_config.bank_time_constant_s = -1.0;
    CHECK_FALSE(guidance_command_filter_config_is_valid(invalid_config));
}

TEST_CASE("Guidance command filter applies temporary state-entry constants without reset")
{
    GuidanceCommandFilterConfig nominal_config{};
    GuidanceCommandFilterStateEntryConfig boost_entry{};
    boost_entry.enabled = true;
    boost_entry.specific_force_time_constant_b_s = core::Vec3::Constant(1.0);
    boost_entry.bank_time_constant_s = 1.0;
    boost_entry.duration_s = 0.5;

    GuidanceCommandFilter filter{};
    REQUIRE(filter.initialize(core::Vec3::Zero(), 0.0, nominal_config));

    GuidanceCommandFilterOutput transition{};
    REQUIRE(filter.advance(
        core::Vec3::Constant(1.0), 1.0, nominal_config, boost_entry, true, 0.25, transition));
    const core::Scalar_t first_entry_value = 1.0 - std::exp(-0.25);
    CHECK(transition.specific_force_filtered_ib_b_mps2.x() == doctest::Approx(first_entry_value));
    CHECK(transition.bank_filtered_n_rad == doctest::Approx(first_entry_value));

    GuidanceCommandFilterOutput final_entry{};
    REQUIRE(filter.advance(
        core::Vec3::Constant(1.0), 1.0, nominal_config, boost_entry, false, 0.25, final_entry));
    CHECK(final_entry.specific_force_filtered_ib_b_mps2.x() ==
          doctest::Approx(1.0 - std::exp(-0.5)));

    GuidanceCommandFilterOutput nominal{};
    REQUIRE(filter.advance(
        core::Vec3::Constant(2.0), 2.0, nominal_config, boost_entry, false, 0.25, nominal));
    CHECK(nominal.specific_force_filtered_ib_b_mps2.isApprox(core::Vec3::Constant(2.0)));
    CHECK(nominal.bank_filtered_n_rad == doctest::Approx(2.0));

    GuidanceCommandFilterStateEntryConfig invalid_entry = boost_entry;
    invalid_entry.duration_s = 0.0;
    CHECK_FALSE(guidance_command_filter_state_entry_config_is_valid(invalid_entry));
}

TEST_CASE("First-order Autopilot response matches exact exponential and zero-tau limit")
{
    FirstOrderAutopilotConfig config{};
    config.controller_rate_time_constant_pqr_s = core::Vec3{1.0, 0.0, 2.0};
    config.attitude_error_gain_pqr_per_s = core::Vec3::Ones();
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitX();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    GuidanceCommand command{};
    command.bank_command_n_rad = 1.0;
    const AutopilotExecutionState execution{.active = true};
    command.specific_force_command_ib_b_mps2 = core::Vec3{1.0, -2.0, 3.0};
    VehicleCommand vehicle_command{};
    AutopilotOutput output{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          1.0,
                          vehicle_command,
                          output));
    // The deterministic command quaternion rotates one radian during this interval. Its
    // quaternion-log kinematics and the one-radian attitude error each contribute 1 rad/s.
    CHECK(output.w_feedforward_ib_b_radps.x() == doctest::Approx(1.0));
    CHECK(output.w_controller_response_ib_b_radps.x() ==
          doctest::Approx(2.0 * (1.0 - std::exp(-1.0))));
    CHECK(output.w_controller_response_ib_b_radps.y() == doctest::Approx(0.0));
    CHECK(output.w_controller_response_ib_b_radps.z() == doctest::Approx(0.0));
    CHECK(vehicle_command.w_command_ib_b_radps.isApprox(output.w_controller_response_ib_b_radps));
    CHECK(vehicle_command.specific_force_command_ib_b_mps2.isApprox(
        command.specific_force_command_ib_b_mps2));
}

TEST_CASE("Vehicle response applies limits only to final cascaded output")
{
    FirstOrderVehicleResponseConfig config{};
    config.vehicle_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.specific_force_command_time_constant_b_s = core::Vec3::Zero();
    config.specific_force_response_time_constant_b_s = core::Vec3::Zero();
    config.angular_rate_limits_enabled = true;
    config.angular_rate_limit_pqr_radps = core::Vec3::Constant(2.0);
    config.specific_force_limits_enabled = true;
    config.specific_force_limit_b_mps2 = core::Vec3::Constant(3.0);
    FirstOrderVehicleResponseModel model{config};
    TrajectoryDynamicState state{};
    REQUIRE(model.initialize(state));
    VehicleCommand control{};
    control.w_command_ib_b_radps = core::Vec3{4.0, -1.0, -5.0};
    control.specific_force_command_ib_b_mps2 = core::Vec3{1.0, 4.0, -6.0};
    TrajectoryEnvironment environment{};
    VehicleResponseOutput output{};
    REQUIRE(model.advance(control, state, environment, 0.1, output));
    CHECK(output.w_ib_b_radps.isApprox(core::Vec3{2.0, -1.0, -2.0}));
    CHECK(output.specific_force_ib_b_mps2.isApprox(core::Vec3{1.0, 3.0, -3.0}));
    CHECK(output.angular_rate_limited(0));
    CHECK_FALSE(output.angular_rate_limited(1));
    CHECK(output.angular_rate_limited(2));
}

TEST_CASE("Trajectory integration reproduces constant acceleration and body rate")
{
    core::Vec3 p_i_m = core::Vec3::Zero();
    core::Vec3 v_i_mps = core::Vec3::Zero();
    const core::Vec3 acceleration_i_mps2{2.0, 0.0, 0.0};
    REQUIRE(integrate_translation_eci(acceleration_i_mps2,
                                      acceleration_i_mps2,
                                      2.0,
                                      TranslationalIntegrationMethod::TrapezoidalPredictorCorrector,
                                      p_i_m,
                                      v_i_mps));
    CHECK(p_i_m.x() == doctest::Approx(4.0));
    CHECK(v_i_mps.x() == doctest::Approx(4.0));

    Eigen::Quaternion<core::Scalar_t> q_b2i{Eigen::Quaternion<core::Scalar_t>::Identity()};
    const core::Vec3 rate_radps{0.0, 0.0, 0.5};
    REQUIRE(integrate_attitude_eci(rate_radps, rate_radps, 2.0, q_b2i));
    CHECK(core::math::rotvec_rad_from_quaternion(q_b2i).z() == doctest::Approx(1.0));
    CHECK(q_b2i.norm() == doctest::Approx(1.0));
}

TEST_CASE("Trajectory translation supports semi-implicit Euler and rejects invalid inputs")
{
    core::Vec3 p_i_m = core::Vec3::Zero();
    core::Vec3 v_i_mps = core::Vec3::Zero();
    const core::Vec3 acceleration_i_mps2{2.0, 0.0, 0.0};
    REQUIRE(integrate_translation_eci(acceleration_i_mps2,
                                      acceleration_i_mps2,
                                      2.0,
                                      TranslationalIntegrationMethod::SemiImplicitEuler,
                                      p_i_m,
                                      v_i_mps));
    CHECK(p_i_m.x() == doctest::Approx(8.0));
    CHECK(v_i_mps.x() == doctest::Approx(4.0));

    CHECK_FALSE(integrate_translation_eci(acceleration_i_mps2,
                                          acceleration_i_mps2,
                                          0.0,
                                          TranslationalIntegrationMethod::SemiImplicitEuler,
                                          p_i_m,
                                          v_i_mps));
    CHECK_FALSE(
        integrate_translation_eci(acceleration_i_mps2,
                                  acceleration_i_mps2,
                                  1.0,
                                  // Exercise the defensive invalid-enum branch explicitly.
                                  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
                                  static_cast<TranslationalIntegrationMethod>(999),
                                  p_i_m,
                                  v_i_mps));
}

TEST_CASE("Autopilot holds initial attitude below its velocity-alignment guard")
{
    FirstOrderAutopilotConfig config{};
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Ones();
    config.velocity_alignment_speed_threshold_mps = 10.0;
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.v_eb_n_mps = core::Vec3::UnitX();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    GuidanceCommand command{};
    command.bank_command_n_rad = 1.0;
    const AutopilotExecutionState execution{.active = true};
    VehicleCommand vehicle_command{};
    AutopilotOutput output{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          1.0,
                          vehicle_command,
                          output));
    CHECK(output.q_command_b2i.isApprox(state.q_b2i));
    CHECK(output.w_command_ib_b_radps.isZero());
}

TEST_CASE("Autopilot latches an acquired velocity-aligned attitude below its speed guard")
{
    FirstOrderAutopilotConfig config{};
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Ones();
    config.velocity_alignment_speed_threshold_mps = 1.0;
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitX();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    GuidanceCommand command{};
    const AutopilotExecutionState execution{.active = true};
    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitY();
    VehicleCommand vehicle_command{};
    AutopilotOutput acquired{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          0.1,
                          vehicle_command,
                          acquired));
    CHECK((acquired.q_command_b2i * core::Vec3::UnitX()).isApprox(core::Vec3::UnitY(), 1.0e-12));

    environment.v_eb_n_mps = core::Vec3::Zero();
    command.bank_command_n_rad = 1.0;
    AutopilotOutput below_threshold{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          0.1,
                          vehicle_command,
                          below_threshold));
    CHECK(below_threshold.q_command_b2i.isApprox(acquired.q_command_b2i, 1.0e-12));
}

TEST_CASE("Autopilot velocity-aligns once the common speed guard is crossed")
{
    FirstOrderAutopilotConfig config{};
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Ones();
    config.velocity_alignment_speed_threshold_mps = 1.0;
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.C_i2n = core::Mat3::Identity();
    environment.C_n2i = core::Mat3::Identity();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    TrajectoryControlState biased_state = state;
    biased_state.q_b2i = Eigen::Quaternion<core::Scalar_t>{
        Eigen::AngleAxis<core::Scalar_t>{0.2, core::Vec3::UnitZ()}};
    REQUIRE(model.initialize(autopilot_state_from(biased_state, environment)));

    GuidanceCommand launch_pad{};
    const AutopilotExecutionState launch_pad_execution{.hold_initial_attitude = true};
    VehicleCommand vehicle_command{};
    AutopilotOutput constrained{};
    REQUIRE(model.advance(launch_pad,
                          autopilot_state_from(biased_state, environment),
                          launch_pad_execution,
                          0.1,
                          vehicle_command,
                          constrained));
    CHECK(constrained.q_command_b2i.isApprox(state.q_b2i, 1.0e-12));

    environment.v_eb_n_mps = 10.0 * core::Vec3::UnitY();
    GuidanceCommand command{};
    const AutopilotExecutionState execution{.active = true};
    AutopilotOutput aligned{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(biased_state, environment),
                          execution,
                          0.1,
                          vehicle_command,
                          aligned));
    CHECK((aligned.q_command_b2i * core::Vec3::UnitX()).isApprox(core::Vec3::UnitY(), 1.0e-12));
}

TEST_CASE("Autopilot command LPF follows the exact quaternion first-order fraction")
{
    FirstOrderAutopilotConfig config{};
    config.attitude_command_time_constant_s = 1.0;
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Zero();
    config.angular_rate_feedback_gain_pqr = core::Vec3::Zero();
    config.velocity_alignment_speed_threshold_mps = 1.0;
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.C_i2n = core::Mat3::Identity();
    environment.C_n2i = core::Mat3::Identity();
    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitX();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitY();
    GuidanceCommand command{};
    const AutopilotExecutionState execution{.active = true};
    VehicleCommand vehicle_command{};
    AutopilotOutput output{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          0.1,
                          vehicle_command,
                          output));

    const core::Scalar_t expected_yaw_rad =
        (1.0 - std::exp(-0.1)) * std::numbers::pi_v<core::Scalar_t> / 2.0;
    CHECK(core::math::rotvec_rad_from_quaternion(output.q_command_b2i).z() ==
          doctest::Approx(expected_yaw_rad));
}

TEST_CASE("Autopilot feedforward is deterministic filtered-command kinematics")
{
    FirstOrderAutopilotConfig config{};
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Zero();
    config.angular_rate_feedback_gain_pqr = core::Vec3::Zero();
    config.velocity_alignment_speed_threshold_mps = 1.0;
    FirstOrderAutopilotModel model{config};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment{};
    environment.C_i2n = core::Mat3::Identity();
    environment.C_n2i = core::Mat3::Identity();
    environment.v_eb_n_mps = 2.0 * core::Vec3::UnitX();
    REQUIRE(model.initialize(autopilot_state_from(state, environment)));

    constexpr core::Time_t dt_s = 0.1;
    constexpr core::Scalar_t yaw_step_rad = 0.1;
    environment.v_eb_n_mps = 2.0 * core::Vec3{std::cos(yaw_step_rad), std::sin(yaw_step_rad), 0.0};
    GuidanceCommand command{};
    const AutopilotExecutionState execution{.active = true};
    VehicleCommand vehicle_command{};
    AutopilotOutput output{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(state, environment),
                          execution,
                          dt_s,
                          vehicle_command,
                          output));

    CHECK(std::abs(output.w_feedforward_ib_b_radps.x()) < 1.0e-12);
    CHECK(std::abs(output.w_feedforward_ib_b_radps.y()) < 1.0e-12);
    CHECK(output.w_feedforward_ib_b_radps.z() ==
          doctest::Approx(yaw_step_rad / dt_s).epsilon(1.0e-10));
    CHECK(output.w_command_ib_b_radps.isApprox(output.w_feedforward_ib_b_radps, 1.0e-12));
}

TEST_CASE("Autopilot feedforward maps noncommuting command motion into current body axes")
{
    FirstOrderAutopilotConfig config{};
    config.attitude_command_time_constant_s = 0.2;
    config.controller_rate_time_constant_pqr_s = core::Vec3::Zero();
    config.attitude_error_gain_pqr_per_s = core::Vec3::Zero();
    config.angular_rate_feedback_gain_pqr = core::Vec3::Zero();
    config.velocity_alignment_speed_threshold_mps = 1.0;
    FirstOrderAutopilotModel model{config};

    TrajectoryControlState initial_state{};
    initial_state.q_b2i = core::math::normalized_with_positive_scalar(
        core::math::quaternion_from_rotvec_rad(core::Vec3{0.2, -0.1, 0.3}));
    TrajectoryEnvironment environment{};
    environment.C_i2n =
        Eigen::AngleAxis<core::Scalar_t>{0.4, core::Vec3::UnitZ()}.toRotationMatrix();
    environment.C_n2i = environment.C_i2n.transpose();
    environment.v_eb_n_mps = 2.0 * environment.C_i2n * (initial_state.q_b2i * core::Vec3::UnitX());
    REQUIRE(model.initialize(autopilot_state_from(initial_state, environment)));

    TrajectoryControlState current_state = initial_state;
    current_state.q_b2i = core::math::normalized_with_positive_scalar(
        initial_state.q_b2i *
        core::math::quaternion_from_rotvec_rad(core::Vec3{0.04, -0.03, 0.02}));
    environment.v_eb_n_mps = 2.0 * core::Vec3{0.8, 0.5, -0.3}.normalized();
    GuidanceCommand command{};
    command.bank_command_n_rad = 0.15;
    const AutopilotExecutionState execution{.active = true};

    constexpr core::Time_t dt_s = 0.1;
    VehicleCommand vehicle_command{};
    AutopilotOutput output{};
    REQUIRE(model.advance(command,
                          autopilot_state_from(current_state, environment),
                          execution,
                          dt_s,
                          vehicle_command,
                          output));

    const Eigen::Quaternion<core::Scalar_t> command_delta =
        core::math::normalized_with_positive_scalar(initial_state.q_b2i.conjugate() *
                                                    output.q_command_b2i);
    const core::Vec3 command_rate_previous_body_radps =
        core::math::rotvec_rad_from_quaternion(command_delta) / dt_s;
    const Eigen::Quaternion<core::Scalar_t> q_previous_command_to_current_body =
        core::math::normalized_with_positive_scalar(current_state.q_b2i.conjugate() *
                                                    initial_state.q_b2i);
    const core::Vec3 expected_feedforward_radps =
        q_previous_command_to_current_body * command_rate_previous_body_radps;

    CHECK(output.w_feedforward_ib_b_radps.isApprox(expected_feedforward_radps, 1.0e-12));
    CHECK((output.w_feedforward_ib_b_radps.array().abs() > 1.0e-4).count() >= 2);
}

TEST_CASE("IMU increment moving average uses summed angle over summed time")
{
    ImuIncrementMovingAverage<4U> average{};
    REQUIRE(average.initialize(2U));
    core::estimation::ImuIncrement first{};
    first.dt_s = 0.01;
    first.delta_theta_ib_b_rad = core::Vec3{0.01, 0.0, 0.0};
    core::estimation::ImuIncrement second{};
    second.dt_s = 0.02;
    second.delta_theta_ib_b_rad = core::Vec3{0.0, 0.02, 0.0};
    REQUIRE(average.push(first));
    REQUIRE(average.push(second));
    core::Vec3 rate_radps{};
    REQUIRE(average.average_rate(rate_radps));
    CHECK(rate_radps.x() == doctest::Approx(1.0 / 3.0));
    CHECK(rate_radps.y() == doctest::Approx(2.0 / 3.0));
}

TEST_CASE("Velocity-aligned attitude maps body forward to velocity and body down to NED down")
{
    const core::Vec3 velocity_n_mps{3.0, 4.0, 0.0};
    Eigen::Quaternion<core::Scalar_t> q_b2n{};
    REQUIRE(velocity_aligned_attitude_b2n(velocity_n_mps, core::Vec3::UnitZ(), q_b2n));
    CHECK((q_b2n * core::Vec3::UnitX()).isApprox(velocity_n_mps.normalized(), 1.0e-12));
    CHECK((q_b2n * core::Vec3::UnitZ()).isApprox(core::Vec3::UnitZ(), 1.0e-12));
    CHECK_FALSE(velocity_aligned_attitude_b2n(core::Vec3::Zero(), core::Vec3::UnitZ(), q_b2n));

    core::Vec3 retained_right_n{};
    REQUIRE(velocity_aligned_attitude_b2n(
        core::Vec3::UnitZ(), core::Vec3::UnitZ(), core::Vec3::UnitY(), q_b2n, retained_right_n));
    CHECK((q_b2n * core::Vec3::UnitX()).isApprox(core::Vec3::UnitZ(), 1.0e-12));
    CHECK((q_b2n * core::Vec3::UnitY()).isApprox(core::Vec3::UnitY(), 1.0e-12));
}

} // namespace navkit::sim::test
