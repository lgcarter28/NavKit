// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"
#include "navkit/sim/sensors/GnssSimulator.hpp"
#include "test_main.hpp"

#include <cmath>
#include <tuple>

namespace
{

template<typename Model>
Model::State_t nominal_test_state()
{
    using Nominal = Model::Nominal;

    typename Model::State_t x = Model::State_t::Zero();
    x.template segment<3>(Nominal::Pos::i) << 10.0, 20.0, 30.0;
    x.template segment<3>(Nominal::Vel::i) << 1.0, 2.0, 3.0;
    const Eigen::Quaternion<navkit::core::Scalar_t> q_b2e =
        navkit::core::math::quaternion_from_rpy_rad(navkit::core::Vec3{0.15, -0.08, 0.21});
    x.template segment<4>(Nominal::AttQuat::i) << q_b2e.w(), q_b2e.x(), q_b2e.y(), q_b2e.z();
    return x;
}

template<typename Model>
Model::State_t attitude_perturbed_state(const typename Model::State_t& x,
                                        const int axis,
                                        const navkit::core::Scalar_t perturbation_rad)
{
    using Nominal = Model::Nominal;

    typename Model::State_t perturbed = x;
    const Eigen::Matrix<navkit::core::Scalar_t, 4, 1> q_segment =
        x.template segment<4>(Nominal::AttQuat::i);
    const Eigen::Quaternion<navkit::core::Scalar_t> q_b2e{
        q_segment(0), q_segment(1), q_segment(2), q_segment(3)};
    navkit::core::Vec3 delta_theta = navkit::core::Vec3::Zero();
    delta_theta(axis) = perturbation_rad;
    const Eigen::Quaternion<navkit::core::Scalar_t> delta_q =
        navkit::core::math::quaternion_from_rotvec_rad(delta_theta);
    const Eigen::Quaternion<navkit::core::Scalar_t> q_true =
        navkit::core::math::normalized_with_positive_scalar(delta_q * q_b2e);
    perturbed.template segment<4>(Nominal::AttQuat::i) << q_true.w(), q_true.x(), q_true.y(),
        q_true.z();
    return perturbed;
}

template<typename Model>
Model::State_t gyro_bias_perturbed_state(const typename Model::State_t& x,
                                         const int axis,
                                         const navkit::core::Scalar_t perturbation_radps)
{
    using Nominal = Model::Nominal;

    typename Model::State_t perturbed = x;
    perturbed(Nominal::GyroB::i + axis) += perturbation_radps;
    return perturbed;
}

} // namespace

TEST_CASE("GNSS position update moves state toward measurement")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
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
    Model::ObservationContext ctx;
    ctx.R_e_m2 = navkit::core::Mat3::Identity();

    kf.observation_update<Model>(z, ctx);
    static_cast<void>(kf.inject());
    kf.reset();

    CHECK(kf.state()(Nominal::Pos::i) < 10.0);
}

TEST_CASE("Direct filter injection reports its accepted measurement correction")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssPosModel<StateDef>;
    navkit::core::estimation::KalmanFilter<StateDef> filter{};

    Model::ObservationContext ctx{};
    ctx.R_e_m2 = navkit::core::Mat3::Identity();
    Model::O_t measurement{};
    measurement.x() = 2.0;

    filter.observation_update<Model>(measurement, ctx);
    const decltype(filter)::AppliedCorrection_t correction = filter.inject();

    CHECK(correction.valid);
    CHECK(correction.value(Error::Pos::i) == doctest::Approx(1.0));
}

TEST_CASE("GNSS position Jacobian follows truth-minus-estimate error convention")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssPosModel<StateDef>;

    const Model::State_t x = Model::State_t::Zero();
    const Model::ObservationContext ctx{};
    const Model::H_t h = Model::compute_h(x, ctx);

    CHECK(h.template block<3, 3>(0, Error::Pos::i)
              .isApprox(Eigen::Matrix<navkit::core::Scalar_t, 3, 3>::Identity()));
}

TEST_CASE("GNSS velocity Jacobian follows truth-minus-estimate error convention")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssVelModel<StateDef>;

    const Model::State_t x = Model::State_t::Zero();
    const Model::ObservationContext ctx{};
    const Model::H_t h = Model::compute_h(x, ctx);

    CHECK(h.template block<3, 3>(0, Error::Vel::i)
              .isApprox(Eigen::Matrix<navkit::core::Scalar_t, 3, 3>::Identity()));
}

TEST_CASE("GNSS position lever-arm Jacobian matches finite difference")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssPosModel<StateDef>;

    Model::ObservationContext ctx{};
    ctx.p_b_ant_b_m << 2.0, -0.4, 0.7;
    const Model::State_t x = nominal_test_state<Model>();
    const Model::H_t h = Model::compute_h(x, ctx);
    const navkit::core::Scalar_t eps = 1.0e-6;

    for (int axis = 0; axis < 3; ++axis) {
        const Model::State_t x_plus = attitude_perturbed_state<Model>(x, axis, eps);
        const Model::State_t x_minus = attitude_perturbed_state<Model>(x, axis, -eps);
        const Model::O_t finite_difference =
            (Model::obs(x_plus, ctx) - Model::obs(x_minus, ctx)) / (2.0 * eps);
        CHECK(finite_difference.isApprox(h.col(Error::AttRotVec::i + axis), 1.0e-7));
    }
}

TEST_CASE("GNSS velocity lever-arm Jacobian matches finite difference")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssVelModel<StateDef>;

    Model::ObservationContext ctx{};
    ctx.p_b_ant_b_m << 2.0, -0.4, 0.7;
    const Model::State_t x = nominal_test_state<Model>();
    const Eigen::Quaternion<navkit::core::Scalar_t> q_b2e =
        navkit::core::estimation::q_b2e<StateDef>(x);
    const navkit::core::Vec3 omega_eb_b_radps{0.02, -0.01, 0.04};
    ctx.omega_ie_e_radps << 0.0, 0.0, 7.2921150e-5;
    ctx.omega_ib_b_meas_radps = omega_eb_b_radps + (q_b2e.conjugate() * ctx.omega_ie_e_radps);
    ctx.imu_angular_rate_valid = true;
    const Model::H_t h = Model::compute_h(x, ctx);
    const navkit::core::Scalar_t eps = 1.0e-6;

    for (int axis = 0; axis < 3; ++axis) {
        const Model::State_t x_plus = attitude_perturbed_state<Model>(x, axis, eps);
        const Model::State_t x_minus = attitude_perturbed_state<Model>(x, axis, -eps);
        const Model::O_t finite_difference =
            (Model::obs(x_plus, ctx) - Model::obs(x_minus, ctx)) / (2.0 * eps);
        CHECK(finite_difference.isApprox(h.col(Error::AttRotVec::i + axis), 1.0e-7));
    }
}

TEST_CASE("GNSS velocity gyro-bias lever-arm Jacobian matches finite difference")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Model = navkit::core::models::GnssVelModel<StateDef>;

    Model::ObservationContext ctx{};
    ctx.p_b_ant_b_m << 2.0, -0.4, 0.7;
    ctx.omega_ie_e_radps << 0.0, 0.0, 7.2921150e-5;
    ctx.omega_ib_b_meas_radps << 0.02, -0.01, 0.04;
    ctx.imu_angular_rate_valid = true;
    const Model::State_t x = nominal_test_state<Model>();
    const Model::H_t h = Model::compute_h(x, ctx);
    const navkit::core::Scalar_t eps = 1.0e-7;

    for (int axis = 0; axis < 3; ++axis) {
        const Model::State_t x_plus = gyro_bias_perturbed_state<Model>(x, axis, eps);
        const Model::State_t x_minus = gyro_bias_perturbed_state<Model>(x, axis, -eps);
        const Model::O_t finite_difference =
            (Model::obs(x_plus, ctx) - Model::obs(x_minus, ctx)) / (2.0 * eps);
        CHECK(finite_difference.isApprox(h.col(Error::GyroB::i + axis), 1.0e-7));
    }
}

TEST_CASE("GNSS velocity model matches lever-arm truth with measured angular rate")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;
    using Model = navkit::core::models::GnssVelModel<StateDef>;

    navkit::sim::GnssSimulatorConfig simulator_config{};
    simulator_config.p_b_ant_b_m << 1.0, 0.25, -0.15;
    simulator_config.noise_enabled = false;
    navkit::sim::GnssSimulator simulator{simulator_config};

    Model::State_t state = nominal_test_state<Model>();
    const navkit::core::Vec3 gyro_bias_b_radps{2.0e-4, -3.0e-4, 1.0e-4};
    state.template segment<3>(Nominal::GyroB::i) = gyro_bias_b_radps;
    const Eigen::Quaternion<navkit::core::Scalar_t> q_b2e =
        navkit::core::estimation::q_b2e<StateDef>(state);
    const navkit::core::Vec3 omega_ib_b_radps{0.02, -0.01, 0.04};

    navkit::sim::TruthSample truth{};
    truth.v_e = state.template segment<3>(Nominal::Vel::i);
    truth.q_b2e = q_b2e;
    truth.w_ib_b_radps = omega_ib_b_radps;

    Model::ObservationContext ctx{};
    ctx.p_b_ant_b_m = simulator_config.p_b_ant_b_m;
    ctx.omega_ie_e_radps =
        navkit::core::environment::planet_rate_fixed_radps<navkit::core::environment::Wgs84>();
    ctx.omega_ib_b_meas_radps = omega_ib_b_radps + gyro_bias_b_radps;
    ctx.imu_angular_rate_valid = true;

    const navkit::core::estimation::Measurement<3> measurement = simulator.generate_velocity(truth);
    CHECK(Model::obs(state, ctx).isApprox(measurement.z, 1.0e-12));
}

TEST_CASE("Sequential GNSS position and velocity inject between sensor updates")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;
    using PositionModel = navkit::core::models::GnssPosModel<StateDef>;
    using VelocityModel = navkit::core::models::GnssVelModel<StateDef>;
    using PositionSensor = navkit::core::estimation::Sensor<0U, PositionModel, 2U>;
    using VelocitySensor = navkit::core::estimation::Sensor<1U, VelocityModel, 2U>;
    using Sensors = std::tuple<PositionSensor, VelocitySensor>;
    using Filter = navkit::core::estimation::KalmanFilter<
        StateDef,
        navkit::core::estimation::DefaultInjectionPolicy<StateDef>,
        navkit::core::estimation::DefaultResetPolicy<StateDef>,
        Sensors>;
    using Update = navkit::core::estimation::UpdateAfterEachSensor<Filter>;
    using Navigator = navkit::core::estimation::
        Navigator<Filter, Sensors, navkit::core::estimation::NoOpPropagation, Update>;

    Navigator navigator{};
    Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state(Nominal::AttQuat::i) = 1.0;
    navigator.filter().set_state(initial_state);
    Filter::P_t covariance = Filter::P_t::Identity();
    covariance(Error::Pos::i, Error::Vel::i) = 0.2;
    covariance(Error::Vel::i, Error::Pos::i) = 0.2;
    covariance(Error::AttRotVec::i, Error::Pos::i) = 0.15;
    covariance(Error::Pos::i, Error::AttRotVec::i) = 0.15;
    covariance(Error::AttRotVec::i + 1, Error::Vel::i) = -0.1;
    covariance(Error::Vel::i, Error::AttRotVec::i + 1) = -0.1;
    navigator.filter().set_covariance(covariance);
    navigator.template sensor<0>().observation_context().R_e_m2 = navkit::core::Mat3::Identity();
    navigator.template sensor<1>().observation_context().R_e_m2ps2 = navkit::core::Mat3::Identity();

    navkit::core::estimation::Measurement<3> position{};
    REQUIRE(
        navkit::core::timestamp_from_seconds(10.0, navkit::core::TimeScale::Monotonic, position.t));
    position.z.x() = 2.0;
    navkit::core::estimation::Measurement<3> velocity{};
    velocity.t = position.t;
    velocity.z.x() = -3.0;
    REQUIRE(navigator.template sensor<0>().push(position));
    REQUIRE(navigator.template sensor<1>().push(velocity));

    navigator.process_measurements();

    CHECK(navigator.filter().measurement_statistics<PositionSensor>().accepted);
    CHECK(navigator.filter().measurement_statistics<VelocitySensor>().accepted);
    CHECK(std::abs(navigator.filter().state()(Nominal::Pos::i)) > 0.0);
    CHECK(std::abs(navigator.filter().state()(Nominal::Vel::i)) > 0.0);
    CHECK(navigator.filter().error_state().isZero(1.0e-15));
    CHECK(navigator.last_applied_correction().valid);

    Filter::State_t replayed_state = initial_state;
    navkit::core::estimation::DefaultInjectionPolicy<StateDef>::apply(
        replayed_state, navigator.last_applied_correction().value);
    CHECK(replayed_state.isApprox(navigator.filter().state(), 1.0e-12));
    CHECK(std::abs(navigator.last_applied_correction().value(Error::AttRotVec::i)) > 0.0);
    CHECK(std::abs(navigator.last_applied_correction().value(Error::AttRotVec::i + 1)) > 0.0);
}

TEST_CASE("Rejected same-epoch GNSS velocity leaves accepted position correction intact")
{
    using StateDef = navkit::core::estimation::InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;
    using PositionModel = navkit::core::models::GnssPosModel<StateDef>;
    using VelocityModel = navkit::core::models::GnssVelModel<StateDef>;
    using PositionSensor = navkit::core::estimation::Sensor<0U, PositionModel, 2U>;
    using VelocitySensor = navkit::core::estimation::Sensor<1U, VelocityModel, 2U>;
    using Sensors = std::tuple<PositionSensor, VelocitySensor>;
    using Filter = navkit::core::estimation::KalmanFilter<
        StateDef,
        navkit::core::estimation::DefaultInjectionPolicy<StateDef>,
        navkit::core::estimation::DefaultResetPolicy<StateDef>,
        Sensors>;
    using Update = navkit::core::estimation::UpdateAfterEachSensor<Filter>;
    using Navigator = navkit::core::estimation::
        Navigator<Filter, Sensors, navkit::core::estimation::NoOpPropagation, Update>;

    Navigator gated_navigator{};
    Navigator position_only_navigator{};
    Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state(Nominal::AttQuat::i) = 1.0;
    gated_navigator.filter().set_state(initial_state);
    position_only_navigator.filter().set_state(initial_state);
    gated_navigator.filter().set_covariance(Filter::P_t::Identity());
    position_only_navigator.filter().set_covariance(Filter::P_t::Identity());

    for (Navigator* navigator : {&gated_navigator, &position_only_navigator}) {
        navigator->template sensor<0>().observation_context().R_e_m2 =
            navkit::core::Mat3::Identity();
        navigator->template sensor<1>().observation_context().R_e_m2ps2 =
            navkit::core::Mat3::Identity();
        REQUIRE(navigator->template sensor<0>().configure_innovation_gate_probability(0.99));
        REQUIRE(navigator->template sensor<1>().configure_innovation_gate_probability(0.95));
    }

    navkit::core::estimation::Measurement<3> position{};
    REQUIRE(
        navkit::core::timestamp_from_seconds(10.0, navkit::core::TimeScale::Monotonic, position.t));
    position.z.x() = 1.0;
    navkit::core::estimation::Measurement<3> rejected_velocity{};
    rejected_velocity.t = position.t;
    rejected_velocity.z.x() = 1000.0;

    REQUIRE(gated_navigator.template sensor<0>().push(position));
    REQUIRE(gated_navigator.template sensor<1>().push(rejected_velocity));
    REQUIRE(position_only_navigator.template sensor<0>().push(position));

    gated_navigator.process_measurements();
    position_only_navigator.process_measurements();

    CHECK(gated_navigator.filter().measurement_statistics<PositionSensor>().accepted);
    CHECK_FALSE(gated_navigator.filter().measurement_statistics<VelocitySensor>().accepted);
    CHECK(gated_navigator.filter().state().isApprox(position_only_navigator.filter().state(),
                                                    1.0e-13));
    CHECK(gated_navigator.filter().covariance().isApprox(
        position_only_navigator.filter().covariance(), 1.0e-13));
    CHECK(gated_navigator.last_applied_correction().value.isApprox(
        position_only_navigator.last_applied_correction().value, 1.0e-13));
}

TEST_CASE("GNSS simulator publishes due samples only in configured active windows")
{
    navkit::sim::GnssSimulatorConfig config{};
    config.rate = navkit::core::RationalRate{.samples = 1U, .s = 1U};
    config.active_windows = {{.start_s = 0.0, .end_s = 2.0}, {.start_s = 4.0, .end_s = 10.0}};
    navkit::sim::GnssSimulator simulator{config};
    navkit::sim::TruthSample truth{};

    truth.t = navkit::core::Timestamp{.s = 100U};
    CHECK(simulator.should_generate(truth));
    truth.t = navkit::core::Timestamp{.s = 101U};
    CHECK(simulator.should_generate(truth));
    truth.t = navkit::core::Timestamp{.s = 102U};
    CHECK_FALSE(simulator.should_generate(truth));
    truth.t = navkit::core::Timestamp{.s = 103U};
    CHECK_FALSE(simulator.should_generate(truth));
    truth.t = navkit::core::Timestamp{.s = 104U};
    CHECK(simulator.should_generate(truth));
    truth.t = navkit::core::Timestamp{.s = 105U};
    CHECK(simulator.should_generate(truth));
}
