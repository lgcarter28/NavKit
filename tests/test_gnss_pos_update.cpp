// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"
#include "test_main.hpp"

namespace
{

template<typename Model>
typename Model::State_t nominal_test_state()
{
    using Nominal = typename Model::Nominal;

    typename Model::State_t x = Model::State_t::Zero();
    x.template segment<3>(Nominal::Pos::i) << 10.0, 20.0, 30.0;
    x.template segment<3>(Nominal::Vel::i) << 1.0, 2.0, 3.0;
    const Eigen::Quaternion<navkit::core::Scalar_t> q_b2e =
        navkit::core::math::quaternion_from_rpy_rad(navkit::core::Vec3{0.15, -0.08, 0.21});
    x.template segment<4>(Nominal::AttQuat::i) << q_b2e.w(), q_b2e.x(), q_b2e.y(), q_b2e.z();
    return x;
}

template<typename Model>
typename Model::State_t attitude_perturbed_state(const typename Model::State_t& x,
                                                 const int axis,
                                                 const navkit::core::Scalar_t perturbation_rad)
{
    using Nominal = typename Model::Nominal;

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
    kf.inject();
    kf.reset();

    CHECK(kf.state()(Nominal::Pos::i) < 10.0);
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
    ctx.omega_eb_b_radps << 0.02, -0.01, 0.04;
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
