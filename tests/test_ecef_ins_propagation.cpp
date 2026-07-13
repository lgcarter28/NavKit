// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "test_main.hpp"

#include <tuple>

namespace navkit::core::estimation::test
{

namespace
{

using navkit::core::Vec3;
using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;
using StationaryPropagation = EcefInsPropagation<Wgs84, J2<Wgs84>>;
using StationaryFilter = KalmanFilter<InsStateDef>;

struct NonzeroProcessNoise
{
    [[nodiscard]] static Vec3 gyro_white_noise_psd_rad2ps()
    {
        return Vec3::Constant(1.0e-8);
    }

    [[nodiscard]] static Vec3 accel_white_noise_psd_m2ps3()
    {
        return Vec3::Constant(1.0e-4);
    }

    [[nodiscard]] static Vec3 gyro_bias_rw_psd_rad2ps3()
    {
        return Vec3::Constant(1.0e-10);
    }

    [[nodiscard]] static Vec3 accel_bias_rw_psd_m2ps5()
    {
        return Vec3::Constant(1.0e-6);
    }
};

using NoisyPropagation = EcefInsPropagation<Wgs84, J2<Wgs84>, NonzeroProcessNoise>;

[[nodiscard]] sim::TruthSample stationary_sample(const Time_t time_s)
{
    sim::TruthSample sample;
    sample.time = time_s;
    sample.p_e = Vec3{Wgs84::a_m, 0.0, 0.0};
    sample.v_e.setZero();
    sample.q_eb.setIdentity();
    return sample;
}

void initialize_stationary_filter(StationaryFilter& filter)
{
    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<InsStateDef::Pos>(state) = Vec3{Wgs84::a_m, 0.0, 0.0};
    segment<InsStateDef::Vel>(state).setZero();
    segment<InsStateDef::Att>(state).setZero();
    filter.set_state(state);

    StationaryFilter::P_t covariance = StationaryFilter::P_t::Zero();
    filter.set_covariance(covariance);
}

[[nodiscard]] bool ideal_stationary_increment(ImuIncrement& increment)
{
    sim::ImuSimulator simulator;
    simulator.initialize(stationary_sample(0.0));
    return simulator.generate(stationary_sample(1.0), increment);
}

} // namespace

TEST_CASE("Rotation-vector quaternion helper round-trips small attitude corrections")
{
    static_assert(PropagationPolicy<StationaryPropagation, InsStateDef>);
    static_assert(PropagationPolicy<NoisyPropagation, InsStateDef>);

    const Vec3 phi{0.01, -0.02, 0.03};
    const auto q = navkit::core::math::quaternion_from_rotation_vector(phi);
    const auto recovered = navkit::core::math::rotation_vector_from_quaternion(q);

    CHECK(recovered.isApprox(phi, 1.0e-12));
}

TEST_CASE("Two-sample coning and sculling compensation applies cross terms")
{
    const Vec3 theta_1{0.01, 0.0, 0.0};
    const Vec3 theta_2{0.0, 0.02, 0.0};
    const Vec3 delta_v_1{0.0, 0.0, 1.0};
    const Vec3 delta_v_2{0.0, 1.0, 0.0};

    const auto compensated = coning_sculling_two_sample(theta_1, delta_v_1, theta_2, delta_v_2);

    const auto expected_theta = theta_1 + theta_2 + ((2.0 / 3.0) * theta_1.cross(theta_2));
    const auto expected_delta_v =
        delta_v_1 + delta_v_2 + (0.5 * (theta_1 + theta_2).cross(delta_v_1 + delta_v_2)) +
        ((2.0 / 3.0) * ((theta_1.cross(delta_v_2)) + (delta_v_1.cross(theta_2))));

    CHECK(compensated.delta_theta_ib_b_rad.isApprox(expected_theta, 1.0e-15));
    CHECK(compensated.delta_v_ib_b_mps.isApprox(expected_delta_v, 1.0e-15));
}

TEST_CASE("Ideal stationary ECEF IMU increment preserves nominal PVA")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);
    ImuIncrement increment;
    REQUIRE(ideal_stationary_increment(increment));

    REQUIRE(StationaryPropagation::process_imu_increment<InsStateDef>(filter.state(), increment));

    const auto& state = filter.state();
    CHECK(segment<InsStateDef::Pos>(state).isApprox(Vec3{Wgs84::a_m, 0.0, 0.0}, 1.0e-8));
    CHECK(segment<InsStateDef::Vel>(state).isZero(1.0e-10));
    CHECK(segment<InsStateDef::Att>(state).isZero(1.0e-12));
}

TEST_CASE("ECEF INS covariance prediction remains symmetric and process-noise driven")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);
    ImuIncrement increment;
    REQUIRE(ideal_stationary_increment(increment));

    StationaryFilter::P_t phi{};
    StationaryFilter::P_t qd{};
    REQUIRE(NoisyPropagation::covariance_step_from_increment<InsStateDef>(
        filter.state(), increment, phi, qd));
    REQUIRE(NoisyPropagation::process_imu_increment<InsStateDef>(filter.state(), increment));
    filter.propagate_covariance(phi, qd);

    const auto& covariance = filter.covariance();
    CHECK(covariance.isApprox(covariance.transpose(), 1.0e-15));
    CHECK(covariance(InsStateDef::Vel::i, InsStateDef::Vel::i) > 0.0);
    CHECK(covariance(InsStateDef::Att::i, InsStateDef::Att::i) > 0.0);
    CHECK(covariance(InsStateDef::GyroB::i, InsStateDef::GyroB::i) > 0.0);
    CHECK(covariance(InsStateDef::AccB::i, InsStateDef::AccB::i) > 0.0);
}

TEST_CASE("ECEF INS propagation rejects invalid IMU intervals without throwing")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);

    CHECK_FALSE(
        StationaryPropagation::process_imu_increment<InsStateDef>(filter.state(), ImuIncrement{}));
}

} // namespace navkit::core::estimation::test
