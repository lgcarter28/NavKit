// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/frames/Frames.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Skew.hpp"
#include "navkit/sim/sensors/ImuSimulator.hpp"
#include "navkit/sim/trajectory/TrajectoryAttitude.hpp"
#include "navkit/sim/trajectory/TrajectoryProfiles.hpp"
#include "test_main.hpp"

#include <numbers>
#include <tuple>

namespace navkit::core::estimation::test
{

namespace
{

using navkit::core::Vec3;
using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;
using StationaryPropagation = EcefInsPropagation<Wgs84, J2<Wgs84>>;
using StationaryFilter = KalmanFilter<InsGyroAccelBiasStateDef>;
using NominalStateDef = InsGyroAccelBiasStateDef::Nominal;
using ErrorStateDef = InsGyroAccelBiasStateDef::Error;

struct NonzeroProcessNoise
{
    using ProcessNoise_t = ImuProcessNoise;

    // The policy contract deliberately exposes an immutable header-owned Eigen configuration.
    // NOLINTNEXTLINE(bugprone-throwing-static-initialization,readability-identifier-naming)
    inline static const ProcessNoise_t process_noise{
        .gyro_white_noise_psd_rad2ps = Vec3::Constant(1.0e-8),
        .accel_white_noise_psd_m2ps3 = Vec3::Constant(1.0e-4),
        .gyro_bias_drive_psd_rad2ps3 = Vec3::Constant(1.0e-10),
        .accel_bias_drive_psd_m2ps5 = Vec3::Constant(1.0e-6),
    };
};

struct NonzeroImuBiasDynamics
{
    using ImuBiasDynamics_t = GaussMarkovImuBiasDynamics;

    // The policy contract deliberately exposes an immutable header-owned Eigen configuration.
    // NOLINTNEXTLINE(bugprone-throwing-static-initialization,readability-identifier-naming)
    inline static const ImuBiasDynamics_t imu_bias_dynamics{
        .gyro_bias_correlation_rate_1ps = Vec3::Constant(0.5),
        .accel_bias_correlation_rate_1ps = Vec3::Constant(0.25),
    };
};

using NoisyPropagation =
    EcefInsPropagation<Wgs84, J2<Wgs84>, NonzeroProcessNoise, NonzeroImuBiasDynamics>;

struct NonrotatingTestPlanet
{
    using InertialFrame = navkit::core::frames::PlanetCenteredInertial;
    using FixedFrame = navkit::core::frames::PlanetCenteredPlanetFixed;

    static constexpr Scalar_t mu_m3_s2 = 0.0;
    static constexpr Scalar_t omega_rad_s = 0.0;
};

struct ZeroTestGravity
{
    using Planet_t = NonrotatingTestPlanet;
    using Frame_t = typename Planet_t::FixedFrame;

    [[nodiscard]] static Vec3 acceleration(const Vec3&)
    {
        return Vec3::Zero();
    }
};

using MidpointPropagation = EcefInsPropagation<NonrotatingTestPlanet, ZeroTestGravity>;
using PrecompensatedPropagation = EcefInsPropagation<NonrotatingTestPlanet,
                                                     ZeroTestGravity,
                                                     ZeroImuProcessNoise,
                                                     ZeroGaussMarkovImuBiasDynamics,
                                                     512U,
                                                     256U,
                                                     100.0,
                                                     false>;

[[nodiscard]] sim::TruthSample stationary_sample(const Time_t time_s)
{
    sim::TruthSample sample{};
    const bool timestamp_valid = timestamp_from_seconds(time_s, TimeScale::Monotonic, sample.t);
    if (!timestamp_valid) {
        return {};
    }
    sample.p_e = Vec3{Wgs84::a_m, 0.0, 0.0};
    sample.v_e.setZero();
    sample.q_b2e.setIdentity();
    return sample;
}

void initialize_stationary_filter(StationaryFilter& filter)
{
    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::Pos>(state) = Vec3{Wgs84::a_m, 0.0, 0.0};
    segment<NominalStateDef::Vel>(state).setZero();
    segment<NominalStateDef::AttQuat>(state) << 1.0, 0.0, 0.0, 0.0;
    filter.set_state(state);

    StationaryFilter::P_t covariance = StationaryFilter::P_t::Zero();
    filter.set_covariance(covariance);
}

[[nodiscard]] bool ideal_stationary_increment(ImuIncrement& increment)
{
    sim::ImuSimulator<> simulator;
    simulator.initialize(stationary_sample(0.0));
    return simulator.generate(stationary_sample(1.0), increment);
}

} // namespace

TEST_CASE("Rotation-vector quaternion helper round-trips small attitude corrections")
{
    static_assert(PropagationPolicy<StationaryPropagation, InsGyroAccelBiasStateDef>);
    static_assert(PropagationPolicy<NoisyPropagation, InsGyroAccelBiasStateDef>);

    const Vec3 phi{0.01, -0.02, 0.03};
    const Eigen::Quaternion<Scalar_t> q = navkit::core::math::quaternion_from_rotvec_rad(phi);
    const Vec3 recovered = navkit::core::math::rotvec_rad_from_quaternion(q);

    CHECK(recovered.isApprox(phi, 1.0e-12));
}

TEST_CASE("Two-sample coning and sculling compensation applies cross terms")
{
    const Vec3 theta_1{0.01, 0.0, 0.0};
    const Vec3 theta_2{0.0, 0.02, 0.0};
    const Vec3 delta_v_1{0.0, 0.0, 1.0};
    const Vec3 delta_v_2{0.0, 1.0, 0.0};

    const ConingSculling compensated =
        coning_sculling_two_sample(theta_1, delta_v_1, theta_2, delta_v_2);

    const Vec3 expected_theta = theta_1 + theta_2 + ((2.0 / 3.0) * theta_1.cross(theta_2));
    const Vec3 expected_delta_v =
        delta_v_1 + delta_v_2 +
        ((2.0 / 3.0) * ((theta_1.cross(delta_v_2)) + (delta_v_1.cross(theta_2))));

    CHECK(compensated.delta_theta_ib_b_rad.isApprox(expected_theta, 1.0e-15));
    CHECK(compensated.delta_v_ib_b_mps.isApprox(expected_delta_v, 1.0e-15));
}

TEST_CASE("Midpoint attitude resolves paired delta velocity without double-counting mean rotation")
{
    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::AttQuat>(state) << 1.0, 0.0, 0.0, 0.0;

    ImuIncrement first{};
    REQUIRE(timestamp_from_seconds(0.5, TimeScale::Monotonic, first.t));
    first.dt_s = 0.5;
    first.delta_theta_ib_b_rad = Vec3{0.0, 0.0, 0.25 * std::numbers::pi};
    first.delta_v_ib_b_mps = Vec3{0.5, 0.0, 0.0};

    ImuIncrement second = first;
    REQUIRE(timestamp_from_seconds(1.0, TimeScale::Monotonic, second.t));

    MidpointPropagation propagation;
    REQUIRE(propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(first, second, state));

    const Scalar_t sqrt_half = std::sqrt(0.5);
    const Vec3 expected_velocity{sqrt_half, sqrt_half, 0.0};
    CHECK(segment<NominalStateDef::Vel>(state).isApprox(expected_velocity, 1.0e-12));
    CHECK(segment<NominalStateDef::Pos>(state).isApprox(0.5 * expected_velocity, 1.0e-12));

    const Eigen::Quaternion<Scalar_t> expected_attitude{
        Eigen::AngleAxis<Scalar_t>{0.5 * std::numbers::pi, Vec3::UnitZ()}};
    const Eigen::Matrix<Scalar_t, 4, 1> expected_quaternion{
        expected_attitude.w(), expected_attitude.x(), expected_attitude.y(), expected_attitude.z()};
    CHECK(segment<NominalStateDef::AttQuat>(state).isApprox(expected_quaternion, 1.0e-12));
}

TEST_CASE("Two-sample coning and sculling rejects unequal or noncontiguous subintervals")
{
    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::AttQuat>(state) << 1.0, 0.0, 0.0, 0.0;

    ImuIncrement first{};
    REQUIRE(timestamp_from_seconds(0.1, TimeScale::Monotonic, first.t));
    first.dt_s = 0.1;

    ImuIncrement unequal = first;
    REQUIRE(timestamp_from_seconds(0.3, TimeScale::Monotonic, unequal.t));
    unequal.dt_s = 0.2;

    MidpointPropagation propagation;
    CHECK_FALSE(
        propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(first, unequal, state));

    ImuIncrement noncontiguous = first;
    REQUIRE(timestamp_from_seconds(0.25, TimeScale::Monotonic, noncontiguous.t));
    CHECK_FALSE(propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(
        first, noncontiguous, state));
}

TEST_CASE("Rotating-Earth ideal IMU pair reconstructs dynamic non-collinear attitude truth")
{
    sim::TruthSample first_truth = stationary_sample(0.0);
    sim::TruthSample second_truth = stationary_sample(0.001);
    sim::TruthSample third_truth = stationary_sample(0.002);
    second_truth.q_b2e = navkit::core::math::normalized_with_positive_scalar(
        first_truth.q_b2e *
        navkit::core::math::quaternion_from_rotvec_rad(Vec3{1.0e-4, 2.0e-5, -3.0e-5}));
    third_truth.q_b2e = navkit::core::math::normalized_with_positive_scalar(
        second_truth.q_b2e *
        navkit::core::math::quaternion_from_rotvec_rad(Vec3{-2.0e-5, 8.0e-5, 4.0e-5}));

    sim::ImuSimulator<> simulator;
    simulator.initialize(first_truth);
    ImuIncrement first_increment{};
    ImuIncrement second_increment{};
    REQUIRE(simulator.generate(second_truth, first_increment));
    REQUIRE(simulator.generate(third_truth, second_increment));

    StationaryFilter filter;
    initialize_stationary_filter(filter);
    StationaryPropagation propagation;
    REQUIRE(propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(
        first_increment, second_increment, filter.state()));

    const StationaryFilter::State_t& state = filter.state();
    const Eigen::Quaternion<Scalar_t> q_reconstructed =
        navkit::core::estimation::q_b2e<InsGyroAccelBiasStateDef>(state);
    CAPTURE((segment<NominalStateDef::Pos>(state) - third_truth.p_e).norm());
    CAPTURE((segment<NominalStateDef::Vel>(state) - third_truth.v_e).norm());
    CAPTURE(q_reconstructed.angularDistance(third_truth.q_b2e));
    CHECK(q_reconstructed.angularDistance(third_truth.q_b2e) < 5.0e-8);
    CHECK(segment<NominalStateDef::Pos>(state).isApprox(third_truth.p_e, 1.0e-6));
    CHECK((segment<NominalStateDef::Vel>(state) - third_truth.v_e).norm() < 1.0e-6);
}

TEST_CASE("Midpoint mechanization reconstructs a high-specific-force ballistic truth pair")
{
    sim::BallisticTrajectoryConfig config{};
    config.profile.duration_s = 0.05;
    config.profile.rate = RationalRate{.samples = 1000U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{Wgs84::a_m, 0.0, 0.0};

    Mat3 C_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, C_e2n));
    Eigen::Quaternion<Scalar_t> q_b2n{};
    REQUIRE(
        sim::velocity_aligned_attitude_b2n(Vec3{0.5, 0.0, -std::sqrt(0.75)}, Vec3::UnitZ(), q_b2n));
    config.profile.q_b2e = navkit::core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<Scalar_t>{C_e2n.transpose()} * q_b2n);
    config.launch_pad_duration_s = 0.0;
    config.boost_duration_s = config.profile.duration_s;
    config.boost_acceleration_b_x_mps2 = 500.0;

    const sim::TruthTrajectory truth = sim::ballistic_trajectory(config);
    REQUIRE(truth.size() > 6U);
    const sim::TruthSample& start = truth.samples().at(3U);
    const sim::TruthSample& middle = truth.samples().at(4U);
    const sim::TruthSample& end = truth.samples().at(5U);

    sim::ImuSimulator<> simulator;
    simulator.initialize(start);
    ImuIncrement first{};
    ImuIncrement second{};
    REQUIRE(simulator.generate(middle, first));
    REQUIRE(simulator.generate(end, second));

    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::Pos>(state) = start.p_e;
    segment<NominalStateDef::Vel>(state) = start.v_e;
    navkit::core::estimation::set_q_b2e<InsGyroAccelBiasStateDef>(state, start.q_b2e);

    StationaryPropagation propagation;
    REQUIRE(propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(first, second, state));

    const Eigen::Quaternion<Scalar_t> q_reconstructed =
        navkit::core::estimation::q_b2e<InsGyroAccelBiasStateDef>(state);
    CHECK(q_reconstructed.angularDistance(end.q_b2e) < 5.0e-6);
    CHECK(segment<NominalStateDef::Pos>(state).isApprox(end.p_e, 5.0e-5));
    CHECK(segment<NominalStateDef::Vel>(state).isApprox(end.v_e, 5.0e-4));
}

TEST_CASE("ECEF INS covariance dynamics use the same midpoint attitude as delta velocity")
{
    StationaryFilter::State_t state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::AttQuat>(state) << 1.0, 0.0, 0.0, 0.0;

    MechanizedImuInterval interval{};
    interval.dt_s = 1.0;
    interval.delta_theta_eb_b_rad = Vec3{0.0, 0.0, 0.5 * std::numbers::pi};
    interval.specific_force_ib_b_mps2 = Vec3::UnitX();

    const StationaryFilter::P_t F =
        MidpointPropagation::build_f_matrix<InsGyroAccelBiasStateDef>(state, interval);
    const Eigen::Matrix3d C_b2e_mid =
        Eigen::AngleAxisd(0.25 * std::numbers::pi, Vec3::UnitZ()).toRotationMatrix();
    const Vec3 f_ib_e = C_b2e_mid * Vec3::UnitX();

    CHECK(F.block<3, 3>(ErrorStateDef::Vel::i, ErrorStateDef::AttRotVec::i)
              .isApprox(-navkit::core::math::skew_symmetric(f_ib_e), 1.0e-12));
    CHECK(
        F.block<3, 3>(ErrorStateDef::Vel::i, ErrorStateDef::AccB::i).isApprox(-C_b2e_mid, 1.0e-12));
    CHECK(F.block<3, 3>(ErrorStateDef::AttRotVec::i, ErrorStateDef::GyroB::i)
              .isApprox(-C_b2e_mid, 1.0e-12));
}

TEST_CASE("Internally compensated IMU intervals are mechanized sequentially")
{
    StationaryFilter::State_t pair_state = StationaryFilter::State_t::Zero();
    segment<NominalStateDef::AttQuat>(pair_state) << 1.0, 0.0, 0.0, 0.0;
    StationaryFilter::State_t sequential_state = pair_state;

    ImuIncrement first{};
    REQUIRE(timestamp_from_seconds(0.5, TimeScale::Monotonic, first.t));
    first.dt_s = 0.5;
    first.delta_theta_ib_b_rad = Vec3{0.0, 0.0, 0.1};
    first.delta_v_ib_b_mps = Vec3{0.5, 0.1, 0.0};
    ImuIncrement second = first;
    REQUIRE(timestamp_from_seconds(1.0, TimeScale::Monotonic, second.t));
    second.delta_theta_ib_b_rad = Vec3{0.0, 0.0, 0.2};
    second.delta_v_ib_b_mps = Vec3{0.4, -0.2, 0.0};

    PrecompensatedPropagation propagation;
    REQUIRE(propagation.process_imu_increment_pair<InsGyroAccelBiasStateDef>(
        first, second, pair_state));
    REQUIRE(propagation.process_imu_increment<InsGyroAccelBiasStateDef>(first, sequential_state));
    REQUIRE(propagation.process_imu_increment<InsGyroAccelBiasStateDef>(second, sequential_state));

    CHECK(pair_state.isApprox(sequential_state, 1.0e-14));
}

TEST_CASE("Ideal stationary ECEF IMU increment preserves nominal PVA")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);
    ImuIncrement increment;
    REQUIRE(ideal_stationary_increment(increment));

    StationaryPropagation propagation;
    REQUIRE(propagation.process_imu_increment<InsGyroAccelBiasStateDef>(increment, filter.state()));

    const StationaryFilter::State_t& state = filter.state();
    CHECK(segment<NominalStateDef::Pos>(state).isApprox(Vec3{Wgs84::a_m, 0.0, 0.0}, 1.0e-8));
    CHECK(segment<NominalStateDef::Vel>(state).isZero(1.0e-10));
    CHECK(segment<NominalStateDef::AttQuat>(state).isApprox(
        Eigen::Matrix<Scalar_t, 4, 1>{1.0, 0.0, 0.0, 0.0}, 1.0e-12));
}

TEST_CASE("Ideal stationary ECEF IMU increments keep pure strapdown bounded at 1000 Hz")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);

    sim::ImuSimulator<> simulator;
    simulator.initialize(stationary_sample(0.0));

    constexpr Time_t dt_s = 0.001;
    constexpr int sample_count = 10000;
    for (int k = 1; k <= sample_count; ++k) {
        ImuIncrement increment;
        REQUIRE(simulator.generate(stationary_sample(static_cast<Time_t>(k) * dt_s), increment));
        StationaryPropagation propagation;
        REQUIRE(
            propagation.process_imu_increment<InsGyroAccelBiasStateDef>(increment, filter.state()));
    }

    const StationaryFilter::State_t& state = filter.state();
    CHECK(segment<NominalStateDef::Pos>(state).isApprox(Vec3{Wgs84::a_m, 0.0, 0.0}, 1.0e-3));
    CHECK(segment<NominalStateDef::Vel>(state).isZero(1.0e-6));
    CHECK(segment<NominalStateDef::AttQuat>(state).isApprox(
        Eigen::Matrix<Scalar_t, 4, 1>{1.0, 0.0, 0.0, 0.0}, 1.0e-10));
}

TEST_CASE("ECEF INS covariance prediction remains symmetric and process-noise driven")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);
    ImuIncrement increment;
    REQUIRE(ideal_stationary_increment(increment));

    StationaryFilter::P_t phi{};
    StationaryFilter::P_t qd{};
    NoisyPropagation propagation;
    REQUIRE(propagation.covariance_step_from_increment<InsGyroAccelBiasStateDef>(
        filter.state(), increment, phi, qd));
    REQUIRE(propagation.process_imu_increment<InsGyroAccelBiasStateDef>(increment, filter.state()));
    filter.propagate_covariance(phi, qd);

    const StationaryFilter::P_t& covariance = filter.covariance();
    CHECK(covariance.isApprox(covariance.transpose(), 1.0e-15));
    CHECK(covariance(ErrorStateDef::Vel::i, ErrorStateDef::Vel::i) > 0.0);
    CHECK(covariance(ErrorStateDef::AttRotVec::i, ErrorStateDef::AttRotVec::i) > 0.0);
    CHECK(covariance(ErrorStateDef::GyroB::i, ErrorStateDef::GyroB::i) > 0.0);
    CHECK(covariance(ErrorStateDef::AccB::i, ErrorStateDef::AccB::i) > 0.0);
}

TEST_CASE("ECEF INS covariance prediction applies Gauss-Markov bias damping")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);
    ImuIncrement increment;
    REQUIRE(ideal_stationary_increment(increment));

    StationaryFilter::P_t phi{};
    StationaryFilter::P_t qd{};
    NoisyPropagation propagation;
    REQUIRE(propagation.covariance_step_from_increment<InsGyroAccelBiasStateDef>(
        filter.state(), increment, phi, qd));

    CHECK(phi(ErrorStateDef::GyroB::i + 0, ErrorStateDef::GyroB::i + 0) == doctest::Approx(0.5));
    CHECK(phi(ErrorStateDef::AccB::i + 0, ErrorStateDef::AccB::i + 0) == doctest::Approx(0.75));
}

TEST_CASE("ECEF INS propagation rejects invalid IMU intervals without throwing")
{
    StationaryFilter filter;
    initialize_stationary_filter(filter);

    StationaryPropagation propagation;
    CHECK_FALSE(propagation.process_imu_increment<InsGyroAccelBiasStateDef>(ImuIncrement{},
                                                                            filter.state()));
}

} // namespace navkit::core::estimation::test
