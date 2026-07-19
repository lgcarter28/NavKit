// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntimeConfig.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "test_main.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace navkit::sim::test
{

namespace
{

using navkit::core::Vec3;
using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;
using DefaultImuSimulator = ImuSimulator<>;

[[nodiscard]] TruthSample stationary_sample(navkit::core::Time_t time_s)
{
    TruthSample sample;
    sample.time = time_s;
    sample.p_e = Vec3{Wgs84::a_m, 0.0, 0.0};
    sample.v_e.setZero();
    sample.q_b2e.setIdentity();
    return sample;
}

[[nodiscard]] TruthSample stationary_body_z_specific_force_sample(navkit::core::Time_t time_s)
{
    constexpr double half_pi_rad = 1.57079632679489661923;

    TruthSample sample = stationary_sample(time_s);
    sample.q_b2e = Eigen::AngleAxisd(half_pi_rad, Vec3::UnitY());
    return sample;
}

[[nodiscard]] bool approx_vec(const Vec3& actual, const Vec3& expected, double tolerance = 1.0e-12)
{
    return actual.isApprox(expected, tolerance);
}

struct RecordingNavigator
{
    int push_count{0};
    navkit::core::estimation::ImuIncrement last_increment{};

    [[nodiscard]] bool push_imu(const navkit::core::estimation::ImuIncrement& increment)
    {
        ++push_count;
        last_increment = increment;
        return true;
    }
};

} // namespace

TEST_CASE("IMU increment sample defaults to zero")
{
    const navkit::core::estimation::ImuIncrement increment;

    CHECK(increment.time_s == doctest::Approx(0.0));
    CHECK(increment.dt_s == doctest::Approx(0.0));
    CHECK(increment.delta_theta_ib_b_rad.isZero());
    CHECK(increment.delta_v_ib_b_mps.isZero());
}

TEST_CASE("Ideal stationary ECEF IMU truth includes Earth-rate gyro and specific force")
{
    const auto previous = stationary_sample(0.0);
    const auto current = stationary_sample(1.0);

    ImuInterval interval;
    REQUIRE(DefaultImuSimulator::interval_from_truth_ecef(previous, current, interval));
    const auto gravity_e = J2<Wgs84>::acceleration(previous.p_e);
    const ImuIncrement truth = DefaultImuSimulator::increment_from_interval(interval);

    CHECK(interval.time_s == doctest::Approx(1.0));
    CHECK(interval.dt_s == doctest::Approx(1.0));
    CHECK(truth.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(truth.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(truth.delta_theta_ib_b_rad.z() == doctest::Approx(Wgs84::omega_rad_s));
    CHECK(approx_vec(truth.delta_v_ib_b_mps, -gravity_e));
    CHECK(truth.delta_v_ib_b_mps.x() > 0.0);
}

TEST_CASE("Ideal gyro truth combines ECEF attitude delta with Earth rotation")
{
    auto previous = stationary_sample(0.0);
    auto current = stationary_sample(2.0);
    current.q_b2e = Eigen::AngleAxisd(0.1, Vec3::UnitZ());

    ImuInterval interval;
    REQUIRE(DefaultImuSimulator::interval_from_truth_ecef(previous, current, interval));
    const ImuIncrement truth = DefaultImuSimulator::increment_from_interval(interval);

    CHECK(truth.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(truth.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(truth.delta_theta_ib_b_rad.z() == doctest::Approx(0.1 + (2.0 * Wgs84::omega_rad_s)));
}

TEST_CASE("Ideal IMU truth interval rejects non-increasing timestamps without throwing")
{
    const auto previous = stationary_sample(1.0);
    const auto current = stationary_sample(1.0);
    ImuInterval interval;

    CHECK_FALSE(DefaultImuSimulator::interval_from_truth_ecef(previous, current, interval));
    CHECK(interval.time_s == doctest::Approx(0.0));
    CHECK(interval.dt_s == doctest::Approx(0.0));
    CHECK(interval.omega_ib_b_radps.isZero());
    CHECK(interval.specific_force_ib_b_mps2.isZero());
}

TEST_CASE("IMU triad calibration applies scale, nonorthogonality, and misalignment ordering")
{
    ImuTriadErrorConfig config;
    config.scale_factor = Vec3{0.1, 0.0, 0.0};
    config.nonorthogonality = Vec3{0.2, 0.3, 0.0};

    const auto output = DefaultImuSimulator::calibration_matrix_apply(Vec3{1.0, 0.0, 0.0}, config);

    CHECK(output.x() == doctest::Approx(1.1));
    CHECK(output.y() == doctest::Approx(0.2));
    CHECK(output.z() == doctest::Approx(0.3));
}

TEST_CASE("IMU simulator applies deterministic bias and quantization to raw increments")
{
    ImuSimulatorConfig config;
    config.gyro.bias_turnon = Vec3{0.26, 0.0, 0.0};
    config.gyro.quantization = Vec3{0.1, 0.0, 0.0};
    config.accel.bias_turnon = Vec3{0.0, 0.2, 0.0};

    DefaultImuSimulator simulator(config);

    const auto previous = stationary_sample(0.0);
    const auto current = stationary_sample(1.0);
    ImuInterval interval;
    REQUIRE(DefaultImuSimulator::interval_from_truth_ecef(previous, current, interval));
    const ImuIncrement truth = DefaultImuSimulator::increment_from_interval(interval);
    ImuIncrement raw;
    REQUIRE(simulator.generate(previous, current, raw));

    CHECK(raw.time_s == doctest::Approx(current.time));
    CHECK(raw.dt_s == doctest::Approx(1.0));
    CHECK(raw.delta_theta_ib_b_rad.x() == doctest::Approx(0.3));
    CHECK(raw.delta_theta_ib_b_rad.z() == doctest::Approx(truth.delta_theta_ib_b_rad.z()));
    CHECK(raw.delta_v_ib_b_mps.x() == doctest::Approx(truth.delta_v_ib_b_mps.x()));
    CHECK(raw.delta_v_ib_b_mps.y() == doctest::Approx(0.2));
}

TEST_CASE("IMU simulator stateful generation consumes consecutive samples")
{
    DefaultImuSimulator simulator;
    ImuIncrement increment;

    CHECK_FALSE(simulator.generate(stationary_sample(0.25), increment));
    CHECK(increment.dt_s == doctest::Approx(0.0));

    simulator.initialize(stationary_sample(0.0));
    REQUIRE(simulator.generate(stationary_sample(0.25), increment));

    ImuInterval interval;
    REQUIRE(DefaultImuSimulator::interval_from_truth_ecef(
        stationary_sample(0.0), stationary_sample(0.25), interval));
    const ImuIncrement truth = DefaultImuSimulator::increment_from_interval(interval);

    CHECK(increment.time_s == doctest::Approx(0.25));
    CHECK(increment.dt_s == doctest::Approx(0.25));
    CHECK(increment.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(increment.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(increment.delta_theta_ib_b_rad.z() == doctest::Approx(0.25 * Wgs84::omega_rad_s));
    CHECK(increment.delta_v_ib_b_mps.x() == doctest::Approx(truth.delta_v_ib_b_mps.x()));
    CHECK(increment.delta_v_ib_b_mps.y() == doctest::Approx(0.0));
    CHECK(increment.delta_v_ib_b_mps.z() == doctest::Approx(0.0));

    CHECK_FALSE(simulator.generate(stationary_sample(0.25), increment));
}

TEST_CASE("IMU runtime cumulative increments advance at generation rate")
{
    const nlohmann::json config =
        nlohmann::json{{"imu", {{"type", "ideal"}, {"seed", 1U}, {"rate_hz", 1000.0}}}};

    navkit::app_support::ImuRuntime<DefaultImuSimulator> runtime(config);
    RecordingNavigator navigator{};
    navkit::app_support::ImuRuntimeSample first_output{};
    navkit::app_support::ImuRuntimeSample second_output{};
    navkit::app_support::ImuRuntimeSample third_output{};

    REQUIRE(runtime.process(stationary_body_z_specific_force_sample(0.0), navigator, first_output));
    CHECK_FALSE(first_output.generated);
    CHECK(navigator.push_count == 0);

    REQUIRE(
        runtime.process(stationary_body_z_specific_force_sample(1.0), navigator, second_output));
    REQUIRE(runtime.process(stationary_body_z_specific_force_sample(2.0), navigator, third_output));

    CHECK(navigator.push_count == 2);
    CHECK(second_output.generated);
    CHECK(third_output.generated);
    CHECK(second_output.truth.delta_v_ib_b_mps.z() > 0.0);
    CHECK(third_output.truth_cumsum_delta_v_ib_b_mps.z() ==
          doctest::Approx(2.0 * second_output.truth.delta_v_ib_b_mps.z()));
    CHECK(third_output.measured_cumsum_delta_v_ib_b_mps.z() ==
          doctest::Approx(2.0 * second_output.measured.delta_v_ib_b_mps.z()));
    CHECK(third_output.truth_cumsum_delta_theta_ib_b_rad.z() ==
          doctest::Approx(second_output.truth.delta_theta_ib_b_rad.z() +
                          third_output.truth.delta_theta_ib_b_rad.z()));
}

TEST_CASE("IMU simulator seeded stochastic terms are deterministic")
{
    ImuSimulatorConfig config;
    config.seed = 7U;
    config.gyro.output_random_walk_psd = Vec3{1.0e-6, 2.0e-6, 3.0e-6};
    config.accel.bias_inrun_psd = Vec3{1.0e-4, 2.0e-4, 3.0e-4};

    DefaultImuSimulator first(config);
    DefaultImuSimulator second(config);

    const auto previous = stationary_sample(0.0);
    const auto current = stationary_sample(1.0);
    ImuIncrement first_increment;
    ImuIncrement second_increment;
    REQUIRE(first.generate(previous, current, first_increment));
    REQUIRE(second.generate(previous, current, second_increment));

    CHECK(first_increment.delta_theta_ib_b_rad.isApprox(second_increment.delta_theta_ib_b_rad));
    CHECK(first_increment.delta_v_ib_b_mps.isApprox(second_increment.delta_v_ib_b_mps));
    CHECK(first.accel_bias_mps2().isApprox(second.accel_bias_mps2()));
}

TEST_CASE("IMU simulator applies first-order Gauss-Markov bias decay")
{
    ImuSimulatorConfig config;
    config.gyro.bias_turnon = Vec3{1.0, -2.0, 3.0};
    config.gyro.bias_correlation_rate_1ps = Vec3{0.5, 0.25, 0.0};
    config.gyro.bias_inrun_psd = Vec3::Zero();
    config.accel.bias_turnon = Vec3{4.0, -5.0, 6.0};
    config.accel.bias_correlation_rate_1ps = Vec3{1.0, 0.5, 0.0};
    config.accel.bias_inrun_psd = Vec3::Zero();

    DefaultImuSimulator simulator(config);
    ImuIncrement increment;
    REQUIRE(simulator.generate(stationary_sample(0.0), stationary_sample(2.0), increment));

    CHECK(simulator.gyro_bias_radps().x() == doctest::Approx(std::exp(-1.0)));
    CHECK(simulator.gyro_bias_radps().y() == doctest::Approx(-2.0 * std::exp(-0.5)));
    CHECK(simulator.gyro_bias_radps().z() == doctest::Approx(3.0));
    CHECK(simulator.accel_bias_mps2().x() == doctest::Approx(4.0 * std::exp(-2.0)));
    CHECK(simulator.accel_bias_mps2().y() == doctest::Approx(-5.0 * std::exp(-1.0)));
    CHECK(simulator.accel_bias_mps2().z() == doctest::Approx(6.0));
}

TEST_CASE("IMU runtime config parser accepts ideal and error-model shapes")
{
    using navkit::app_support::imu_simulator_config_from_json;
    using navkit::app_support::validate_imu_runtime_config;

    const nlohmann::json ideal =
        nlohmann::json{{"imu", {{"type", "ideal"}, {"rate_hz", 200.0}, {"seed", 11U}}}};
    CHECK_NOTHROW(validate_imu_runtime_config(ideal));
    CHECK(imu_simulator_config_from_json(ideal).seed == 11U);

    const nlohmann::json error_model = {
        {"imu",
         {{"type", "error_model"},
          {"seed", 12U},
          {"rate_hz", 200.0},
          {"gyro",
           {{"bias_turnon_radps", {1.0, 2.0, 3.0}},
            {"bias_inrun_psd_rad2ps3", {0.0, 0.0, 0.0}},
            {"bias_correlation_rate_1ps", {0.1, 0.2, 0.3}},
            {"angle_random_walk_psd_rad2ps", {1.0e-6, 2.0e-6, 3.0e-6}},
            {"scale_factor", {0.1, 0.2, 0.3}},
            {"misalignment_rad", {0.01, 0.02, 0.03}},
            {"nonorthogonality", {0.001, 0.002, 0.003}},
            {"angular_rate_limit_radps", {0.0, 0.0, 0.0}},
            {"quantization_rad", {0.0, 0.0, 0.0}}}},
          {"accel",
           {{"bias_turnon_mps2", {4.0, 5.0, 6.0}},
            {"bias_inrun_psd_m2ps5", {0.0, 0.0, 0.0}},
            {"bias_correlation_rate_1ps", {0.4, 0.5, 0.6}},
            {"velocity_random_walk_psd_m2ps3", {4.0e-6, 5.0e-6, 6.0e-6}},
            {"scale_factor", {0.4, 0.5, 0.6}},
            {"misalignment_rad", {0.04, 0.05, 0.06}},
            {"nonorthogonality", {0.004, 0.005, 0.006}},
            {"acceleration_limit_mps2", {0.0, 0.0, 0.0}},
            {"quantization_mps", {0.0, 0.0, 0.0}}}}}}};

    CHECK_NOTHROW(validate_imu_runtime_config(error_model));
    const auto parsed = imu_simulator_config_from_json(error_model);
    CHECK(parsed.seed == 12U);
    CHECK(parsed.gyro.bias_turnon.x() == doctest::Approx(1.0));
    CHECK(parsed.accel.bias_turnon.z() == doctest::Approx(6.0));
    CHECK(parsed.gyro.bias_correlation_rate_1ps.y() == doctest::Approx(0.2));
    CHECK(parsed.accel.bias_correlation_rate_1ps.z() == doctest::Approx(0.6));
    CHECK(parsed.gyro.limit.isZero());
    CHECK(parsed.accel.limit.isZero());
}

TEST_CASE("IMU simulator applies configured absolute rate and acceleration limits")
{
    ImuSimulatorConfig config;
    config.gyro.bias_turnon = Vec3{10.0, 0.0, 0.0};
    config.gyro.limit = Vec3{0.25, 0.0, 0.0};
    config.accel.bias_turnon = Vec3{0.0, -10.0, 0.0};
    config.accel.limit = Vec3{0.0, 0.5, 0.0};

    DefaultImuSimulator simulator(config);
    ImuIncrement increment;
    REQUIRE(simulator.generate(stationary_sample(0.0), stationary_sample(1.0), increment));

    CHECK(increment.delta_theta_ib_b_rad.x() == doctest::Approx(0.25));
    CHECK(increment.delta_v_ib_b_mps.y() == doctest::Approx(-0.5));
}

TEST_CASE("IMU runtime config parser accepts seeded variance and covariance draws")
{
    using navkit::app_support::imu_simulator_config_from_json;
    using navkit::app_support::validate_imu_runtime_config;

    const nlohmann::json random_error_model = {
        {"imu",
         {{"type", "error_model"},
          {"seed", 12U},
          {"rate_hz", 200.0},
          {"gyro",
           {{"bias_turnon_var_rad2ps2", {1.0, 1.0, 1.0}},
            {"bias_inrun_psd_rad2ps3", {0.0, 0.0, 0.0}},
            {"angle_random_walk_psd_rad2ps", {0.0, 0.0, 0.0}},
            {"scale_factor_cov", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
            {"misalignment_rad", {0.0, 0.0, 0.0}},
            {"nonorthogonality", {0.0, 0.0, 0.0}},
            {"quantization_rad", {0.0, 0.0, 0.0}}}},
          {"accel",
           {{"bias_turnon_var_m2ps4", {1.0, 1.0, 1.0}},
            {"bias_inrun_psd_m2ps5", {0.0, 0.0, 0.0}},
            {"velocity_random_walk_psd_m2ps3", {0.0, 0.0, 0.0}},
            {"scale_factor", {0.0, 0.0, 0.0}},
            {"misalignment_rad", {0.0, 0.0, 0.0}},
            {"nonorthogonality_cov", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
            {"quantization_mps", {0.0, 0.0, 0.0}}}}}}};

    CHECK_NOTHROW(validate_imu_runtime_config(random_error_model));
    const ImuSimulatorConfig first = imu_simulator_config_from_json(random_error_model);
    const ImuSimulatorConfig second = imu_simulator_config_from_json(random_error_model);

    CHECK(first.gyro.bias_turnon.isZero());
    CHECK(first.gyro.scale_factor.isZero());
    CHECK(first.accel.bias_turnon.isZero());
    CHECK(first.accel.nonorthogonality.isZero());
    CHECK(first.gyro_random.bias_turnon.enabled);
    CHECK(first.gyro_random.scale_factor.enabled);
    CHECK(first.accel_random.bias_turnon.enabled);
    CHECK(first.accel_random.nonorthogonality.enabled);
    CHECK(first.gyro_random.bias_turnon.covariance.isApprox(core::Mat3::Identity()));
    CHECK(first.gyro_random.scale_factor.covariance.isApprox(core::Mat3::Identity()));
    CHECK(first.accel_random.bias_turnon.covariance.isApprox(core::Mat3::Identity()));
    CHECK(first.accel_random.nonorthogonality.covariance.isApprox(core::Mat3::Identity()));

    CHECK(first.gyro_random.bias_turnon.covariance.isApprox(
        second.gyro_random.bias_turnon.covariance));
    CHECK(first.gyro_random.scale_factor.covariance.isApprox(
        second.gyro_random.scale_factor.covariance));
    CHECK(first.accel_random.bias_turnon.covariance.isApprox(
        second.accel_random.bias_turnon.covariance));
    CHECK(first.accel_random.nonorthogonality.covariance.isApprox(
        second.accel_random.nonorthogonality.covariance));

    const DefaultImuSimulator first_simulator(first);
    const DefaultImuSimulator second_simulator(second);
    CHECK(first_simulator.config().gyro.bias_turnon.isApprox(
        second_simulator.config().gyro.bias_turnon));
    CHECK(first_simulator.config().gyro.scale_factor.isApprox(
        second_simulator.config().gyro.scale_factor));
    CHECK(first_simulator.config().accel.bias_turnon.isApprox(
        second_simulator.config().accel.bias_turnon));
    CHECK(first_simulator.config().accel.nonorthogonality.isApprox(
        second_simulator.config().accel.nonorthogonality));
}

TEST_CASE("IMU runtime config parser normalizes diagonal variance to covariance")
{
    using navkit::app_support::imu_simulator_config_from_json;

    const nlohmann::json diagonal_config = {
        {"imu",
         {{"type", "error_model"},
          {"seed", 1U},
          {"rate_hz", 100.0},
          {"gyro", {{"bias_turnon_var_rad2ps2", {1.0, 2.0, 3.0}}}},
          {"accel", nlohmann::json::object()}}}};
    const nlohmann::json full_config = {
        {"imu",
         {{"type", "error_model"},
          {"seed", 1U},
          {"rate_hz", 100.0},
          {"gyro", {{"bias_turnon_cov_rad2ps2", {1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0}}}},
          {"accel", nlohmann::json::object()}}}};

    const ImuSimulatorConfig diagonal = imu_simulator_config_from_json(diagonal_config);
    const ImuSimulatorConfig full = imu_simulator_config_from_json(full_config);

    CHECK(diagonal.gyro_random.bias_turnon.enabled);
    CHECK(full.gyro_random.bias_turnon.enabled);
    CHECK(diagonal.gyro_random.bias_turnon.covariance.isApprox(
        full.gyro_random.bias_turnon.covariance));
}

TEST_CASE("IMU runtime config parser rejects malformed error-model inputs")
{
    using navkit::app_support::validate_imu_runtime_config;

    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{{"imu", {{"type", "mystery"}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{{"imu", {{"type", "error_model"}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{
                        {"imu",
                         {{"type", "error_model"},
                          {"gyro", {{"angle_random_walk_psd_rad2ps", {-1.0, 0.0, 0.0}}}},
                          {"accel", nlohmann::json::object()}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{
                        {"imu",
                         {{"type", "error_model"},
                          {"gyro", {{"bias_correlation_rate_1ps", {-0.1, 0.0, 0.0}}}},
                          {"accel", nlohmann::json::object()}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(
        validate_imu_runtime_config(nlohmann::json{{"imu",
                                                    {{"type", "error_model"},
                                                     {"gyro", {{"bias_turnon_radps", {1.0, 2.0}}}},
                                                     {"accel", nlohmann::json::object()}}}}),
        std::runtime_error);
    CHECK_THROWS_AS(
        validate_imu_runtime_config(nlohmann::json{{"imu",
                                                    {{"type", "error_model"},
                                                     {"seed", 1U},
                                                     {"rate_hz", 100.0},
                                                     {"gyro", {{"bias_radps", {0.0, 0.0, 0.0}}}},
                                                     {"accel", nlohmann::json::object()}}}}),
        std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(
                        nlohmann::json{{"imu",
                                        {{"type", "error_model"},
                                         {"seed", 1U},
                                         {"rate_hz", 100.0},
                                         {"gyro",
                                          {{"bias_turnon_radps", {0.0, 0.0, 0.0}},
                                           {"bias_turnon_var_rad2ps2", {1.0, 1.0, 1.0}}}},
                                         {"accel", nlohmann::json::object()}}}}),
                    std::runtime_error);
}

} // namespace navkit::sim::test
