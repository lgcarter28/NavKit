// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/emulation/concrete/ImuRuntimeConfig.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "test_main.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace navkit::sim::test
{

namespace
{

using navkit::core::Vec3;
using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;

[[nodiscard]] TruthSample stationary_sample(navkit::core::Time_t time_s)
{
    TruthSample sample;
    sample.time = time_s;
    sample.p_e = Vec3{Wgs84::a_m, 0.0, 0.0};
    sample.v_e.setZero();
    sample.q_b2e.setIdentity();
    return sample;
}

[[nodiscard]] bool approx_vec(const Vec3& actual, const Vec3& expected, double tolerance = 1.0e-12)
{
    return actual.isApprox(expected, tolerance);
}

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

    IdealImuInterval ideal;
    REQUIRE(ImuSimulator::ideal_interval_from_truth_ecef(previous, current, ideal));
    const auto gravity_e = J2<Wgs84>::acceleration(previous.p_e);

    CHECK(ideal.time_s == doctest::Approx(1.0));
    CHECK(ideal.dt_s == doctest::Approx(1.0));
    CHECK(ideal.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(ideal.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(ideal.delta_theta_ib_b_rad.z() == doctest::Approx(Wgs84::omega_rad_s));
    CHECK(approx_vec(ideal.delta_v_ib_b_mps, -gravity_e));
    CHECK(ideal.delta_v_ib_b_mps.x() > 0.0);
}

TEST_CASE("Ideal gyro truth combines ECEF attitude delta with Earth rotation")
{
    auto previous = stationary_sample(0.0);
    auto current = stationary_sample(2.0);
    current.q_b2e = Eigen::AngleAxisd(0.1, Vec3::UnitZ());

    IdealImuInterval ideal;
    REQUIRE(ImuSimulator::ideal_interval_from_truth_ecef(previous, current, ideal));

    CHECK(ideal.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(ideal.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(ideal.delta_theta_ib_b_rad.z() == doctest::Approx(0.1 + (2.0 * Wgs84::omega_rad_s)));
}

TEST_CASE("Ideal IMU truth interval rejects non-increasing timestamps without throwing")
{
    const auto previous = stationary_sample(1.0);
    const auto current = stationary_sample(1.0);
    IdealImuInterval ideal;

    CHECK_FALSE(ImuSimulator::ideal_interval_from_truth_ecef(previous, current, ideal));
    CHECK(ideal.time_s == doctest::Approx(0.0));
    CHECK(ideal.dt_s == doctest::Approx(0.0));
    CHECK(ideal.delta_theta_ib_b_rad.isZero());
    CHECK(ideal.delta_v_ib_b_mps.isZero());
}

TEST_CASE("IMU triad calibration applies scale, nonorthogonality, and misalignment ordering")
{
    ImuTriadErrorConfig config;
    config.scale_factor = Vec3{0.1, 0.0, 0.0};
    config.nonorthogonality = Vec3{0.2, 0.3, 0.0};

    const auto output = ImuSimulator::calibration_matrix_apply(Vec3{1.0, 0.0, 0.0}, config);

    CHECK(output.x() == doctest::Approx(1.1));
    CHECK(output.y() == doctest::Approx(0.2));
    CHECK(output.z() == doctest::Approx(0.3));
}

TEST_CASE("IMU simulator applies deterministic bias and quantization to raw increments")
{
    ImuSimulatorConfig config;
    config.gyro.bias = Vec3{0.26, 0.0, 0.0};
    config.gyro.quantization = Vec3{0.1, 0.0, 0.0};
    config.accel.bias = Vec3{0.0, 0.2, 0.0};

    ImuSimulator simulator(config);

    const auto previous = stationary_sample(0.0);
    const auto current = stationary_sample(1.0);
    IdealImuInterval ideal;
    REQUIRE(ImuSimulator::ideal_interval_from_truth_ecef(previous, current, ideal));
    ImuIncrement raw;
    REQUIRE(simulator.generate(previous, current, raw));

    CHECK(raw.time_s == doctest::Approx(current.time));
    CHECK(raw.dt_s == doctest::Approx(1.0));
    CHECK(raw.delta_theta_ib_b_rad.x() == doctest::Approx(0.3));
    CHECK(raw.delta_theta_ib_b_rad.z() == doctest::Approx(ideal.delta_theta_ib_b_rad.z()));
    CHECK(raw.delta_v_ib_b_mps.x() == doctest::Approx(ideal.delta_v_ib_b_mps.x()));
    CHECK(raw.delta_v_ib_b_mps.y() == doctest::Approx(0.2));
}

TEST_CASE("IMU simulator stateful generation consumes consecutive samples")
{
    ImuSimulator simulator;
    ImuIncrement increment;

    CHECK_FALSE(simulator.generate(stationary_sample(0.25), increment));
    CHECK(increment.dt_s == doctest::Approx(0.0));

    simulator.initialize(stationary_sample(0.0));
    REQUIRE(simulator.generate(stationary_sample(0.25), increment));

    IdealImuInterval ideal;
    REQUIRE(ImuSimulator::ideal_interval_from_truth_ecef(
        stationary_sample(0.0), stationary_sample(0.25), ideal));

    CHECK(increment.time_s == doctest::Approx(0.25));
    CHECK(increment.dt_s == doctest::Approx(0.25));
    CHECK(increment.delta_theta_ib_b_rad.x() == doctest::Approx(0.0));
    CHECK(increment.delta_theta_ib_b_rad.y() == doctest::Approx(0.0));
    CHECK(increment.delta_theta_ib_b_rad.z() == doctest::Approx(0.25 * Wgs84::omega_rad_s));
    CHECK(increment.delta_v_ib_b_mps.x() == doctest::Approx(ideal.delta_v_ib_b_mps.x()));
    CHECK(increment.delta_v_ib_b_mps.y() == doctest::Approx(0.0));
    CHECK(increment.delta_v_ib_b_mps.z() == doctest::Approx(0.0));

    CHECK_FALSE(simulator.generate(stationary_sample(0.25), increment));
}

TEST_CASE("IMU simulator seeded stochastic terms are deterministic")
{
    ImuSimulatorConfig config;
    config.seed = 7U;
    config.gyro.white_noise_psd = Vec3{1.0e-6, 2.0e-6, 3.0e-6};
    config.accel.bias_random_walk_psd = Vec3{1.0e-4, 2.0e-4, 3.0e-4};

    ImuSimulator first(config);
    ImuSimulator second(config);

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

TEST_CASE("IMU runtime config parser accepts ideal and error-model shapes")
{
    using navkit::app_support::imu_simulator_config_from_json;
    using navkit::app_support::validate_imu_runtime_config;

    const nlohmann::json ideal = {{"imu", {{"type", "ideal"}, {"seed", 11U}}}};
    CHECK_NOTHROW(validate_imu_runtime_config(ideal));
    CHECK(imu_simulator_config_from_json(ideal).seed == 11U);

    const nlohmann::json error_model = {{"imu",
                                         {{"type", "error_model"},
                                          {"seed", 12U},
                                          {"rate_hz", 200.0},
                                          {"gyro",
                                           {{"bias_radps", {1.0, 2.0, 3.0}},
                                            {"bias_rw_psd_rad2ps3", {0.0, 0.0, 0.0}},
                                            {"white_noise_psd_rad2ps", {1.0e-6, 2.0e-6, 3.0e-6}},
                                            {"scale_factor", {0.1, 0.2, 0.3}},
                                            {"misalignment_rad", {0.01, 0.02, 0.03}},
                                            {"nonorthogonality", {0.001, 0.002, 0.003}},
                                            {"quantization_rad", {0.0, 0.0, 0.0}}}},
                                          {"accel",
                                           {{"bias_mps2", {4.0, 5.0, 6.0}},
                                            {"bias_rw_psd_m2ps5", {0.0, 0.0, 0.0}},
                                            {"white_noise_psd_m2ps3", {4.0e-6, 5.0e-6, 6.0e-6}},
                                            {"scale_factor", {0.4, 0.5, 0.6}},
                                            {"misalignment_rad", {0.04, 0.05, 0.06}},
                                            {"nonorthogonality", {0.004, 0.005, 0.006}},
                                            {"quantization_mps", {0.0, 0.0, 0.0}}}}}}};

    CHECK_NOTHROW(validate_imu_runtime_config(error_model));
    const auto parsed = imu_simulator_config_from_json(error_model);
    CHECK(parsed.seed == 12U);
    CHECK(parsed.gyro.bias.x() == doctest::Approx(1.0));
    CHECK(parsed.accel.bias.z() == doctest::Approx(6.0));
}

TEST_CASE("IMU runtime config parser rejects malformed error-model inputs")
{
    using navkit::app_support::validate_imu_runtime_config;

    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{{"imu", {{"type", "mystery"}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(nlohmann::json{{"imu", {{"type", "error_model"}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(validate_imu_runtime_config(
                        nlohmann::json{{"imu",
                                        {{"type", "error_model"},
                                         {"gyro", {{"white_noise_psd_rad2ps", {-1.0, 0.0, 0.0}}}},
                                         {"accel", nlohmann::json::object()}}}}),
                    std::runtime_error);
    CHECK_THROWS_AS(
        validate_imu_runtime_config(nlohmann::json{{"imu",
                                                    {{"type", "error_model"},
                                                     {"gyro", {{"bias_radps", {1.0, 2.0}}}},
                                                     {"accel", nlohmann::json::object()}}}}),
        std::runtime_error);
}

} // namespace navkit::sim::test
