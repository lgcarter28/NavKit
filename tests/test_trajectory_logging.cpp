// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/logging/TrajectoryLogDataBuilder.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/io/LogProductPolicy.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/io/log_payloads/TrajectoryLogPayload.hpp"
#include "navkit/io/log_products/TrajectoryAutopilotVehicleLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryBodyLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryEcefLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryEciLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryGuidanceLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryNedLogProduct.hpp"
#include "navkit/sim/trajectory/TrajectoryDynamics.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <string>

namespace navkit::app_support::test
{

namespace
{

using TrajectoryLogger = io::RunLogger<io::TrajectoryEcefLogProduct,
                                       io::TrajectoryEciLogProduct,
                                       io::TrajectoryNedLogProduct,
                                       io::TrajectoryBodyLogProduct,
                                       io::TrajectoryGuidanceLogProduct,
                                       io::TrajectoryAutopilotVehicleLogProduct>;

} // namespace

TEST_CASE("trajectory log products expose distinct typed payload boundaries")
{
    static_assert(io::LogProductPolicy<io::TrajectoryEcefLogProduct, io::TrajectoryEcefLogPayload>);
    static_assert(io::LogProductPolicy<io::TrajectoryEciLogProduct, io::TrajectoryEciLogPayload>);
    static_assert(io::LogProductPolicy<io::TrajectoryNedLogProduct, io::TrajectoryNedLogPayload>);
    static_assert(io::LogProductPolicy<io::TrajectoryBodyLogProduct, io::TrajectoryBodyLogPayload>);
    static_assert(
        io::LogProductPolicy<io::TrajectoryGuidanceLogProduct, io::TrajectoryGuidanceLogPayload>);
    static_assert(io::LogProductPolicy<io::TrajectoryAutopilotVehicleLogProduct,
                                       io::TrajectoryAutopilotVehicleLogPayload>);
    static_assert(TrajectoryLogger::matching_product_count_v<io::TrajectoryEcefLogPayload> == 1U);
    static_assert(TrajectoryLogger::matching_product_count_v<io::TrajectoryEciLogPayload> == 1U);
    static_assert(TrajectoryLogger::matching_product_count_v<io::TrajectoryNedLogPayload> == 1U);
    static_assert(TrajectoryLogger::matching_product_count_v<io::TrajectoryBodyLogPayload> == 1U);
    static_assert(TrajectoryLogger::matching_product_count_v<io::TrajectoryGuidanceLogPayload> ==
                  1U);
    static_assert(
        TrajectoryLogger::matching_product_count_v<io::TrajectoryAutopilotVehicleLogPayload> == 1U);
    CHECK(true);
}

TEST_CASE("trajectory log builder resolves stationary truth into all inspection frames")
{
    using Planet = core::environment::Wgs84;

    sim::TruthSample truth{};
    const core::Timestamp source_epoch{};
    truth.p_e = core::Vec3{Planet::a_m, 0.0, 0.0};
    truth.v_e = core::Vec3::Zero();
    truth.q_b2e = Eigen::Quaternion<core::Scalar_t>::Identity();

    sim::TrajectoryDiagnostics diagnostics{};
    diagnostics.p_i_m = truth.p_e;
    diagnostics.q_b2i = truth.q_b2e;
    diagnostics.w_ib_b_radps = core::environment::planet_rate_fixed_radps<Planet>();
    diagnostics.specific_force_ib_b_mps2 = core::Vec3{0.5, 1.0, 1.5};
    diagnostics.guidance_acceleration_command_b_mps2 = core::Vec3{1.0, 2.0, 3.0};
    diagnostics.guidance_acceleration_response_b_mps2 = core::Vec3{0.75, 1.5, 2.25};
    diagnostics.guidance_state_index = 4U;
    diagnostics.autopilot_q_command_b2i = truth.q_b2e;
    diagnostics.autopilot_q_response_b2i = truth.q_b2e;
    diagnostics.autopilot_angular_rate_feedforward_b_radps = core::Vec3{0.1, 0.2, 0.3};
    diagnostics.vehicle_specific_force_command_b_mps2 = core::Vec3{1.0, 2.0, 3.0};
    diagnostics.vehicle_specific_force_response_b_mps2 = core::Vec3{0.5, 1.0, 1.5};
    diagnostics.velocity_tracking_error_b_mps = core::Vec3{4.0, 5.0, 6.0};
    diagnostics.acceleration_tracking_error_b_mps2 = core::Vec3{7.0, 8.0, 9.0};
    diagnostics.attitude_tracking_error_b_rad = core::Vec3{0.1, 0.2, 0.3};
    diagnostics.angular_rate_tracking_error_b_radps = core::Vec3{0.4, 0.5, 0.6};
    diagnostics.specific_force_tracking_error_b_mps2 = core::Vec3{0.5, 1.0, 1.5};

    io::TrajectoryLogData data{};
    REQUIRE(trajectory_log_data_from_truth<Planet>(truth, diagnostics, source_epoch, data));
    CHECK(data.p_lla_deg_m.x() == doctest::Approx(0.0));
    CHECK(data.p_lla_deg_m.y() == doctest::Approx(0.0));
    CHECK(data.p_lla_deg_m.z() == doctest::Approx(0.0));
    CHECK(data.w_eb_b_radps.norm() == doctest::Approx(0.0));
    CHECK(data.w_nb_b_radps.norm() == doctest::Approx(0.0));
    CHECK(data.q_b2e.norm() == doctest::Approx(1.0));
    CHECK(data.q_b2i.norm() == doctest::Approx(1.0));
    CHECK(data.q_b2n.norm() == doctest::Approx(1.0));
    CHECK(data.v_ib_b_mps == diagnostics.v_i_mps);
    CHECK(data.a_ib_b_mps2 == diagnostics.a_i_mps2);
    CHECK(data.v_eb_b_mps.isZero());
    CHECK(data.guidance_acceleration_command_b_mps2 ==
          diagnostics.guidance_acceleration_command_b_mps2);
    CHECK(data.guidance_state_index == diagnostics.guidance_state_index);
    CHECK(data.autopilot_angular_rate_feedforward_b_radps ==
          diagnostics.autopilot_angular_rate_feedforward_b_radps);
    CHECK(data.velocity_tracking_error_b_mps == diagnostics.velocity_tracking_error_b_mps);
    CHECK(data.acceleration_tracking_error_b_mps2 ==
          diagnostics.acceleration_tracking_error_b_mps2);
    CHECK(data.attitude_tracking_error_b_rad == diagnostics.attitude_tracking_error_b_rad);
    CHECK(data.angular_rate_tracking_error_b_radps ==
          diagnostics.angular_rate_tracking_error_b_radps);
    CHECK(data.specific_force_tracking_error_b_mps2 ==
          diagnostics.specific_force_tracking_error_b_mps2);
}

TEST_CASE("trajectory log builder resolves Earth orientation from a nonzero source epoch")
{
    using Planet = core::environment::Wgs84;

    core::Timestamp source_epoch{};
    core::Timestamp truth_time{};
    REQUIRE(core::timestamp_from_seconds(100.0, core::TimeScale::Monotonic, source_epoch));
    REQUIRE(core::timestamp_from_seconds(101.0, core::TimeScale::Monotonic, truth_time));

    sim::TruthSample truth{};
    truth.t = truth_time;
    truth.p_e = core::Vec3{Planet::a_m, 0.0, 0.0};
    truth.v_e = core::Vec3{1.0, 2.0, 3.0};
    truth.q_b2e = Eigen::Quaternion<core::Scalar_t>::Identity();

    sim::TrajectoryDiagnostics diagnostics{};
    diagnostics.p_i_m = core::Vec3{Planet::a_m, 1000.0, 500.0};
    diagnostics.v_i_mps = core::Vec3{25.0, 150.0, -2.0};
    diagnostics.a_i_mps2 = core::Vec3{1.0, -0.5, 0.25};
    diagnostics.q_b2i = truth.q_b2e;
    diagnostics.autopilot_q_command_b2i = truth.q_b2e;
    diagnostics.autopilot_q_response_b2i = truth.q_b2e;

    core::Vec3 expected_a_e_mps2{};
    core::Vec3 absolute_time_a_e_mps2{};
    REQUIRE(core::frames::inertial_to_fixed_acceleration<Planet>(
        diagnostics.p_i_m, diagnostics.v_i_mps, diagnostics.a_i_mps2, 1.0, expected_a_e_mps2));
    REQUIRE(core::frames::inertial_to_fixed_acceleration<Planet>(diagnostics.p_i_m,
                                                                 diagnostics.v_i_mps,
                                                                 diagnostics.a_i_mps2,
                                                                 101.0,
                                                                 absolute_time_a_e_mps2));

    io::TrajectoryLogData data{};
    REQUIRE(trajectory_log_data_from_truth<Planet>(truth, diagnostics, source_epoch, data));
    CHECK((data.a_e_mps2 - expected_a_e_mps2).norm() < 1.0e-12);
    CHECK((data.a_e_mps2 - absolute_time_a_e_mps2).norm() > 1.0e-4);
}

TEST_CASE("trajectory body log resolves inertial kinematics with the body-to-ECI attitude")
{
    using Planet = core::environment::Wgs84;

    sim::TruthSample truth{};
    truth.p_e = core::Vec3{Planet::a_m, 0.0, 0.0};
    truth.q_b2e = Eigen::Quaternion<core::Scalar_t>::Identity();

    sim::TrajectoryDiagnostics diagnostics{};
    diagnostics.p_i_m = truth.p_e;
    diagnostics.v_i_mps = core::Vec3{5.0, 2.0, -1.0};
    diagnostics.a_i_mps2 = core::Vec3{-3.0, 4.0, 0.5};
    diagnostics.q_b2i = Eigen::Quaternion<core::Scalar_t>{Eigen::AngleAxis<core::Scalar_t>{
        0.5 * std::numbers::pi_v<core::Scalar_t>, core::Vec3::UnitZ()}};

    io::TrajectoryLogData data{};
    REQUIRE(trajectory_log_data_from_truth<Planet>(truth, diagnostics, core::Timestamp{}, data));
    CHECK((data.v_ib_b_mps - diagnostics.q_b2i.conjugate() * diagnostics.v_i_mps).norm() < 1.0e-12);
    CHECK((data.a_ib_b_mps2 - diagnostics.q_b2i.conjugate() * diagnostics.a_i_mps2).norm() <
          1.0e-12);
    CHECK((data.v_eb_b_mps - truth.q_b2e.conjugate() * truth.v_e).norm() < 1.0e-12);
    CHECK((data.a_eb_b_mps2 - truth.q_b2e.conjugate() * data.a_e_mps2).norm() < 1.0e-12);
}

TEST_CASE("trajectory logger writes frame-specific CSV and manifest products")
{
    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "navkit_trajectory_logger_test";
    std::filesystem::remove_all(output_dir);

    io::TrajectoryLogData data{};
    data.t.s = 1;
    {
        TrajectoryLogger logger(output_dir, "trajectory_logger_test", nlohmann::json::object());
        logger.log(io::TrajectoryEcefLogPayload{.data = data});
        logger.log(io::TrajectoryEciLogPayload{.data = data});
        logger.log(io::TrajectoryNedLogPayload{.data = data});
        logger.log(io::TrajectoryBodyLogPayload{.data = data});
        logger.log(io::TrajectoryGuidanceLogPayload{.data = data});
        logger.log(io::TrajectoryAutopilotVehicleLogPayload{.data = data});
        logger.close();
    }

    CHECK(std::filesystem::exists(output_dir / "trajectory_kinematics_ecef.csv"));
    CHECK(std::filesystem::exists(output_dir / "trajectory_kinematics_eci.csv"));
    CHECK(std::filesystem::exists(output_dir / "trajectory_kinematics_ned.csv"));
    CHECK(std::filesystem::exists(output_dir / "trajectory_kinematics_body.csv"));
    CHECK(std::filesystem::exists(output_dir / "trajectory_guidance.csv"));
    CHECK(std::filesystem::exists(output_dir / "trajectory_autopilot_vehicle.csv"));
    CHECK(std::filesystem::exists(output_dir / "run_manifest.json"));

    {
        std::ifstream eci_file(output_dir / "trajectory_kinematics_eci.csv");
        std::string header{};
        std::getline(eci_file, header);
        CHECK(header.find("a_i_x_mps2") != std::string::npos);
        CHECK(header.find("w_ib_b_z_radps") != std::string::npos);
    }
    {
        std::ifstream body_file(output_dir / "trajectory_kinematics_body.csv");
        std::string header{};
        std::getline(body_file, header);
        CHECK(header.find("v_ib_b_x_mps") != std::string::npos);
        CHECK(header.find("a_ib_b_z_mps2") != std::string::npos);
        CHECK(header.find("v_eb_b_x_mps") != std::string::npos);
        CHECK(header.find("a_eb_b_z_mps2") != std::string::npos);
        CHECK(header.find("w_eb_b_y_radps") != std::string::npos);
    }
    {
        std::ifstream guidance_file(output_dir / "trajectory_guidance.csv");
        std::string header{};
        std::getline(guidance_file, header);
        CHECK(header.find("guidance_acceleration_command_b_x_mps2") != std::string::npos);
        CHECK(header.find("guidance_bank_response_n_rad") != std::string::npos);
        CHECK(header.find("guidance_reference_index") != std::string::npos);
        CHECK(header.find("guidance_reference_position_valid") != std::string::npos);
        CHECK(header.find("guidance_state_index") != std::string::npos);
        CHECK(header.find("guidance_mode") == std::string::npos);
    }
    {
        std::ifstream response_file(output_dir / "trajectory_autopilot_vehicle.csv");
        std::string header{};
        std::getline(response_file, header);
        CHECK(header.find("vehicle_specific_force_command_b_x_mps2") != std::string::npos);
        CHECK(header.find("autopilot_angular_rate_feedforward_b_z_radps") != std::string::npos);
        CHECK(header.find("velocity_tracking_error_b_x_mps") != std::string::npos);
        CHECK(header.find("attitude_tracking_error_b_z_rad") != std::string::npos);
        CHECK(header.find("specific_force_tracking_error_b_z_mps2") != std::string::npos);
    }

    std::filesystem::remove_all(output_dir);
}

} // namespace navkit::app_support::test
