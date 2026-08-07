// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/app_support/emulation/EmulatorPolicy.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
#include "navkit/app_support/initialization/CovarianceFloorJson.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/initialization/NavInitializationProviderPolicy.hpp"
#include "navkit/app_support/initialization/TransferAlignmentProviderPolicy.hpp"
#include "navkit/app_support/logging/RuntimeLogger.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/sensors/ImuSimulatorPolicy.hpp"
#include "test_main.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <numbers>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

using EcefInsGnssAppConfig =
    navkit::config::apps::navkit_sim::EcefInsGnssLcGyroAccelBiasDefaultConfig;
using AppRuntimeLogger = RuntimeLogger<EcefInsGnssAppConfig::NavKit>;

struct DuplicateSensorIdConfig
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using ImuSimulator = EcefInsGnssAppConfig::ImuSimulator;
    using EmulatorBindings =
        std::tuple<navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<0U>,
                                                        NavKit::PrimaryGnssPositionSensor>,
                   navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<0U>,
                                                        NavKit::PrimaryGnssPositionSensor>>;
};

struct UnknownSensor
{
    static constexpr SensorId Id = 99U;
};

struct MissingTargetSensorConfig
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using ImuSimulator = EcefInsGnssAppConfig::ImuSimulator;
    using EmulatorBindings =
        std::tuple<navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<99U>,
                                                        UnknownSensor>>;
};

struct NotAnEmulator
{};

struct NotABinding
{};

[[nodiscard]] nlohmann::json valid_ecef_ins_gnss_runtime_config()
{
    return {
        {"run_name", "ecef_ins_gnss_demo"},
        {"output_dir", "output/logs/ecef_ins_gnss_demo"},
        {"application",
         {{"clock", "simulated"},
          {"control_state_source", "navigation_estimate"},
          {"rate_hz", 1000.0}}},
        {"logging",
         {{"console", {{"enabled", true}, {"rate_hz", 1.0}}},
          {"truth", {{"enabled", true}, {"rate_hz", 10.0}}},
          {"nav_estimate", {{"enabled", true}, {"rate_hz", 10.0}, {"covariance", "triangular"}}},
          {"measurement_statistics", {{"enabled", true}, {"rate_hz", 1.0}}},
          {"imu", {{"enabled", true}, {"rate_hz", 100.0}}},
          {"imu_debug", {{"enabled", true}, {"rate_hz", 10.0}}},
          {"filter_correction",
           {{"enabled", true}, {"rate_hz", 10.0}, {"covariance", "diagonal"}}}}},
        {"trajectory",
         {{"type", "stationary"},
          {"duration_s", 60.0},
          {"dynamics_rate_hz", 1000.0},
          {"p_lla_deg_m", {0.0, 0.0, 0.0}},
          {"v_n_mps", {0.0, 0.0, 0.0}},
          {"rpy_b2n_deg", {0.0, 0.0, 0.0}},
          {"w_nb_b_degps", {0.0, 0.0, 0.0}}}},
        {"imu", {{"type", "ideal"}, {"rate_hz", 1000.0}, {"seed", 42U}}},
        {"gnss",
         {{"dt_s", 1.0},
          {"position_cov", {{"frame", "ned"}, {"diag", {{"pos_m2", {9.0, 9.0, 25.0}}}}}},
          {"velocity_cov", {{"frame", "ned"}, {"diag", {{"vel_m2ps2", {0.04, 0.04, 0.04}}}}}},
          {"chi_square_acceptance",
           {{"position", {{"enabled", true}, {"probability", 0.9973}}},
            {"velocity", {{"enabled", true}, {"probability", 0.9973}}}}},
          {"p_b_ant_b_m", {1.0, 0.25, -0.15}},
          {"seed", 42U},
          {"noise_enabled", true}}},
        {"pva_initialization",
         {{"type", "pva_random_error"},
          {"seed", 7U},
          {"pva_error_frame", "ned"},
          {"pva_error_cov",
           {{"diag",
             {10000.0,
              10000.0,
              10000.0,
              100.0,
              100.0,
              100.0,
              0.007615435494667714,
              0.007615435494667714,
              0.030461741978670857}}}}}}};
}

TEST_CASE("Runtime JSON clock parser accepts only supported app-support modes")
{
    const nlohmann::json simulated{{"clock", "simulated"}};
    const nlohmann::json realtime{{"clock", "realtime"}};
    const nlohmann::json invalid{{"clock", "invalid"}};
    ClockMode mode{};

    REQUIRE(detail::clock_mode_from_json(simulated, "clock", mode));
    CHECK(mode == ClockMode::Simulated);
    REQUIRE(detail::clock_mode_from_json(realtime, "clock", mode));
    CHECK(mode == ClockMode::Realtime);
    CHECK_FALSE(detail::clock_mode_from_json(invalid, "clock", mode));
    CHECK_FALSE(detail::clock_mode_from_json(simulated, "missing", mode));
}

TEST_CASE("Runtime control-state source parser accepts only explicit source selections")
{
    ControlStateSourceMode mode{};

    REQUIRE(control_state_source_mode_from_string("navigation_estimate", mode));
    CHECK(mode == ControlStateSourceMode::NavigationEstimate);
    REQUIRE(control_state_source_mode_from_string("truth_passthrough", mode));
    CHECK(mode == ControlStateSourceMode::TruthPassthrough);
    CHECK_FALSE(control_state_source_mode_from_string("truth", mode));
}

[[nodiscard]] std::vector<double> identity_pva_cov_full()
{
    std::vector<double> values(81U, 0.0);
    for (std::size_t i = 0; i < 9U; ++i) {
        values.at((i * 9U) + i) = 1.0;
    }
    return values;
}

[[nodiscard]] std::vector<double> identity_vec3_cov_full()
{
    std::vector<double> values(9U, 0.0);
    for (std::size_t i = 0; i < 3U; ++i) {
        values.at((i * 3U) + i) = 1.0;
    }
    return values;
}

[[nodiscard]] std::vector<double> identity_initial_covariance_full()
{
    std::vector<double> values(225U, 0.0);
    for (std::size_t i = 0; i < 15U; ++i) {
        values.at((i * 15U) + i) = 1.0;
    }
    return values;
}

[[nodiscard]] nlohmann::json runtime_propagation_json()
{
    return nlohmann::json{{"process_noise",
                           {{"gyro_white_noise_psd_rad2ps", {1.0e-11, 1.0e-11, 1.0e-11}},
                            {"accel_white_noise_psd_m2ps3", {1.0e-7, 1.0e-7, 1.0e-7}},
                            {"gyro_bias_drive_psd_rad2ps3", {1.0e-14, 1.0e-14, 1.0e-14}},
                            {"accel_bias_drive_psd_m2ps5", {1.0e-10, 1.0e-10, 1.0e-10}}}},
                          {"imu_bias_dynamics",
                           {{"gyro_bias_correlation_rate_1ps", {0.25, 0.25, 0.25}},
                            {"accel_bias_correlation_rate_1ps", {0.5, 0.5, 0.5}}}}};
}

[[nodiscard]] nlohmann::json random_pva_runtime_config()
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization").at("pva_error_cov") = {{"full", identity_pva_cov_full()}};
    cfg.at("pva_initialization").at("pva_error_frame") = "ecef";
    return cfg;
}

[[nodiscard]] nlohmann::json explicit_pva_runtime_config()
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization") = {{"type", "pva_error"},
                                    {"pva_error",
                                     {{"p_n_m", {25.0, -15.0, 10.0}},
                                      {"v_n_mps", {0.0, 0.0, 0.0}},
                                      {"rotvec_b2n_deg", {0.0, 0.0, 0.0}}}}};
    return cfg;
}

[[nodiscard]] nlohmann::json valid_guidance_state(const std::string& id)
{
    return {
        {"id", id},
        {"plant", {{"constraint", "none"}}},
        {"guidance",
         {{"enabled", true},
          {"translation",
           {{"reference", {{"type", "current_state"}}},
            {"acceleration",
             nlohmann::json::array({{{"type", "body_specific_force"},
                                     {"specific_force_ib_b_mps2", {0.0, 0.0, 0.0}}}})}}},
          {"bank", {{"type", "zero"}}},
          {"body_y_specific_force_enabled", true}}},
        {"autopilot", {{"enabled", true}}},
        {"terminal", {{"behavior", "run_until_trajectory_termination"}}},
    };
}

[[nodiscard]] nlohmann::json valid_state_machine_runtime_config()
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory") = {
        {"type", "state_machine"},
        {"duration_s", 10.0},
        {"dynamics_rate_hz", 1000.0},
        {"guidance_rate_hz", 100.0},
        {"autopilot_rate_hz", 500.0},
        {"translational_integration", "trapezoidal_predictor_corrector"},
        {"termination", {{"type", "configured_duration"}}},
        {"guidance_command_filter",
         {{"specific_force_time_constant_b_s", {0.2, 0.3, 0.4}}, {"bank_time_constant_s", 0.5}}},
        {"autopilot",
         {{"type", "first_order"},
          {"controller_rate_time_constant_pqr_s", {0.0, 0.0, 0.0}},
          {"attitude_command_time_constant_s", 0.1},
          {"attitude_error_gain_pqr_per_s", {1.0, 1.0, 1.0}},
          {"angular_rate_feedback_gain_pqr", {0.0, 0.0, 0.0}},
          {"velocity_alignment_speed_threshold_mps", 1.0},
          {"initial_velocity_alignment_tolerance_deg", 5.0},
          {"gyro_moving_average_window_samples", 10U}}},
        {"maximum_bank_angle_deg", 45.0},
        {"vehicle_response",
         {{"type", "first_order"},
          {"vehicle_rate_time_constant_pqr_s", {0.0, 0.0, 0.0}},
          {"specific_force_command_time_constant_b_s", {0.0, 0.0, 0.0}},
          {"specific_force_response_time_constant_b_s", {0.0, 0.0, 0.0}}}},
        {"state_machine",
         {{"initial_state_id", "active"},
          {"cycle_policy", "reject"},
          {"states", nlohmann::json::array({valid_guidance_state("active")})}}},
        {"p_lla_deg_m", {35.0, -106.0, 1500.0}},
        {"v_n_mps", {100.0, 0.0, 0.0}},
        {"rpy_b2n_deg", {0.0, 0.0, 0.0}},
    };
    return cfg;
}

void make_guidance_state_nonterminal(nlohmann::json& state,
                                     const std::string& target,
                                     const std::size_t priority = 0U)
{
    state.erase("terminal");
    state.emplace("transitions",
                  nlohmann::json::array(
                      {{{"to", target},
                        {"priority", priority},
                        {"when", {{"type", "elapsed_in_state"}, {"greater_equal_s", 1.0}}}}}));
}

} // namespace

TEST_CASE("ECEF INS GNSS runtime validator accepts the documented input shape")
{
    const nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();

    static_assert(SimulationAppConfigPolicy<EcefInsGnssAppConfig>);
    static_assert(navkit::sim::ImuSimulatorPolicy<EcefInsGnssAppConfig::ImuSimulator>);
    static_assert(!SimulationAppConfigPolicy<DuplicateSensorIdConfig>);
    static_assert(!SimulationAppConfigPolicy<MissingTargetSensorConfig>);
    static_assert(EmulatorPolicy<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator,
                                 EcefInsGnssAppConfig::PrimaryGnssPositionSensor,
                                 AppRuntimeLogger>);
    static_assert(!EmulatorPolicy<NotAnEmulator,
                                  EcefInsGnssAppConfig::PrimaryGnssPositionSensor,
                                  AppRuntimeLogger>);
    static_assert(
        EmulatorBindingPolicy<EcefInsGnssAppConfig::PrimaryGnssPositionBinding, AppRuntimeLogger>);
    static_assert(!EmulatorBindingPolicy<NotABinding, AppRuntimeLogger>);
    static_assert(EmulatorBindingTuplePolicy<EcefInsGnssAppConfig::EmulatorBindings,
                                             EcefInsGnssAppConfig::NavKit::Sensors,
                                             AppRuntimeLogger>);
    static_assert(NavInitializationProviderPolicy<EcefInsGnssAppConfig::NavInitializationProvider>);
    static_assert(TransferAlignmentProviderPolicy<EcefInsGnssAppConfig::TransferAlignmentProvider,
                                                  EcefInsGnssAppConfig::NavKit::Navigator>);
    static_assert(navkit::core::estimation::InitialCovarianceConfigPolicy<
                  EcefInsGnssAppConfig::NavKit::InitialCovariance,
                  EcefInsGnssAppConfig::NavKit::StateDef>);
    static_assert(navkit::core::estimation::CovarianceFloorConfigPolicy<
                  EcefInsGnssAppConfig::NavKit::CovarianceFloor,
                  EcefInsGnssAppConfig::NavKit::StateDef>);
    static_assert(navkit::core::estimation::ImuProcessNoiseConfigPolicy<
                  EcefInsGnssAppConfig::NavKit::PropagationConfig::ProcessNoise>);
    static_assert(navkit::core::estimation::ImuBiasDynamicsConfigPolicy<
                  EcefInsGnssAppConfig::NavKit::PropagationConfig::ImuBiasDynamics>);
    static_assert(!EmulatorBindingTuplePolicy<DuplicateSensorIdConfig::EmulatorBindings,
                                              DuplicateSensorIdConfig::NavKit::Sensors,
                                              RuntimeLogger<DuplicateSensorIdConfig::NavKit>>);
    static_assert(!EmulatorBindingTuplePolicy<MissingTargetSensorConfig::EmulatorBindings,
                                              MissingTargetSensorConfig::NavKit::Sensors,
                                              RuntimeLogger<MissingTargetSensorConfig::NavKit>>);
    static_assert(emulator_binding_ids_unique_v<EcefInsGnssAppConfig::EmulatorBindings>);
    static_assert(
        std::is_same_v<EmulatorFromId_t<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::Id,
                                        EcefInsGnssAppConfig::EmulatorBindings>,
                       GnssEmulator<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::Id>>);

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
    const navkit::sim::GnssSimulatorConfig gnss =
        detail::gnss_runtime_config_from_json(cfg, "gnss");
    CHECK(gnss.p_b_ant_b_m.isApprox(core::Vec3{1.0, 0.25, -0.15}));

    const navkit::sim::GnssSimulatorConfig position_gnss =
        EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::runtime_config_from_json(cfg);
    const navkit::sim::GnssSimulatorConfig velocity_gnss =
        EcefInsGnssAppConfig::PrimaryGnssVelocityEmulator::runtime_config_from_json(cfg);
    CHECK(position_gnss.seed ==
          navkit::sim::derive_random_stream_seed(
              gnss.seed, static_cast<std::uint32_t>(detail::GnssRandomStream::Position)));
    CHECK(velocity_gnss.seed ==
          navkit::sim::derive_random_stream_seed(
              gnss.seed, static_cast<std::uint32_t>(detail::GnssRandomStream::Velocity)));
    CHECK(position_gnss.seed != velocity_gnss.seed);
    CHECK(position_gnss.seed ==
          EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::runtime_config_from_json(cfg).seed);
    CHECK(velocity_gnss.seed ==
          EcefInsGnssAppConfig::PrimaryGnssVelocityEmulator::runtime_config_from_json(cfg).seed);

    EcefInsGnssAppConfig::PrimaryGnssPositionSensor position_sensor{};
    EcefInsGnssAppConfig::PrimaryGnssVelocitySensor velocity_sensor{};
    EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::configure_sensor(position_sensor, cfg);
    EcefInsGnssAppConfig::PrimaryGnssVelocityEmulator::configure_sensor(velocity_sensor, cfg);
    CHECK(position_sensor.innovation_gate().enabled());
    CHECK(position_sensor.innovation_gate().probability() == doctest::Approx(0.9973));
    CHECK(EcefInsGnssAppConfig::PrimaryGnssPositionSensor::InnovationGate_t::dof == 3U);
    CHECK(position_sensor.innovation_gate().threshold() > 0.0);
    CHECK(velocity_sensor.innovation_gate().enabled());
    CHECK(velocity_sensor.innovation_gate().probability() == doctest::Approx(0.9973));
    CHECK(EcefInsGnssAppConfig::PrimaryGnssVelocitySensor::InnovationGate_t::dof == 3U);
    CHECK(velocity_sensor.innovation_gate().threshold() > 0.0);
}

TEST_CASE("Default compile-time attitude covariance is symmetric in ECEF")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using Error = NavKit::StateDef::Error;

    const NavKit::InitialCovariance::InitialCovariance_t& covariance =
        NavKit::InitialCovariance::initial_covariance;
    const core::Scalar_t attitude_variance = covariance(Error::AttRotVec::i, Error::AttRotVec::i);

    CHECK(attitude_variance > 0.0);
    CHECK(covariance(Error::AttRotVec::i + 1, Error::AttRotVec::i + 1) ==
          doctest::Approx(attitude_variance));
    CHECK(covariance(Error::AttRotVec::i + 2, Error::AttRotVec::i + 2) ==
          doctest::Approx(attitude_variance));
}

TEST_CASE("ECEF INS GNSS runtime validator accepts a CSV trajectory source")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory") = {{"type", "csv"}, {"csv_path", "truth/example.csv"}};
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("trajectory").erase("csv_path");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts a Guidance state-machine trajectory")
{
    nlohmann::json cfg = valid_state_machine_runtime_config();
    const sim::StateMachineTrajectoryConfig inherited_filter_trajectory =
        state_machine_trajectory_config_from_json(cfg);
    REQUIRE(inherited_filter_trajectory.state_machine.states.size() == 1U);
    CHECK(inherited_filter_trajectory.state_machine.states.front()
              .guidance_command_filter.specific_force_time_constant_b_s.isApprox(
                  core::Vec3{0.2, 0.3, 0.4}));
    CHECK(inherited_filter_trajectory.state_machine.states.front()
              .guidance_command_filter.bank_time_constant_s == doctest::Approx(0.5));

    nlohmann::json& configured_state = cfg.at("trajectory").at("state_machine").at("states").at(0U);
    configured_state.emplace("guidance_command_filter",
                             nlohmann::json{{"specific_force_time_constant_b_s", {0.6, 0.7, 0.8}},
                                            {"bank_time_constant_s", 0.9}});
    configured_state.emplace("on_entry",
                             nlohmann::json{{"guidance_command_filter",
                                             {{"specific_force_time_constant_b_s", {1.0, 1.1, 1.2}},
                                              {"bank_time_constant_s", 1.3},
                                              {"duration_s", 2.0}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
    const sim::StateMachineTrajectoryConfig trajectory =
        state_machine_trajectory_config_from_json(cfg);
    REQUIRE(trajectory.state_machine.states.size() == 1U);
    CHECK(trajectory.state_machine.initial_state_id == "active");
    CHECK(trajectory.state_machine.states.front().id == "active");
    CHECK(trajectory.state_machine.states.front().terminal);
    CHECK(trajectory.state_machine.states.front()
              .guidance_command_filter.specific_force_time_constant_b_s.isApprox(
                  core::Vec3{0.6, 0.7, 0.8}));
    CHECK(trajectory.state_machine.states.front()
              .on_entry_guidance_command_filter.specific_force_time_constant_b_s.isApprox(
                  core::Vec3{1.0, 1.1, 1.2}));
    CHECK(trajectory.state_machine.states.front().on_entry_guidance_command_filter.enabled);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed Guidance command filters")
{
    nlohmann::json cfg = valid_state_machine_runtime_config();
    cfg.at("trajectory").at("guidance_command_filter").at("specific_force_time_constant_b_s") = {
        0.2, -0.1, 0.4};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_state_machine_runtime_config();
    nlohmann::json& state = cfg.at("trajectory").at("state_machine").at("states").at(0U);
    state.emplace("on_entry",
                  nlohmann::json{{"guidance_command_filter",
                                  {{"specific_force_time_constant_b_s", {0.8, 0.9, 1.0}},
                                   {"bank_time_constant_s", 1.1},
                                   {"duration_s", 0.0}}}});
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects invalid Guidance graph topology")
{
    SUBCASE("duplicate state IDs")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        cfg.at("trajectory")
            .at("state_machine")
            .at("states")
            .push_back(valid_guidance_state("active"));
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }

    SUBCASE("unknown transition target")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        nlohmann::json& active = cfg.at("trajectory").at("state_machine").at("states").at(0U);
        make_guidance_state_nonterminal(active, "missing");
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }

    SUBCASE("duplicate transition priorities")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        nlohmann::json& machine = cfg.at("trajectory").at("state_machine");
        nlohmann::json& active = machine.at("states").at(0U);
        make_guidance_state_nonterminal(active, "done");
        active.at("transitions")
            .push_back({{"to", "done"},
                        {"priority", 0U},
                        {"when", {{"type", "elapsed_in_state"}, {"greater_equal_s", 2.0}}}});
        machine.at("states").push_back(valid_guidance_state("done"));
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }

    SUBCASE("nonpositive elapsed transition threshold")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        nlohmann::json& machine = cfg.at("trajectory").at("state_machine");
        nlohmann::json& active = machine.at("states").at(0U);
        make_guidance_state_nonterminal(active, "done");
        active.at("transitions").at(0U).at("when").at("greater_equal_s") = 0.0;
        machine.at("states").push_back(valid_guidance_state("done"));
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }

    SUBCASE("unreachable state")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        cfg.at("trajectory")
            .at("state_machine")
            .at("states")
            .push_back(valid_guidance_state("orphan"));
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }

    SUBCASE("cycle rejected by policy")
    {
        nlohmann::json cfg = valid_state_machine_runtime_config();
        nlohmann::json& machine = cfg.at("trajectory").at("state_machine");
        nlohmann::json& first = machine.at("states").at(0U);
        first.at("id") = "first";
        machine.at("initial_state_id") = "first";
        make_guidance_state_nonterminal(first, "second");
        nlohmann::json second = valid_guidance_state("second");
        make_guidance_state_nonterminal(second, "first");
        machine.at("states").push_back(std::move(second));
        CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
    }
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unknown trajectory model discriminators")
{
    nlohmann::json trajectory{
        {"translational_integration", "trapezoidal_predictor_corrector"},
        {"autopilot", {{"type", "second_order"}}},
        {"vehicle_response", {{"type", "first_order"}}},
    };
    CHECK_THROWS_AS(static_cast<void>(autopilot_model_type_from_json(trajectory)),
                    std::runtime_error);

    trajectory.at("autopilot").at("type") = "first_order";
    trajectory.at("vehicle_response").at("type") = "six_dof";
    CHECK_THROWS_AS(static_cast<void>(vehicle_response_model_type_from_json(trajectory)),
                    std::runtime_error);

    trajectory.at("translational_integration") = "rk4";
    CHECK_THROWS_AS(static_cast<void>(translational_integration_method_from_json(trajectory)),
                    std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts every trajectory attitude form")
{
    const nlohmann::json q_identity{1.0, 0.0, 0.0, 0.0};
    const nlohmann::json dcm_identity{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    const nlohmann::json rpy_zero{0.0, 0.0, 0.0};
    const std::array<std::pair<std::string, nlohmann::json>, 18> attitude_forms{{
        {"q_b2e", q_identity},
        {"dcm_b2e", dcm_identity},
        {"rpy_b2e_deg", rpy_zero},
        {"q_e2b", q_identity},
        {"dcm_e2b", dcm_identity},
        {"rpy_e2b_deg", rpy_zero},
        {"q_b2i", q_identity},
        {"dcm_b2i", dcm_identity},
        {"rpy_b2i_deg", rpy_zero},
        {"q_i2b", q_identity},
        {"dcm_i2b", dcm_identity},
        {"rpy_i2b_deg", rpy_zero},
        {"q_b2n", q_identity},
        {"dcm_b2n", dcm_identity},
        {"rpy_b2n_deg", rpy_zero},
        {"q_n2b", q_identity},
        {"dcm_n2b", dcm_identity},
        {"rpy_n2b_deg", rpy_zero},
    }};

    for (const std::pair<std::string, nlohmann::json>& attitude_form : attitude_forms) {
        nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
        cfg.at("trajectory").erase("rpy_b2n_deg");
        cfg.at("trajectory").emplace(attitude_form.first, attitude_form.second);
        CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
    }

    nlohmann::json missing = valid_ecef_ins_gnss_runtime_config();
    missing.at("trajectory").erase("rpy_b2n_deg");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(missing), std::runtime_error);

    nlohmann::json ambiguous = valid_ecef_ins_gnss_runtime_config();
    ambiguous.at("trajectory").emplace("q_b2e", q_identity);
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(ambiguous), std::runtime_error);
}

TEST_CASE("Generated trajectory common validation accepts initial velocity and rejects body rate")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory") = {{"type", "constant_altitude"},
                            {"duration_s", 30.0},
                            {"dynamics_rate_hz", 1000.0},
                            {"p_lla_deg_m", {35.0, -106.0, 1500.0}},
                            {"rpy_b2n_deg", {0.0, 0.0, 28.64788975654116}},
                            {"speed_mps", 120.0},
                            {"v_n_mps", {120.0, 0.0, 0.0}}};
    CHECK_NOTHROW(detail::validate_generated_trajectory_common(cfg.at("trajectory"), true, false));

    cfg.at("trajectory").emplace("w_ib_b_degps", nlohmann::json{0.0, 0.0, 0.0});
    CHECK_THROWS_AS(detail::validate_generated_trajectory_common(cfg.at("trajectory"), true, false),
                    std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts GNSS full covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("position_cov") =
        nlohmann::json{{"frame", "ecef"}, {"full", identity_vec3_cov_full()}};
    cfg.at("gnss").at("velocity_cov") =
        nlohmann::json{{"frame", "ecef"}, {"full", identity_vec3_cov_full()}};

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed GNSS covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("position_cov").at("frame") = "body";
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("position_cov").at("diag").at("pos_m2") = {1.0, -1.0, 1.0};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("position_cov") =
        nlohmann::json{{"frame", "ecef"}, {"full", {1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}}};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("position_cov") =
        nlohmann::json{{"frame", "ecef"}, {"full", {1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0}}};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Runtime JSON components merge relative to the scenario master")
{
    const std::filesystem::path temp_root =
        std::filesystem::temp_directory_path() / "navkit_json_component_test";
    const std::filesystem::path component_dir = temp_root / "components";
    const std::filesystem::path component_path = component_dir / "base.json";
    const std::filesystem::path master_path = temp_root / "scenario.json";

    std::filesystem::remove_all(temp_root);
    std::filesystem::create_directories(component_dir);

    {
        std::ofstream component_file{component_path};
        component_file << R"({
  "trajectory": {
    "duration_s": 60.0,
    "dynamics_rate_hz": 1000.0
  },
  "gnss": {
    "position_cov": {
      "frame": "ned",
      "diag": {
        "pos_m2": [9.0, 9.0, 25.0]
      }
    }
  }
})";
    }

    {
        std::ofstream master_file{master_path};
        master_file << R"({
  "components": {
    "base": "components/base.json"
  },
  "run_name": "json_component_test",
  "trajectory": {
    "duration_s": 5.0
  }
})";
    }

    const nlohmann::json cfg = load_json_file(master_path);

    CHECK_FALSE(cfg.contains("components"));
    CHECK(cfg.at("run_name").get<std::string>() == "json_component_test");
    CHECK(cfg.at("trajectory").at("duration_s").get<double>() == doctest::Approx(5.0));
    CHECK(cfg.at("trajectory").at("dynamics_rate_hz").get<double>() == doctest::Approx(1000.0));
    CHECK(cfg.at("gnss").at("position_cov").at("diag").at("pos_m2").at(0).get<double>() ==
          doctest::Approx(9.0));

    std::filesystem::remove_all(temp_root);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing required sections")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("gnss");

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing runtime-owned app settings")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("run_name");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("output_dir");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("logging");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing runtime-owned cadences")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").erase("dynamics_rate_hz");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").erase("duration_s");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("application");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application").erase("clock");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application").at("clock") = "realtime";
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application").at("clock") = "invalid";
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application").erase("control_state_source");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application").at("control_state_source") = "truth";
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("application") = {{"clock", "simulated"},
                             {"control_state_source", "navigation_estimate"},
                             {"rate_hz", 600.0}};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").erase("dt_s");
    cfg.at("gnss").emplace("rate_hz", 600.0);
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg.at("application") = {{"clock", "simulated"},
                             {"control_state_source", "navigation_estimate"},
                             {"rate_hz", 3000.0}};
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("logging").at("truth").erase("rate_hz");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing runtime-owned simulator settings")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("imu").erase("seed");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("imu").erase("type");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").erase("noise_enabled");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").erase("position_cov");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("GNSS innovation acceptance requires probabilities strictly between zero and one")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("gnss").at("chi_square_acceptance").at("position").at("probability") = 0.0;
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("chi_square_acceptance").at("velocity").at("probability") = 1.0;
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("chi_square_acceptance").at("position").erase("probability");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").at("chi_square_acceptance").at("position") = {{"enabled", false}};
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("gnss").at("chi_square_acceptance").at("position").emplace("probability", 0.99);
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    EcefInsGnssAppConfig::PrimaryGnssPositionSensor position_sensor{};
    EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::configure_sensor(position_sensor, cfg);
    CHECK_FALSE(position_sensor.innovation_gate().enabled());

    cfg.at("gnss").at("chi_square_acceptance").at("position").at("probability") = -0.1;
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("GNSS innovation gate can be disabled by a deep-merged scenario overlay")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    const nlohmann::json overlay{
        {"gnss", {{"chi_square_acceptance", {{"position", {{"enabled", false}}}}}}}};
    detail::merge_json_object(overlay, cfg);

    CHECK(cfg.at("gnss").at("chi_square_acceptance").at("position").contains("probability"));
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    EcefInsGnssAppConfig::PrimaryGnssPositionSensor position_sensor{};
    EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::configure_sensor(position_sensor, cfg);
    CHECK_FALSE(position_sensor.innovation_gate().enabled());
}

TEST_CASE("ECEF INS GNSS runtime validator accepts sorted non-overlapping active windows")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").emplace("active_windows",
                           nlohmann::json::array({
                               {{"start_s", 10.0}, {"end_s", 20.0}},
                               {{"start_s", 30.0}, {"end_s", 35.0}},
                           }));

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
    const navkit::sim::GnssSimulatorConfig gnss =
        detail::gnss_runtime_config_from_json(cfg, "gnss");
    REQUIRE(gnss.active_windows.size() == 2U);
    CHECK(gnss.active_windows.at(0U).start_s == doctest::Approx(10.0));
    CHECK(gnss.active_windows.at(1U).end_s == doctest::Approx(35.0));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed active windows")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").emplace("active_windows", nlohmann::json::object());
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").emplace("active_windows",
                           nlohmann::json::array({{{"start_s", 10.0}, {"end_s", 10.0}}}));
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("gnss").emplace("active_windows",
                           nlohmann::json::array({
                               {{"start_s", 10.0}, {"end_s", 20.0}},
                               {{"start_s", 19.0}, {"end_s", 25.0}},
                           }));
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing initialization")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("pva_initialization");

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unsupported initialization type")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization").at("type") = "explicit_pva";

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed initialization covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization").at("pva_error_cov") = {{"diag", {1.0, 2.0}}};

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects ambiguous initialization covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization").at("pva_error_cov") = {
        {"diag", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}, {"full", identity_pva_cov_full()}};

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed runtime initial covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"initial_covariance", {{"diag", {1.0, 2.0}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts runtime initial covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"initial_covariance",
                                {{"diag",
                                  {1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   7.0,
                                   8.0,
                                   9.0,
                                   10.0,
                                   11.0,
                                   12.0,
                                   13.0,
                                   14.0,
                                   15.0}}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator accepts runtime covariance floor")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{
            {"covariance_floor",
             {{"diag",
               {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0}}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed runtime covariance floor")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"covariance_floor", {{"diag", {1.0, 2.0}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{
            {"covariance_floor",
             {{"diag",
               {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"covariance_floor", {{"full", identity_initial_covariance_full()}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts non-PVA nominal state override")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"nominal_state",
                        {{"non_pva_values", {1.0e-4, 2.0e-4, 3.0e-4, 1.0e-3, 2.0e-3, 3.0e-3}}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed non-PVA nominal state override")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"nominal_state", {{"non_pva_values", {1.0, 2.0}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"nominal_state", {{"gyro_bias_radps", {1.0e-4, 2.0e-4, 3.0e-4}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unknown filter initialization keys")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization", nlohmann::json{{"monte_carlo_truth_error", {}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts generic random initial estimate error")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{
            {"initial_estimate_error",
             {{"type", "random_error"},
              {"covariance",
               {{"diag",
                 {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}}},
              {"seed", 123U}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed initial estimate error")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{
            {"initial_estimate_error",
             {{"type", "random_error"}, {"covariance", {{"diag", {1.0, 2.0}}}}, {"seed", 123U}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts runtime propagation config")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("propagation", runtime_propagation_json());

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed runtime propagation config")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("propagation", runtime_propagation_json());
    cfg.at("propagation").at("process_noise").erase("gyro_white_noise_psd_rad2ps");

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("propagation", runtime_propagation_json());
    cfg.at("propagation").at("imu_bias_dynamics").at("gyro_bias_correlation_rate_1ps").at(0) = -1.0;

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts frame-aware PVA initial covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"initial_covariance",
                        {{"pva_frame", "ned"},
                         {"pva_diag",
                          {{"pos_m2", {1.0, 2.0, 3.0}},
                           {"vel_m2ps2", {4.0, 5.0, 6.0}},
                           {"att_rotvec_rad2", {7.0, 8.0, 9.0}}}},
                         {"remaining_error_state_diag", {10.0, 11.0, 12.0, 13.0, 14.0, 15.0}}}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed frame-aware PVA initial covariance")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"initial_covariance",
                                {{"pva_frame", "ned"},
                                 {"pva_diag",
                                  {{"pos_m2", {1.0, 2.0, 3.0}},
                                   {"vel_m2ps2", {4.0, 5.0, 6.0}},
                                   {"att_rotvec_rad2", {7.0, 8.0, 9.0}}}},
                                 {"remaining_error_state_diag", {10.0, 11.0}}}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects disabled transfer alignment inputs")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"nominal_state",
                        {{"non_pva_values", {1.0e-4, 2.0e-4, 3.0e-4, 1.0e-3, 2.0e-3, 3.0e-3}}}}});
    cfg.emplace("transfer_alignment", nlohmann::json{{"type", "pva_aiding"}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unsupported sensor sections")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("baro", nlohmann::json::object());

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unsupported IMU configuration")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("imu").at("type") = "perfectish";

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects invalid trajectory shape")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").at("p_lla_deg_m") = {1.0, 2.0};

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects ambiguous runtime rates")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").emplace("dynamics_dt_s", 0.001);

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects ambiguous logging rates")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("logging").at("nav_estimate").emplace("dt_s", 0.1);

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator requires enabled logging cadence")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("logging").at("imu_debug") = {{"enabled", false}};
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("logging").at("imu_debug") = {{"enabled", true}};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("trajectory inspection logs are optional and require cadence when enabled")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("logging").emplace("trajectory_kinematics_eci", nlohmann::json{{"enabled", false}});
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));

    cfg.at("logging").at("trajectory_kinematics_eci") = {{"enabled", true}};
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg.at("logging").at("trajectory_kinematics_eci").emplace("rate_hz", 20.0);
    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unsupported covariance log modes")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("logging").at("nav_estimate").at("covariance") = "full";

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator keeps numeric tuning runtime-configurable")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").at("duration_s") = 5.0;
    cfg.at("gnss").at("position_cov").at("diag").at("pos_m2") = {0.0, 0.0, 0.0};
    cfg.at("pva_initialization").at("pva_error_cov").at("diag").at(0) = 0.0;

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("Trajectory w_nb_b initialization includes Earth and local transport rates")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory") = {{"type", "stationary"},
                            {"duration_s", 1.0},
                            {"dynamics_rate_hz", 1.0},
                            {"p_lla_deg_m", {0.0, 0.0, 0.0}},
                            {"v_n_mps", {0.0, 100.0, 0.0}},
                            {"rpy_b2n_deg", {0.0, 0.0, 0.0}},
                            {"w_nb_b_degps", {0.0, 0.0, 0.0}}};

    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const core::Scalar_t expected_x_radps =
        core::environment::Wgs84::omega_rad_s + (100.0 / core::environment::Wgs84::a_m);
    CHECK(trajectory.initial_truth.w_ib_b_radps.x() == doctest::Approx(expected_x_radps));
    CHECK(trajectory.initial_truth.w_ib_b_radps.y() == doctest::Approx(0.0));
    CHECK(trajectory.initial_truth.w_ib_b_radps.z() == doctest::Approx(0.0));
}

TEST_CASE("Explicit PVA initialization provider applies configured errors")
{
    const nlohmann::json cfg = explicit_pva_runtime_config();
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);

    static_assert(NavInitializationProviderPolicy<PvaExplicitInitializationProvider>);
    CHECK_NOTHROW(PvaExplicitInitializationProvider::validate_runtime_config(cfg));

    const PvaInitialization pva_init =
        PvaExplicitInitializationProvider::initialize(cfg, trajectory);

    CHECK(core::timestamp_seconds(pva_init.t) == doctest::Approx(0.0));
    CHECK(core::estimation::pos_e_m(pva_init.pva)(0) == doctest::Approx(6378137.0 - 10.0));
    CHECK(core::estimation::pos_e_m(pva_init.pva)(1) == doctest::Approx(-15.0));
    CHECK(core::estimation::pos_e_m(pva_init.pva)(2) == doctest::Approx(25.0));
    CHECK(core::estimation::vel_e_mps(pva_init.pva).isZero());
    CHECK(core::estimation::rpy_b2e_rad(pva_init.pva)
              .isApprox(core::math::rpy_rad_from_quaternion(trajectory.initial_truth.q_b2e)));
}

TEST_CASE("Direct PVA initialization provider uses configured values")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization") = {
        {"type", "pva_direct"},
        {"pva",
         {{"time_s", 12.5},
          {"p_e_m", {1.0, 2.0, 3.0}},
          {"v_e_mps", {4.0, 5.0, 6.0}},
          {"rpy_b2e_deg", {5.729577951308233, 11.459155902616466, 17.188733853924695}}}}};
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);

    static_assert(NavInitializationProviderPolicy<PvaDirectInitializationProvider>);
    CHECK_NOTHROW(PvaDirectInitializationProvider::validate_runtime_config(cfg));

    const PvaInitialization pva_init = PvaDirectInitializationProvider::initialize(cfg, trajectory);

    CHECK(core::timestamp_seconds(pva_init.t) == doctest::Approx(12.5));
    CHECK(core::estimation::pos_e_m(pva_init.pva).isApprox(core::Vec3{1.0, 2.0, 3.0}));
    CHECK(core::estimation::vel_e_mps(pva_init.pva).isApprox(core::Vec3{4.0, 5.0, 6.0}));
    CHECK(core::estimation::rpy_b2e_rad(pva_init.pva).isApprox(core::Vec3{0.1, 0.2, 0.3}));
}

TEST_CASE("Full row-major filter initial covariance populates the Kalman filter")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Error = StateDef::Error;

    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    std::vector<double> full_cov = identity_initial_covariance_full();
    full_cov.at((0U * 15U) + 3U) = 0.25;
    full_cov.at((3U * 15U) + 0U) = 0.25;
    cfg.emplace("filter_initialization",
                nlohmann::json{{"initial_covariance", {{"full", full_cov}}}});

    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const PvaInitialization pva_init = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    std::unique_ptr<NavKit::Navigator> navigator = std::make_unique<NavKit::Navigator>();
    initialize_navigator<NavKit>(pva_init, cfg, *navigator);
    CHECK(navigator->filter().covariance()(Error::Pos::i, Error::Vel::i) == doctest::Approx(0.25));
    CHECK(navigator->filter().covariance()(Error::Vel::i, Error::Pos::i) == doctest::Approx(0.25));
}

TEST_CASE("Frame-aware PVA initial covariance populates the full filter covariance")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Error = StateDef::Error;

    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"initial_covariance",
                        {{"pva_frame", "ned"},
                         {"pva_diag",
                          {{"pos_m2", {1.0, 2.0, 3.0}},
                           {"vel_m2ps2", {4.0, 5.0, 6.0}},
                           {"att_rotvec_rad2", {7.0, 8.0, 9.0}}}},
                         {"remaining_error_state_diag", {10.0, 11.0, 12.0, 13.0, 14.0, 15.0}}}}});
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const PvaInitialization pva_init = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    std::unique_ptr<NavKit::Navigator> navigator = std::make_unique<NavKit::Navigator>();
    initialize_navigator<NavKit>(pva_init, cfg, *navigator);
    const typename NavKit::Filter::P_t& covariance = navigator->filter().covariance();

    CHECK(covariance(Error::Pos::i + 0, Error::Pos::i + 0) == doctest::Approx(3.0));
    CHECK(covariance(Error::Pos::i + 1, Error::Pos::i + 1) == doctest::Approx(2.0));
    CHECK(covariance(Error::Pos::i + 2, Error::Pos::i + 2) == doctest::Approx(1.0));
    CHECK(covariance(Error::Vel::i + 0, Error::Vel::i + 0) == doctest::Approx(6.0));
    CHECK(covariance(Error::Vel::i + 1, Error::Vel::i + 1) == doctest::Approx(5.0));
    CHECK(covariance(Error::Vel::i + 2, Error::Vel::i + 2) == doctest::Approx(4.0));
    CHECK(covariance(Error::AttRotVec::i + 0, Error::AttRotVec::i + 0) == doctest::Approx(9.0));
    CHECK(covariance(Error::AttRotVec::i + 1, Error::AttRotVec::i + 1) == doctest::Approx(8.0));
    CHECK(covariance(Error::AttRotVec::i + 2, Error::AttRotVec::i + 2) == doctest::Approx(7.0));
    CHECK(covariance(Error::GyroB::i + 0, Error::GyroB::i + 0) == doctest::Approx(10.0));
    CHECK(covariance(Error::GyroB::i + 1, Error::GyroB::i + 1) == doctest::Approx(11.0));
    CHECK(covariance(Error::GyroB::i + 2, Error::GyroB::i + 2) == doctest::Approx(12.0));
    CHECK(covariance(Error::AccB::i + 0, Error::AccB::i + 0) == doctest::Approx(13.0));
    CHECK(covariance(Error::AccB::i + 1, Error::AccB::i + 1) == doctest::Approx(14.0));
    CHECK(covariance(Error::AccB::i + 2, Error::AccB::i + 2) == doctest::Approx(15.0));
}

TEST_CASE("Runtime covariance floor populates and clamps the Navigator filter covariance")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Error = StateDef::Error;

    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"covariance_floor",
                        {{"pva_frame", "ned"},
                         {"pva_diag",
                          {{"pos_m2", {1.0, 2.0, 3.0}},
                           {"vel_m2ps2", {4.0, 5.0, 6.0}},
                           {"att_rotvec_rad2", {7.0, 8.0, 9.0}}}},
                         {"remaining_error_state_diag", {10.0, 11.0, 12.0, 13.0, 14.0, 15.0}}}}});
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const PvaInitialization pva_init = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    std::unique_ptr<NavKit::Navigator> navigator = std::make_unique<NavKit::Navigator>();
    NavKit::Filter::P_t covariance = NavKit::Filter::P_t::Zero();
    navigator->filter().set_covariance(covariance);
    navigator->filter().set_covariance_floor(detail::covariance_floor_from_json<StateDef>(
        cfg, NavKit::CovarianceFloor::covariance_floor, core::estimation::pos_e_m(pva_init.pva)));

    CHECK(navigator->filter().covariance()(Error::Pos::i + 0, Error::Pos::i + 0) ==
          doctest::Approx(3.0));
    CHECK(navigator->filter().covariance()(Error::Pos::i + 1, Error::Pos::i + 1) ==
          doctest::Approx(2.0));
    CHECK(navigator->filter().covariance()(Error::Pos::i + 2, Error::Pos::i + 2) ==
          doctest::Approx(1.0));
    CHECK(navigator->filter().covariance()(Error::GyroB::i + 0, Error::GyroB::i + 0) ==
          doctest::Approx(10.0));
    CHECK(navigator->filter().covariance()(Error::AccB::i + 2, Error::AccB::i + 2) ==
          doctest::Approx(15.0));
}

TEST_CASE("Runtime propagation override populates the Navigator propagation policy")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;

    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.emplace("propagation", runtime_propagation_json());

    std::unique_ptr<NavKit::Navigator> navigator = std::make_unique<NavKit::Navigator>();
    navigator->propagation().set_runtime_config(
        detail::propagation_runtime_config_from_json<NavKit::Propagation>(
            cfg, NavKit::Propagation::runtime_config));

    CHECK(navigator->propagation()
              .runtime_config_value()
              .imu_bias_dynamics.gyro_bias_correlation_rate_1ps.x() == doctest::Approx(0.25));
    CHECK(navigator->propagation()
              .runtime_config_value()
              .imu_bias_dynamics.accel_bias_correlation_rate_1ps.z() == doctest::Approx(0.5));
}

TEST_CASE("Random PVA initialization provider produces deterministic colored draws")
{
    const nlohmann::json cfg = random_pva_runtime_config();
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);

    static_assert(NavInitializationProviderPolicy<PvaRandomInitializationProvider>);
    CHECK_NOTHROW(PvaRandomInitializationProvider::validate_runtime_config(cfg));

    const PvaInitialization first = PvaRandomInitializationProvider::initialize(cfg, trajectory);
    const PvaInitialization second = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    CHECK(core::estimation::pos_e_m(first.pva).isApprox(core::estimation::pos_e_m(second.pva)));
    CHECK(core::estimation::vel_e_mps(first.pva).isApprox(core::estimation::vel_e_mps(second.pva)));
    CHECK(core::estimation::rpy_b2e_rad(first.pva).isApprox(
        core::estimation::rpy_b2e_rad(second.pva)));
    CHECK_FALSE((core::estimation::pos_e_m(first.pva) - trajectory.initial_truth.p_e).isZero());
}

TEST_CASE("Random PVA initialization provider accepts NED covariance frame")
{
    nlohmann::json cfg = random_pva_runtime_config();
    cfg.at("pva_initialization").at("pva_error_frame") = "ned";
    cfg.at("pva_initialization").at("pva_error_cov") = {
        {"diag", {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);

    CHECK_NOTHROW(PvaRandomInitializationProvider::validate_runtime_config(cfg));
    const PvaInitialization pva_init = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    CHECK_FALSE((core::estimation::pos_e_m(pva_init.pva) - trajectory.initial_truth.p_e).isZero());
}

TEST_CASE("NavInitialization maps into the configured Navigator filter state")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;

    nlohmann::json cfg = explicit_pva_runtime_config();
    cfg.emplace(
        "filter_initialization",
        nlohmann::json{{"initial_covariance",
                        {{"pva_frame", "ned"},
                         {"pva_diag",
                          {{"pos_m2", {10000.0, 10000.0, 10000.0}},
                           {"vel_m2ps2", {100.0, 100.0, 100.0}},
                           {"att_rotvec_rad2",
                            {0.007615435494667714, 0.007615435494667714, 0.030461741978670857}}}},
                         {"remaining_error_state_diag",
                          {NavKit::InitialCovariance::initial_covariance(Error::GyroB::i + 0,
                                                                         Error::GyroB::i + 0),
                           NavKit::InitialCovariance::initial_covariance(Error::GyroB::i + 1,
                                                                         Error::GyroB::i + 1),
                           NavKit::InitialCovariance::initial_covariance(Error::GyroB::i + 2,
                                                                         Error::GyroB::i + 2),
                           NavKit::InitialCovariance::initial_covariance(Error::AccB::i + 0,
                                                                         Error::AccB::i + 0),
                           NavKit::InitialCovariance::initial_covariance(Error::AccB::i + 1,
                                                                         Error::AccB::i + 1),
                           NavKit::InitialCovariance::initial_covariance(Error::AccB::i + 2,
                                                                         Error::AccB::i + 2)}}}}});
    cfg.at("filter_initialization")
        .emplace(
            "nominal_state",
            nlohmann::json{{"non_pva_values", {1.0e-4, 2.0e-4, 3.0e-4, 1.0e-3, 2.0e-3, 3.0e-3}}});
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const PvaInitialization nav_init =
        PvaExplicitInitializationProvider::initialize(cfg, trajectory);
    const Eigen::Quaternion<core::Scalar_t> expected_q_b2e =
        trajectory.initial_truth.q_b2e.normalized();

    auto navigator = std::make_unique<NavKit::Navigator>();
    initialize_navigator<NavKit>(nav_init, cfg, *navigator);

    const auto& filter = navigator->filter();
    CHECK(filter.state()(Nominal::Pos::i + 0) == doctest::Approx(6378137.0 - 10.0));
    CHECK(filter.state()(Nominal::Pos::i + 1) == doctest::Approx(-15.0));
    CHECK(filter.state()(Nominal::Pos::i + 2) == doctest::Approx(25.0));
    CHECK(filter.state().template segment<3>(Nominal::Vel::i).isZero());
    CHECK(filter.state()
              .template segment<4>(Nominal::AttQuat::i)
              .isApprox(Eigen::Matrix<core::Scalar_t, 4, 1>{
                  expected_q_b2e.w(), expected_q_b2e.x(), expected_q_b2e.y(), expected_q_b2e.z()}));
    CHECK(filter.covariance()(Error::Pos::i, Error::Pos::i) == doctest::Approx(10000.0));
    CHECK(filter.covariance()(Error::Vel::i, Error::Vel::i) == doctest::Approx(100.0));
    CHECK(filter.covariance()(Error::AttRotVec::i, Error::AttRotVec::i) ==
          doctest::Approx(0.030461741978670857));
    CHECK(filter.covariance()(Error::AttRotVec::i + 1, Error::AttRotVec::i + 1) ==
          doctest::Approx(0.007615435494667714));
    CHECK(filter.covariance()(Error::AttRotVec::i + 2, Error::AttRotVec::i + 2) ==
          doctest::Approx(0.007615435494667714));

    CHECK(filter.covariance()(Error::GyroB::i, Error::GyroB::i) ==
          doctest::Approx(
              NavKit::InitialCovariance::initial_covariance(Error::GyroB::i, Error::GyroB::i)));
    CHECK(filter.covariance()(Error::AccB::i, Error::AccB::i) ==
          doctest::Approx(
              NavKit::InitialCovariance::initial_covariance(Error::AccB::i, Error::AccB::i)));
    CHECK(filter.state()(Nominal::GyroB::i + 0) == doctest::Approx(1.0e-4));
    CHECK(filter.state()(Nominal::GyroB::i + 1) == doctest::Approx(2.0e-4));
    CHECK(filter.state()(Nominal::GyroB::i + 2) == doctest::Approx(3.0e-4));
    CHECK(filter.state()(Nominal::AccB::i + 0) == doctest::Approx(1.0e-3));
    CHECK(filter.state()(Nominal::AccB::i + 1) == doctest::Approx(2.0e-3));
    CHECK(filter.state()(Nominal::AccB::i + 2) == doctest::Approx(3.0e-3));
}

TEST_CASE("Initial estimate error applies against the simulation truth reference")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Nominal = StateDef::Nominal;

    nlohmann::json cfg = explicit_pva_runtime_config();
    cfg.emplace("filter_initialization",
                nlohmann::json{{"initial_estimate_error",
                                {{"type", "explicit_error"},
                                 {"values",
                                  {1.0,
                                   2.0,
                                   3.0,
                                   4.0,
                                   5.0,
                                   6.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0e-4,
                                   2.0e-4,
                                   3.0e-4,
                                   1.0e-3,
                                   2.0e-3,
                                   3.0e-3}}}}});
    const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
    const PvaInitialization pva_init =
        PvaExplicitInitializationProvider::initialize(cfg, trajectory);
    InitialTruthReference<StateDef> reference{};
    populate_initial_pva_from_truth<StateDef>(trajectory.initial_truth, reference);
    core::estimation::segment<Nominal::GyroB>(reference.truth_state) = core::Vec3{0.01, 0.02, 0.03};
    core::estimation::segment<Nominal::AccB>(reference.truth_state) = core::Vec3{0.1, 0.2, 0.3};

    std::unique_ptr<NavKit::Navigator> navigator = std::make_unique<NavKit::Navigator>();
    initialize_navigator<NavKit>(pva_init, cfg, reference, *navigator);

    const typename NavKit::Filter::State_t& state = navigator->filter().state();
    CHECK(state(Nominal::Pos::i + 0) ==
          doctest::Approx(reference.truth_state(Nominal::Pos::i) + 1.0));
    CHECK(state(Nominal::Vel::i + 2) ==
          doctest::Approx(reference.truth_state(Nominal::Vel::i + 2) + 6.0));
    CHECK(state(Nominal::GyroB::i + 0) == doctest::Approx(0.0101));
    CHECK(state(Nominal::GyroB::i + 2) == doctest::Approx(0.0303));
    CHECK(state(Nominal::AccB::i + 0) == doctest::Approx(0.101));
    CHECK(state(Nominal::AccB::i + 2) == doctest::Approx(0.303));
    CHECK(state.template segment<4>(Nominal::AttQuat::i).norm() == doctest::Approx(1.0));

    std::unique_ptr<NavKit::Navigator> navigator_without_reference =
        std::make_unique<NavKit::Navigator>();
    CHECK_THROWS_AS(initialize_navigator<NavKit>(pva_init, cfg, *navigator_without_reference),
                    std::runtime_error);
}

} // namespace navkit::app_support::test
