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
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/ImuSimulatorPolicy.hpp"
#include "test_main.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
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

struct DuplicateSensorIdConfig
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using Logger = EcefInsGnssAppConfig::Logger;
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
    using Logger = EcefInsGnssAppConfig::Logger;
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
          {"rate_hz", 1000.0},
          {"p_lla_deg_m", {0.0, 0.0, 0.0}},
          {"v_n_mps", {0.0, 0.0, 0.0}},
          {"rpy_b2n_rad", {0.0, 0.0, 0.0}},
          {"w_nb_b_radps", {0.0, 0.0, 0.0}}}},
        {"imu", {{"type", "ideal"}, {"rate_hz", 1000.0}, {"seed", 42U}}},
        {"gnss",
         {{"dt_s", 1.0},
          {"position_cov", {{"frame", "ned"}, {"diag", {{"pos_m2", {9.0, 9.0, 25.0}}}}}},
          {"velocity_cov", {{"frame", "ned"}, {"diag", {{"vel_m2ps2", {0.04, 0.04, 0.04}}}}}},
          {"p_b_ant_b_m", {0.0, 0.0, 0.0}},
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
                                      {"rotvec_b2n_rad", {0.0, 0.0, 0.0}}}}};
    return cfg;
}

} // namespace

TEST_CASE("ECEF INS GNSS runtime validator accepts the documented input shape")
{
    const auto cfg = valid_ecef_ins_gnss_runtime_config();

    static_assert(SimulationAppConfigPolicy<EcefInsGnssAppConfig>);
    static_assert(navkit::sim::ImuSimulatorPolicy<EcefInsGnssAppConfig::ImuSimulator>);
    static_assert(!SimulationAppConfigPolicy<DuplicateSensorIdConfig>);
    static_assert(!SimulationAppConfigPolicy<MissingTargetSensorConfig>);
    static_assert(EmulatorPolicy<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator,
                                 EcefInsGnssAppConfig::PrimaryGnssPositionSensor,
                                 EcefInsGnssAppConfig::Logger>);
    static_assert(!EmulatorPolicy<NotAnEmulator,
                                  EcefInsGnssAppConfig::PrimaryGnssPositionSensor,
                                  EcefInsGnssAppConfig::Logger>);
    static_assert(EmulatorBindingPolicy<EcefInsGnssAppConfig::PrimaryGnssPositionBinding,
                                        EcefInsGnssAppConfig::Logger>);
    static_assert(!EmulatorBindingPolicy<NotABinding, EcefInsGnssAppConfig::Logger>);
    static_assert(EmulatorBindingTuplePolicy<EcefInsGnssAppConfig::EmulatorBindings,
                                             EcefInsGnssAppConfig::NavKit::Sensors,
                                             EcefInsGnssAppConfig::Logger>);
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
                                              DuplicateSensorIdConfig::Logger>);
    static_assert(!EmulatorBindingTuplePolicy<MissingTargetSensorConfig::EmulatorBindings,
                                              MissingTargetSensorConfig::NavKit::Sensors,
                                              MissingTargetSensorConfig::Logger>);
    static_assert(emulator_binding_ids_unique_v<EcefInsGnssAppConfig::EmulatorBindings>);
    static_assert(
        std::is_same_v<EmulatorFromId_t<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::Id,
                                        EcefInsGnssAppConfig::EmulatorBindings>,
                       GnssEmulator<EcefInsGnssAppConfig::PrimaryGnssPositionEmulator::Id>>);

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
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
    "rate_hz": 1000.0
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
    CHECK(cfg.at("trajectory").at("rate_hz").get<double>() == doctest::Approx(1000.0));
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
    cfg.at("trajectory").erase("rate_hz");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

    cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").erase("duration_s");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);

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
    cfg.at("trajectory").emplace("dt_s", 0.001);

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
    CHECK(
        core::estimation::rpy_b2e_rad(pva_init.pva)
            .isApprox(core::math::rpy_rad_from_quaternion(trajectory.truth_samples.front().q_b2e)));
}

TEST_CASE("Direct PVA initialization provider uses configured values")
{
    nlohmann::json cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("pva_initialization") = {{"type", "pva_direct"},
                                    {"pva",
                                     {{"time_s", 12.5},
                                      {"p_e_m", {1.0, 2.0, 3.0}},
                                      {"v_e_mps", {4.0, 5.0, 6.0}},
                                      {"rpy_b2e_rad", {0.1, 0.2, 0.3}}}}};
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
    typename NavKit::Filter::P_t covariance = NavKit::Filter::P_t::Zero();
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
    CHECK_FALSE((core::estimation::pos_e_m(first.pva) - trajectory.initial_position_e_m).isZero());
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

    CHECK_FALSE(
        (core::estimation::pos_e_m(pva_init.pva) - trajectory.initial_position_e_m).isZero());
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
        trajectory.truth_samples.front().q_b2e.normalized();

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
    populate_initial_pva_from_truth<StateDef>(trajectory.truth_samples.front(), reference);
    core::estimation::segment<typename Nominal::GyroB>(reference.truth_state) =
        core::Vec3{0.01, 0.02, 0.03};
    core::estimation::segment<typename Nominal::AccB>(reference.truth_state) =
        core::Vec3{0.1, 0.2, 0.3};

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
