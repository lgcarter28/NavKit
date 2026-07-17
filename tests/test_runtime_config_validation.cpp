// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "apps/navkit_sim/EcefInsGnss.hpp"
#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/app_support/emulation/EmulatorPolicy.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
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

using EcefInsGnssAppConfig = navkit::config::apps::navkit_sim::EcefInsGnssConfig;

struct DuplicateSensorIdConfig
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using Logger = EcefInsGnssAppConfig::Logger;
    using ImuSimulator = EcefInsGnssAppConfig::ImuSimulator;
    using EmulatorBindings =
        std::tuple<navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<0U>,
                                                        NavKit::PrimaryGnssSensor>,
                   navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<0U>,
                                                        NavKit::PrimaryGnssSensor>>;
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
          {"sigma_h_m", 3.0},
          {"sigma_v_m", 5.0},
          {"seed", 42U},
          {"noise_enabled", true}}},
        {"initialization",
         {{"type", "pva_error"},
          {"pva_error",
           {{"p_n_m", {25.0, -15.0, 10.0}},
            {"v_n_mps", {0.0, 0.0, 0.0}},
            {"rotvec_b2n_rad", {0.0, 0.0, 0.0}}}},
          {"pva_cov",
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

[[nodiscard]] std::vector<double> identity_initial_covariance_full()
{
    std::vector<double> values(225U, 0.0);
    for (std::size_t i = 0; i < 15U; ++i) {
        values.at((i * 15U) + i) = 1.0;
    }
    return values;
}

[[nodiscard]] nlohmann::json random_pva_runtime_config()
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization").erase("pva_error");
    cfg.at("initialization").at("type") = "pva_random";
    cfg.at("initialization").at("pva_cov") = {{"full", identity_pva_cov_full()}};
    cfg.at("initialization").emplace("pva_error_frame", "ecef");
    cfg.at("initialization").emplace("seed", 7U);
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
    static_assert(EmulatorPolicy<EcefInsGnssAppConfig::PrimaryGnssEmulator,
                                 EcefInsGnssAppConfig::PrimaryGnssSensor,
                                 EcefInsGnssAppConfig::Logger>);
    static_assert(!EmulatorPolicy<NotAnEmulator,
                                  EcefInsGnssAppConfig::PrimaryGnssSensor,
                                  EcefInsGnssAppConfig::Logger>);
    static_assert(EmulatorBindingPolicy<EcefInsGnssAppConfig::PrimaryGnssBinding,
                                        EcefInsGnssAppConfig::Logger>);
    static_assert(!EmulatorBindingPolicy<NotABinding, EcefInsGnssAppConfig::Logger>);
    static_assert(EmulatorBindingTuplePolicy<EcefInsGnssAppConfig::EmulatorBindings,
                                             EcefInsGnssAppConfig::NavKit::Sensors,
                                             EcefInsGnssAppConfig::Logger>);
    static_assert(NavInitializationProviderPolicy<EcefInsGnssAppConfig::NavInitializationProvider>);
    static_assert(TransferAlignmentProviderPolicy<EcefInsGnssAppConfig::TransferAlignmentProvider,
                                                  EcefInsGnssAppConfig::NavKit::Navigator>);
    static_assert(navkit::core::estimation::InitialCovarianceConfigPolicy<
                  EcefInsGnssAppConfig::NavKit,
                  EcefInsGnssAppConfig::NavKit::StateDef>);
    static_assert(!EmulatorBindingTuplePolicy<DuplicateSensorIdConfig::EmulatorBindings,
                                              DuplicateSensorIdConfig::NavKit::Sensors,
                                              DuplicateSensorIdConfig::Logger>);
    static_assert(!EmulatorBindingTuplePolicy<MissingTargetSensorConfig::EmulatorBindings,
                                              MissingTargetSensorConfig::NavKit::Sensors,
                                              MissingTargetSensorConfig::Logger>);
    static_assert(emulator_binding_ids_unique_v<EcefInsGnssAppConfig::EmulatorBindings>);
    static_assert(std::is_same_v<EmulatorFromId_t<EcefInsGnssAppConfig::PrimaryGnssEmulator::Id,
                                                  EcefInsGnssAppConfig::EmulatorBindings>,
                                 GnssEmulator<EcefInsGnssAppConfig::PrimaryGnssEmulator::Id>>);

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
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
    "sigma_h_m": 3.0
  }
})";
    }

    {
        std::ofstream master_file{master_path};
        master_file << R"({
  "components": ["components/base.json"],
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
    CHECK(cfg.at("gnss").at("sigma_h_m").get<double>() == doctest::Approx(3.0));

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
    cfg.at("gnss").erase("sigma_h_m");
    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects missing initialization")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.erase("initialization");

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects unsupported initialization type")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization").at("type") = "explicit_pva";

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed initialization covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization").at("pva_cov") = {{"diag", {1.0, 2.0}}};

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects ambiguous initialization covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization").at("pva_cov") = {
        {"diag", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}, {"full", identity_pva_cov_full()}};

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator rejects malformed runtime initial covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization")
        .emplace("initial_covariance", nlohmann::json{{"source", "runtime"}, {"diag", {1.0, 2.0}}});

    CHECK_THROWS_AS(validate_runtime_config<EcefInsGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("ECEF INS GNSS runtime validator accepts runtime initial covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization")
        .emplace("initial_covariance",
                 nlohmann::json{{"source", "runtime"},
                                {"diag",
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
                                  15.0}}});

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("ECEF INS GNSS runtime validator rejects disabled transfer alignment inputs")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
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
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("trajectory").at("duration_s") = 5.0;
    cfg.at("gnss").at("sigma_h_m") = 0.0;
    cfg.at("gnss").at("sigma_v_m") = 0.0;
    cfg.at("initialization").at("pva_cov").at("diag").at(0) = 0.0;

    CHECK_NOTHROW(validate_runtime_config<EcefInsGnssAppConfig>(cfg));
}

TEST_CASE("Explicit PVA initialization provider preserves ECEF INS GNSS demo behavior")
{
    const auto cfg = valid_ecef_ins_gnss_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);

    const auto nav_init =
        EcefInsGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);

    CHECK(nav_init.time_s == doctest::Approx(0.0));
    CHECK(core::estimation::pos_e_m(nav_init.pva)(0) == doctest::Approx(6378137.0 - 10.0));
    CHECK(core::estimation::pos_e_m(nav_init.pva)(1) == doctest::Approx(-15.0));
    CHECK(core::estimation::pos_e_m(nav_init.pva)(2) == doctest::Approx(25.0));
    CHECK(core::estimation::vel_e_mps(nav_init.pva).isZero());
    CHECK(
        core::estimation::rpy_b2e_rad(nav_init.pva)
            .isApprox(core::math::rpy_rad_from_quaternion(trajectory.truth_samples.front().q_b2e)));
    CHECK(nav_init.pva_cov(0, 0) == doctest::Approx(10000.0));
    CHECK(nav_init.pva_cov(3, 3) == doctest::Approx(100.0));
    CHECK(nav_init.pva_cov(6, 6) == doctest::Approx(0.030461741978670857));
    CHECK(nav_init.pva_cov(7, 7) == doctest::Approx(0.007615435494667714));
    CHECK(nav_init.pva_cov(8, 8) == doctest::Approx(0.007615435494667714));
}

TEST_CASE("Explicit PVA initialization provider accepts full row-major covariance")
{
    auto cfg = valid_ecef_ins_gnss_runtime_config();
    cfg.at("initialization").at("pva_error") = {{"p_e_m", {25.0, -15.0, 10.0}},
                                                {"v_e_mps", {0.0, 0.0, 0.0}},
                                                {"rotvec_b2e_rad", {0.0, 0.0, 0.0}}};
    auto full_cov = identity_pva_cov_full();
    full_cov.at((0U * 9U) + 3U) = 0.25;
    full_cov.at((3U * 9U) + 0U) = 0.25;
    cfg.at("initialization").at("pva_cov") = {{"full", full_cov}};

    const auto trajectory = trajectory_run_from_json(cfg);
    const auto nav_init =
        EcefInsGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);

    CHECK(nav_init.pva_cov(0, 0) == doctest::Approx(1.0));
    CHECK(nav_init.pva_cov(0, 3) == doctest::Approx(0.25));
    CHECK(nav_init.pva_cov(3, 0) == doctest::Approx(0.25));

    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Error = StateDef::Error;
    auto navigator = std::make_unique<NavKit::Navigator>();
    cfg.at("initialization")
        .emplace(
            "initial_covariance",
            nlohmann::json{{"source", "runtime"}, {"full", identity_initial_covariance_full()}});
    cfg.at("initialization").at("initial_covariance").at("full").at((0U * 15U) + 3U) = 0.25;
    cfg.at("initialization").at("initial_covariance").at("full").at((3U * 15U) + 0U) = 0.25;
    initialize_navigator<NavKit>(nav_init, cfg, *navigator);
    CHECK(navigator->filter().covariance()(Error::Pos::i, Error::Vel::i) == doctest::Approx(0.25));
    CHECK(navigator->filter().covariance()(Error::Vel::i, Error::Pos::i) == doctest::Approx(0.25));
}

TEST_CASE("Random PVA initialization provider produces deterministic colored draws")
{
    const auto cfg = random_pva_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);

    static_assert(NavInitializationProviderPolicy<PvaRandomInitializationProvider>);
    CHECK_NOTHROW(PvaRandomInitializationProvider::validate_runtime_config(cfg));

    const auto first = PvaRandomInitializationProvider::initialize(cfg, trajectory);
    const auto second = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    CHECK(core::estimation::pos_e_m(first.pva).isApprox(core::estimation::pos_e_m(second.pva)));
    CHECK(core::estimation::vel_e_mps(first.pva).isApprox(core::estimation::vel_e_mps(second.pva)));
    CHECK(core::estimation::rpy_b2e_rad(first.pva).isApprox(
        core::estimation::rpy_b2e_rad(second.pva)));
    CHECK_FALSE((core::estimation::pos_e_m(first.pva) - trajectory.initial_position_e_m).isZero());
    CHECK(first.pva_cov(0, 0) == doctest::Approx(1.0));
}

TEST_CASE("Random PVA initialization provider accepts NED covariance frame")
{
    auto cfg = random_pva_runtime_config();
    cfg.at("initialization").at("pva_error_frame") = "ned";
    cfg.at("initialization").at("pva_cov") = {
        {"diag", {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0}}};
    const auto trajectory = trajectory_run_from_json(cfg);

    CHECK_NOTHROW(PvaRandomInitializationProvider::validate_runtime_config(cfg));
    const auto nav_init = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    CHECK(nav_init.pva_cov(0, 0) == doctest::Approx(3.0));
    CHECK(nav_init.pva_cov(1, 1) == doctest::Approx(2.0));
    CHECK(nav_init.pva_cov(2, 2) == doctest::Approx(1.0));
    CHECK(nav_init.pva_cov(3, 3) == doctest::Approx(6.0));
    CHECK(nav_init.pva_cov(4, 4) == doctest::Approx(5.0));
    CHECK(nav_init.pva_cov(5, 5) == doctest::Approx(4.0));
    CHECK(nav_init.pva_cov(6, 6) == doctest::Approx(9.0));
    CHECK(nav_init.pva_cov(7, 7) == doctest::Approx(8.0));
    CHECK(nav_init.pva_cov(8, 8) == doctest::Approx(7.0));
}

TEST_CASE("NavInitialization maps into the configured Navigator filter state")
{
    using NavKit = EcefInsGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;

    const auto cfg = valid_ecef_ins_gnss_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);
    const auto nav_init =
        EcefInsGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);
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
          doctest::Approx(0.007615435494667714));
    CHECK(filter.covariance()(Error::AttRotVec::i + 1, Error::AttRotVec::i + 1) ==
          doctest::Approx(0.007615435494667714));
    CHECK(filter.covariance()(Error::AttRotVec::i + 2, Error::AttRotVec::i + 2) ==
          doctest::Approx(0.030461741978670857));

    CHECK(filter.covariance()(Error::GyroB::i, Error::GyroB::i) ==
          doctest::Approx(NavKit::initial_covariance(Error::GyroB::i, Error::GyroB::i)));
    CHECK(filter.covariance()(Error::AccB::i, Error::AccB::i) ==
          doctest::Approx(NavKit::initial_covariance(Error::AccB::i, Error::AccB::i)));
}

} // namespace navkit::app_support::test
