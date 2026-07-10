// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "apps/navkit_sim/StationaryGnss.hpp"
#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/app_support/emulation/EmulatorPolicy.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/initialization/NavInitializationProviderPolicy.hpp"
#include "navkit/app_support/initialization/TransferAlignmentProviderPolicy.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/io/RunLogger.hpp"
#include "test_main.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

using StationaryGnssAppConfig = navkit::config::apps::navkit_sim::StationaryGnssConfig;

struct DuplicateSensorIdConfig
{
    using NavKit = StationaryGnssAppConfig::NavKit;
    using Logger = StationaryGnssAppConfig::Logger;
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
    using NavKit = StationaryGnssAppConfig::NavKit;
    using Logger = StationaryGnssAppConfig::Logger;
    using EmulatorBindings =
        std::tuple<navkit::app_support::EmulatorBinding<navkit::app_support::GnssEmulator<99U>,
                                                        UnknownSensor>>;
};

struct NotAnEmulator
{};

struct NotABinding
{};

[[nodiscard]] nlohmann::json valid_stationary_gnss_runtime_config()
{
    return {{"run_name", "stationary_gnss_demo"},
            {"output_dir", "data/logs/stationary_gnss_demo"},
            {"trajectory",
             {{"type", "stationary"},
              {"duration_s", 60.0},
              {"dt_s", 1.0},
              {"p_e_m", {6378137.0, 0.0, 0.0}}}},
            {"gnss", {{"dt_s", 1.0}, {"sigma_h_m", 3.0}, {"sigma_v_m", 5.0}, {"seed", 42U}}},
            {"initialization",
             {{"type", "pva_error"},
              {"pva_error",
               {{"pos_m", {25.0, -15.0, 10.0}},
                {"vel_mps", {0.0, 0.0, 0.0}},
                {"rpy_be_rad", {0.0, 0.0, 0.0}}}},
              {"pva_cov",
               {{"diag",
                 {10000.0, 10000.0, 10000.0, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6}}}}}}};
}

[[nodiscard]] std::vector<double> identity_pva_cov_full()
{
    std::vector<double> values(81U, 0.0);
    for (std::size_t i = 0; i < 9U; ++i) {
        values.at((i * 9U) + i) = 1.0;
    }
    return values;
}

[[nodiscard]] nlohmann::json random_pva_runtime_config()
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("initialization").erase("pva_error");
    cfg.at("initialization").at("type") = "pva_random";
    cfg.at("initialization").at("pva_cov") = {{"full", identity_pva_cov_full()}};
    cfg.at("initialization").emplace("seed", 7U);
    return cfg;
}

} // namespace

TEST_CASE("Stationary GNSS runtime validator accepts the documented input shape")
{
    const auto cfg = valid_stationary_gnss_runtime_config();

    static_assert(SimulationAppConfigPolicy<StationaryGnssAppConfig>);
    static_assert(!SimulationAppConfigPolicy<DuplicateSensorIdConfig>);
    static_assert(!SimulationAppConfigPolicy<MissingTargetSensorConfig>);
    static_assert(EmulatorPolicy<StationaryGnssAppConfig::PrimaryGnssEmulator,
                                 StationaryGnssAppConfig::PrimaryGnssSensor,
                                 StationaryGnssAppConfig::Logger>);
    static_assert(!EmulatorPolicy<NotAnEmulator,
                                  StationaryGnssAppConfig::PrimaryGnssSensor,
                                  StationaryGnssAppConfig::Logger>);
    static_assert(EmulatorBindingPolicy<StationaryGnssAppConfig::PrimaryGnssBinding,
                                        StationaryGnssAppConfig::Logger>);
    static_assert(!EmulatorBindingPolicy<NotABinding, StationaryGnssAppConfig::Logger>);
    static_assert(EmulatorBindingTuplePolicy<StationaryGnssAppConfig::EmulatorBindings,
                                             StationaryGnssAppConfig::NavKit::Sensors,
                                             StationaryGnssAppConfig::Logger>);
    static_assert(
        NavInitializationProviderPolicy<StationaryGnssAppConfig::NavInitializationProvider>);
    static_assert(
        TransferAlignmentProviderPolicy<StationaryGnssAppConfig::TransferAlignmentProvider,
                                        StationaryGnssAppConfig::NavKit::Navigator>);
    static_assert(!EmulatorBindingTuplePolicy<DuplicateSensorIdConfig::EmulatorBindings,
                                              DuplicateSensorIdConfig::NavKit::Sensors,
                                              DuplicateSensorIdConfig::Logger>);
    static_assert(!EmulatorBindingTuplePolicy<MissingTargetSensorConfig::EmulatorBindings,
                                              MissingTargetSensorConfig::NavKit::Sensors,
                                              MissingTargetSensorConfig::Logger>);
    static_assert(emulator_binding_ids_unique_v<StationaryGnssAppConfig::EmulatorBindings>);
    static_assert(std::is_same_v<EmulatorFromId_t<StationaryGnssAppConfig::PrimaryGnssEmulator::Id,
                                                  StationaryGnssAppConfig::EmulatorBindings>,
                                 GnssEmulator<StationaryGnssAppConfig::PrimaryGnssEmulator::Id>>);

    CHECK_NOTHROW(validate_runtime_config<StationaryGnssAppConfig>(cfg));
}

TEST_CASE("Stationary GNSS runtime validator rejects missing required sections")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.erase("gnss");

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects missing initialization")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.erase("initialization");

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects unsupported initialization type")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("initialization").at("type") = "explicit_pva";

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects malformed initialization covariance")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("initialization").at("pva_cov") = {{"diag", {1.0, 2.0}}};

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects ambiguous initialization covariance")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("initialization").at("pva_cov") = {
        {"diag", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}}, {"full", identity_pva_cov_full()}};

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects disabled transfer alignment inputs")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.emplace("transfer_alignment", nlohmann::json{{"type", "pva_aiding"}});

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects unsupported sensor sections")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.emplace("imu", nlohmann::json::object());

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects invalid trajectory shape")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("trajectory").at("p_e_m") = {1.0, 2.0};

    CHECK_THROWS_AS(validate_runtime_config<StationaryGnssAppConfig>(cfg), std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator keeps numeric tuning runtime-configurable")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.at("trajectory").at("duration_s") = 5.0;
    cfg.at("gnss").at("sigma_h_m") = 0.0;
    cfg.at("gnss").at("sigma_v_m") = 0.0;
    cfg.at("initialization").at("pva_cov").at("diag").at(0) = 0.0;

    CHECK_NOTHROW(validate_runtime_config<StationaryGnssAppConfig>(cfg));
}

TEST_CASE("Explicit PVA initialization provider preserves stationary demo behavior")
{
    const auto cfg = valid_stationary_gnss_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);

    const auto nav_init =
        StationaryGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);

    CHECK(nav_init.time_s == doctest::Approx(0.0));
    CHECK(nav_init.p_e_m(0) == doctest::Approx(6378137.0 + 25.0));
    CHECK(nav_init.p_e_m(1) == doctest::Approx(-15.0));
    CHECK(nav_init.p_e_m(2) == doctest::Approx(10.0));
    CHECK(nav_init.v_e_mps.isZero());
    CHECK(nav_init.rpy_be_rad.isZero());
    CHECK(nav_init.pva_cov(0, 0) == doctest::Approx(10000.0));
    CHECK(nav_init.pva_cov(3, 3) == doctest::Approx(1.0e-6));
    CHECK(nav_init.pva_cov(6, 6) == doctest::Approx(1.0e-6));
}

TEST_CASE("Explicit PVA initialization provider accepts full row-major covariance")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    auto full_cov = identity_pva_cov_full();
    full_cov.at((0U * 9U) + 3U) = 0.25;
    full_cov.at((3U * 9U) + 0U) = 0.25;
    cfg.at("initialization").at("pva_cov") = {{"full", full_cov}};

    const auto trajectory = trajectory_run_from_json(cfg);
    const auto nav_init =
        StationaryGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);

    CHECK(nav_init.pva_cov(0, 0) == doctest::Approx(1.0));
    CHECK(nav_init.pva_cov(0, 3) == doctest::Approx(0.25));
    CHECK(nav_init.pva_cov(3, 0) == doctest::Approx(0.25));

    using NavKit = StationaryGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;
    NavKit::Navigator navigator;
    initialize_navigator<StateDef>(navigator, nav_init);
    CHECK(navigator.filter().covariance()(StateDef::Pos::i, StateDef::Vel::i) ==
          doctest::Approx(0.25));
    CHECK(navigator.filter().covariance()(StateDef::Vel::i, StateDef::Pos::i) ==
          doctest::Approx(0.25));
}

TEST_CASE("Random PVA initialization provider produces deterministic colored draws")
{
    const auto cfg = random_pva_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);

    static_assert(NavInitializationProviderPolicy<PvaRandomInitializationProvider>);
    CHECK_NOTHROW(PvaRandomInitializationProvider::validate_runtime_config(cfg));

    const auto first = PvaRandomInitializationProvider::initialize(cfg, trajectory);
    const auto second = PvaRandomInitializationProvider::initialize(cfg, trajectory);

    CHECK(first.p_e_m.isApprox(second.p_e_m));
    CHECK(first.v_e_mps.isApprox(second.v_e_mps));
    CHECK(first.rpy_be_rad.isApprox(second.rpy_be_rad));
    CHECK_FALSE((first.p_e_m - trajectory.initial_position_e_m).isZero());
    CHECK(first.pva_cov(0, 0) == doctest::Approx(1.0));
}

TEST_CASE("NavInitialization maps into the configured Navigator filter state")
{
    using NavKit = StationaryGnssAppConfig::NavKit;
    using StateDef = NavKit::StateDef;

    const auto cfg = valid_stationary_gnss_runtime_config();
    const auto trajectory = trajectory_run_from_json(cfg);
    const auto nav_init =
        StationaryGnssAppConfig::NavInitializationProvider::initialize(cfg, trajectory);

    NavKit::Navigator navigator;
    initialize_navigator<StateDef>(navigator, nav_init);

    const auto& filter = navigator.filter();
    CHECK(filter.state()(StateDef::Pos::i + 0) == doctest::Approx(6378137.0 + 25.0));
    CHECK(filter.state()(StateDef::Pos::i + 1) == doctest::Approx(-15.0));
    CHECK(filter.state()(StateDef::Pos::i + 2) == doctest::Approx(10.0));
    CHECK(filter.state().template segment<3>(StateDef::Vel::i).isZero());
    CHECK(filter.state().template segment<3>(StateDef::Att::i).isZero());
    CHECK(filter.covariance()(StateDef::Pos::i, StateDef::Pos::i) == doctest::Approx(10000.0));
    CHECK(filter.covariance()(StateDef::Vel::i, StateDef::Vel::i) == doctest::Approx(1.0e-6));
    CHECK(filter.covariance()(StateDef::Att::i, StateDef::Att::i) == doctest::Approx(1.0e-6));
}

} // namespace navkit::app_support::test
