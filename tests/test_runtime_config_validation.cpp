// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "apps/navkit_sim/StationaryGnss.hpp"
#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/app_support/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/GnssEmulator.hpp"
#include "navkit/app_support/RuntimeConfigValidation.hpp"
#include "navkit/app_support/SimulationApp.hpp"
#include "test_main.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <tuple>
#include <type_traits>

namespace navkit::app_support::test
{

namespace
{

using StationaryGnssAppConfig = navkit::config::apps::navkit_sim::StationaryGnssConfig;

struct DuplicateSensorIdConfig
{
    using NavKit = StationaryGnssAppConfig::NavKit;
    using EmulatorBindings = std::tuple<
        navkit::app_support::
            EmulatorBinding<0U, navkit::app_support::GnssEmulator, NavKit::PrimaryGnssSensor>,
        navkit::app_support::
            EmulatorBinding<0U, navkit::app_support::GnssEmulator, NavKit::PrimaryGnssSensor>>;
};

struct UnknownSensor
{
    static constexpr SensorId Id = 99U;
};

struct MissingTargetSensorConfig
{
    using NavKit = StationaryGnssAppConfig::NavKit;
    using EmulatorBindings =
        std::tuple<navkit::app_support::
                       EmulatorBinding<99U, navkit::app_support::GnssEmulator, UnknownSensor>>;
};

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
            {"filter",
             {{"initial_position_offset_m", {25.0, -15.0, 10.0}},
              {"initial_position_sigma_m", 100.0}}}};
}

} // namespace

TEST_CASE("Stationary GNSS runtime validator accepts the documented input shape")
{
    const auto cfg = valid_stationary_gnss_runtime_config();

    static_assert(SimulationAppConfigPolicy<StationaryGnssAppConfig>);
    static_assert(!SimulationAppConfigPolicy<DuplicateSensorIdConfig>);
    static_assert(!SimulationAppConfigPolicy<MissingTargetSensorConfig>);
    static_assert(emulator_binding_ids_unique_v<StationaryGnssAppConfig::EmulatorBindings>);
    static_assert(std::is_same_v<EmulatorFromId_t<StationaryGnssAppConfig::primary_gnss_sensor_id,
                                                  StationaryGnssAppConfig::EmulatorBindings>,
                                 GnssEmulator>);

    CHECK_NOTHROW(validate_runtime_config<StationaryGnssAppConfig>(cfg));
}

TEST_CASE("Stationary GNSS runtime validator rejects missing required sections")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.erase("gnss");

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
    cfg.at("filter").at("initial_position_sigma_m") = 0.0;

    CHECK_NOTHROW(validate_runtime_config<StationaryGnssAppConfig>(cfg));
}

} // namespace navkit::app_support::test
