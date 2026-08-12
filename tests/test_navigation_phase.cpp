// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/navigation/NavigationPhaseJson.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/products/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
#include "test_main.hpp"

#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

using NavKit = navkit::config::navkit::EcefInsGnssLcGyroAccelBiasDefaultConfig;
using PositionSensor = NavKit::PrimaryGnssPositionSensor;
using VelocitySensor = NavKit::PrimaryGnssVelocitySensor;

[[nodiscard]] nlohmann::json gate_phase(const bool enabled)
{
    return {
        {"sensors",
         {{"primary_gnss_position", {{"chi_square_acceptance", {{"enabled", enabled}}}}},
          {"primary_gnss_velocity", {{"chi_square_acceptance", {{"enabled", enabled}}}}}}},
    };
}

[[nodiscard]] nlohmann::json navigation_phase_runtime_config()
{
    return {
        {"gnss",
         {{"chi_square_acceptance",
           {{"position", {{"enabled", true}, {"probability", 0.9973}}},
            {"velocity", {{"enabled", true}, {"probability", 0.9973}}}}}}},
        {"mission",
         {{"phases",
           nlohmann::json::array({{{"id", "startup"}, {"navigation", gate_phase(false)}},
                                  {{"id", "cruise"}, {"navigation", gate_phase(true)}}})}}},
    };
}

} // namespace

TEST_CASE("Navigation phases resolve baseline probabilities and preserve mission order")
{
    nlohmann::json cfg = navigation_phase_runtime_config();
    cfg.at("mission")
        .at("phases")
        .at(0)
        .at("navigation")
        .at("sensors")
        .at("primary_gnss_position")
        .at("chi_square_acceptance")["probability"] = 0.95;

    const std::vector<NavigationPhase> phases = navigation_phases_from_mission_json(cfg);
    REQUIRE(phases.size() == 2U);
    CHECK(phases.at(0).id == "startup");
    CHECK(phases.at(1).id == "cruise");
    REQUIRE(phases.at(0).primary_gnss_position.probability.has_value());
    REQUIRE(phases.at(0).primary_gnss_velocity.probability.has_value());
    REQUIRE(phases.at(1).primary_gnss_position.probability.has_value());
    REQUIRE(phases.at(1).primary_gnss_velocity.probability.has_value());
    CHECK(*phases.at(0).primary_gnss_position.probability == doctest::Approx(0.95));
    CHECK(*phases.at(0).primary_gnss_velocity.probability == doctest::Approx(0.9973));
    CHECK(*phases.at(1).primary_gnss_position.probability == doctest::Approx(0.9973));
    CHECK(*phases.at(1).primary_gnss_velocity.probability == doctest::Approx(0.9973));
}

TEST_CASE("Every mission phase must own complete Navigation controls")
{
    nlohmann::json cfg = navigation_phase_runtime_config();
    cfg.at("mission").at("phases").at(1).erase("navigation");

    CHECK_THROWS_AS(static_cast<void>(navigation_phases_from_mission_json(cfg)),
                    std::runtime_error);
}

TEST_CASE("Navigation phase probabilities must be complete and valid")
{
    nlohmann::json missing = navigation_phase_runtime_config();
    missing.at("gnss").at("chi_square_acceptance").at("position").erase("probability");
    CHECK_THROWS_AS(static_cast<void>(navigation_phases_from_mission_json(missing)),
                    std::runtime_error);

    nlohmann::json out_of_range = navigation_phase_runtime_config();
    out_of_range.at("mission")
        .at("phases")
        .at(1)
        .at("navigation")
        .at("sensors")
        .at("primary_gnss_velocity")
        .at("chi_square_acceptance")["probability"] = 1.0;
    CHECK_THROWS_AS(static_cast<void>(navigation_phases_from_mission_json(out_of_range)),
                    std::runtime_error);
}

TEST_CASE("Navigation phase application restores complete configured gate state")
{
    nlohmann::json cfg = navigation_phase_runtime_config();
    cfg.at("mission")
        .at("phases")
        .at(0)
        .at("navigation")
        .at("sensors")
        .at("primary_gnss_position")
        .at("chi_square_acceptance")["probability"] = 0.95;
    cfg.at("mission")
        .at("phases")
        .at(0)
        .at("navigation")
        .at("sensors")
        .at("primary_gnss_velocity")
        .at("chi_square_acceptance")["probability"] = 0.96;
    const std::vector<NavigationPhase> phases = navigation_phases_from_mission_json(cfg);
    std::unique_ptr<NavKit::Navigator> navigator_storage = std::make_unique<NavKit::Navigator>();
    NavKit::Navigator& navigator = *navigator_storage;

    REQUIRE(apply_navigation_phase<NavKit>(phases.at(0), navigator));
    constexpr std::size_t position_sensor_index =
        core::estimation::SensorIndexFromId_v<PositionSensor::Id, NavKit::Sensors>;
    constexpr std::size_t velocity_sensor_index =
        core::estimation::SensorIndexFromId_v<VelocitySensor::Id, NavKit::Sensors>;
    const PositionSensor& position_sensor = navigator.template sensor<position_sensor_index>();
    const VelocitySensor& velocity_sensor = navigator.template sensor<velocity_sensor_index>();
    CHECK_FALSE(position_sensor.innovation_gate().enabled());
    CHECK_FALSE(velocity_sensor.innovation_gate().enabled());
    CHECK(position_sensor.innovation_gate().probability() == doctest::Approx(0.95));
    CHECK(velocity_sensor.innovation_gate().probability() == doctest::Approx(0.96));

    REQUIRE(apply_navigation_phase<NavKit>(phases.at(1), navigator));
    CHECK(position_sensor.innovation_gate().enabled());
    CHECK(velocity_sensor.innovation_gate().enabled());
    CHECK(position_sensor.innovation_gate().probability() == doctest::Approx(0.9973));
    CHECK(velocity_sensor.innovation_gate().probability() == doctest::Approx(0.9973));
}

} // namespace navkit::app_support::test
