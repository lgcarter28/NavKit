// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/mission/MissionRuntimeJson.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/products/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
#include "test_main.hpp"

#include <cstddef>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

using NavKit = navkit::config::navkit::EcefInsGnssLcGyroAccelBiasDefaultConfig;
using PositionSensor = NavKit::PrimaryGnssPositionSensor;

[[nodiscard]] NavigationPhase navigation_phase(const std::string& id,
                                               const bool enabled,
                                               const core::Scalar_t probability = 0.9973)
{
    const NavigationInnovationGatePhase gate{
        .probability = probability,
        .enabled = enabled,
    };
    return NavigationPhase{
        .id = id,
        .primary_gnss_position = gate,
        .primary_gnss_velocity = gate,
    };
}

[[nodiscard]] MissionRuntime mission_runtime()
{
    std::vector<MissionPhase> phases{};
    phases.push_back(MissionPhase{
        .id = "startup",
        .navigation = navigation_phase("startup", false),
        .allowed_transition_indices = {1U},
        .terminal = false,
    });
    phases.push_back(MissionPhase{
        .id = "cruise",
        .navigation = navigation_phase("cruise", true, 0.95),
        .allowed_transition_indices = {2U},
        .terminal = false,
    });
    phases.push_back(MissionPhase{
        .id = "complete",
        .navigation = navigation_phase("complete", true, 0.99),
        .allowed_transition_indices = {},
        .terminal = true,
    });
    return MissionRuntime{std::move(phases), 0U};
}

[[nodiscard]] nlohmann::json gate_json(const bool enabled)
{
    return {
        {"sensors",
         {{"primary_gnss_position", {{"chi_square_acceptance", {{"enabled", enabled}}}}},
          {"primary_gnss_velocity", {{"chi_square_acceptance", {{"enabled", enabled}}}}}}},
    };
}

[[nodiscard]] nlohmann::json mission_json()
{
    return {
        {"gnss",
         {{"chi_square_acceptance",
           {{"position", {{"enabled", true}, {"probability", 0.9973}}},
            {"velocity", {{"enabled", true}, {"probability", 0.9973}}}}}}},
        {"mission",
         {{"initial_phase_id", "startup"},
          {"cycle_policy", "reject"},
          {"phases",
           nlohmann::json::array(
               {{{"id", "startup"},
                 {"navigation", gate_json(false)},
                 {"transitions", nlohmann::json::array({{{"to", "cruise"}}})}},
                {{"id", "cruise"},
                 {"navigation", gate_json(true)},
                 {"terminal", {{"behavior", "run_until_mission_termination"}}}}})}}},
    };
}

} // namespace

TEST_CASE("Mission runtime resolves stable phase IDs")
{
    const MissionRuntime runtime = mission_runtime();
    REQUIRE(runtime.is_valid());
    CHECK(runtime.phase_count() == 3U);
    CHECK(runtime.initial_phase_index() == 0U);
    CHECK(runtime.initial_phase_id() == "startup");
    CHECK_FALSE(runtime.active_phase_is_terminal());

    std::size_t index = 99U;
    REQUIRE(runtime.phase_index("cruise", index));
    CHECK(index == 1U);
    CHECK_FALSE(runtime.phase_index("missing", index));
}

TEST_CASE("Mission runtime initializes once at the configured source phase")
{
    MissionRuntime runtime = mission_runtime();
    std::unique_ptr<NavKit::Navigator> navigator_storage = std::make_unique<NavKit::Navigator>();
    NavKit::Navigator& navigator = *navigator_storage;

    CHECK_FALSE(runtime.initialize<NavKit>("cruise", navigator));
    CHECK_FALSE(runtime.initialized());
    REQUIRE(runtime.initialize<NavKit>("startup", navigator));
    CHECK_FALSE(runtime.active_phase_is_terminal());
    REQUIRE(runtime.active_phase_index().has_value());
    CHECK(*runtime.active_phase_index() == 0U);
    REQUIRE(runtime.active_phase_id().has_value());
    CHECK(*runtime.active_phase_id() == "startup");
    CHECK_FALSE(runtime.initialize<NavKit>("startup", navigator));
}

TEST_CASE("Mission runtime accepts only declared transitions and rejects terminal advancement")
{
    MissionRuntime runtime = mission_runtime();
    std::unique_ptr<NavKit::Navigator> navigator_storage = std::make_unique<NavKit::Navigator>();
    NavKit::Navigator& navigator = *navigator_storage;
    REQUIRE(runtime.initialize<NavKit>("startup", navigator));

    REQUIRE(runtime.synchronize<NavKit>("startup", navigator));
    CHECK_FALSE(runtime.synchronize<NavKit>("complete", navigator));
    CHECK(*runtime.active_phase_index() == 0U);

    REQUIRE(runtime.synchronize<NavKit>("cruise", navigator));
    CHECK(*runtime.active_phase_index() == 1U);
    REQUIRE(runtime.synchronize<NavKit>("complete", navigator));
    CHECK(*runtime.active_phase_index() == 2U);
    CHECK(runtime.active_phase_is_terminal());
    CHECK_FALSE(runtime.synchronize<NavKit>("startup", navigator));
    CHECK(*runtime.active_phase_index() == 2U);
}

TEST_CASE("Mission runtime commits a phase only after Navigation application succeeds")
{
    std::vector<MissionPhase> phases{};
    phases.push_back(MissionPhase{
        .id = "valid",
        .navigation = navigation_phase("valid", false),
        .allowed_transition_indices = {1U},
        .terminal = false,
    });
    phases.push_back(MissionPhase{
        .id = "invalid",
        .navigation = navigation_phase("invalid", true, 1.0),
        .allowed_transition_indices = {},
        .terminal = true,
    });
    MissionRuntime runtime{std::move(phases), 0U};
    std::unique_ptr<NavKit::Navigator> navigator_storage = std::make_unique<NavKit::Navigator>();
    NavKit::Navigator& navigator = *navigator_storage;
    REQUIRE(runtime.initialize<NavKit>("valid", navigator));

    CHECK_FALSE(runtime.synchronize<NavKit>("invalid", navigator));
    REQUIRE(runtime.active_phase_index().has_value());
    CHECK(*runtime.active_phase_index() == 0U);

    constexpr std::size_t position_sensor_index =
        core::estimation::SensorIndexFromId_v<PositionSensor::Id, NavKit::Sensors>;
    const PositionSensor& position_sensor = navigator.template sensor<position_sensor_index>();
    CHECK_FALSE(position_sensor.innovation_gate().enabled());
}

TEST_CASE("Mission runtime JSON resolves transition targets once")
{
    MissionRuntime runtime = mission_runtime_from_json(mission_json());
    REQUIRE(runtime.is_valid());
    CHECK(runtime.phase_count() == 2U);

    std::size_t cruise_index = 99U;
    REQUIRE(runtime.phase_index("cruise", cruise_index));
    CHECK(cruise_index == 1U);

    std::unique_ptr<NavKit::Navigator> navigator_storage = std::make_unique<NavKit::Navigator>();
    NavKit::Navigator& navigator = *navigator_storage;
    REQUIRE(runtime.initialize<NavKit>("startup", navigator));
    REQUIRE(runtime.synchronize<NavKit>("cruise", navigator));
}

TEST_CASE("Mission runtime JSON rejects unreachable phases")
{
    nlohmann::json cfg = mission_json();
    cfg.at("mission").at("phases").push_back(
        {{"id", "orphan"},
         {"navigation", gate_json(true)},
         {"terminal", {{"behavior", "run_until_mission_termination"}}}});

    CHECK_THROWS_WITH_AS(static_cast<void>(mission_runtime_from_json(cfg)),
                         doctest::Contains("unreachable phase"),
                         std::runtime_error);
}

TEST_CASE("Mission runtime JSON rejects cycles when configured fail closed")
{
    nlohmann::json cfg = mission_json();
    nlohmann::json& cruise = cfg.at("mission").at("phases").at(1U);
    cruise.erase("terminal");
    cruise["transitions"] = nlohmann::json::array({{{"to", "startup"}}});

    CHECK_THROWS_WITH_AS(static_cast<void>(mission_runtime_from_json(cfg)),
                         doctest::Contains("cycle_policy is 'reject'"),
                         std::runtime_error);
}

TEST_CASE("Mission runtime JSON accepts declared cycles when explicitly allowed")
{
    nlohmann::json cfg = mission_json();
    cfg.at("mission").at("cycle_policy") = "allow";
    nlohmann::json& cruise = cfg.at("mission").at("phases").at(1U);
    cruise.erase("terminal");
    cruise["transitions"] = nlohmann::json::array({{{"to", "startup"}}});

    const MissionRuntime runtime = mission_runtime_from_json(cfg);
    CHECK(runtime.is_valid());
    CHECK(runtime.phase_count() == 2U);
}

} // namespace navkit::app_support::test
