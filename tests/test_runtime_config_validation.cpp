// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/StationaryGnss.hpp"
#include "navkit/app_support/RuntimeConfigValidation.hpp"
#include "test_main.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>

namespace navkit::app_support::test
{

namespace
{

struct StationaryGnssAppConfig
{
    using NavKit = navkit::config::navkit::StationaryGnssConfig;
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

    CHECK_NOTHROW(validate_stationary_gnss_runtime_config<StationaryGnssAppConfig>(cfg));
}

TEST_CASE("Stationary GNSS runtime validator rejects missing required sections")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg.erase("gnss");

    CHECK_THROWS_AS(validate_stationary_gnss_runtime_config<StationaryGnssAppConfig>(cfg),
                    std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects unsupported sensor sections")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg["imu"] = nlohmann::json::object();

    CHECK_THROWS_AS(validate_stationary_gnss_runtime_config<StationaryGnssAppConfig>(cfg),
                    std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator rejects invalid trajectory shape")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg["trajectory"]["p_e_m"] = {1.0, 2.0};

    CHECK_THROWS_AS(validate_stationary_gnss_runtime_config<StationaryGnssAppConfig>(cfg),
                    std::runtime_error);
}

TEST_CASE("Stationary GNSS runtime validator keeps numeric tuning runtime-configurable")
{
    auto cfg = valid_stationary_gnss_runtime_config();
    cfg["trajectory"]["duration_s"] = 5.0;
    cfg["gnss"]["sigma_h_m"] = 0.0;
    cfg["gnss"]["sigma_v_m"] = 0.0;
    cfg["filter"]["initial_position_sigma_m"] = 0.0;

    CHECK_NOTHROW(validate_stationary_gnss_runtime_config<StationaryGnssAppConfig>(cfg));
}

} // namespace navkit::app_support::test
