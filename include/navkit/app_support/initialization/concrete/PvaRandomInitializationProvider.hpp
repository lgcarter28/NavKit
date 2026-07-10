// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <cstdint>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

struct PvaRandomInitializationProvider
{
    static constexpr const char* runtime_type = "pva_random";

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const auto& initialization = detail::require_object(cfg, "initialization");
        detail::require_initialization_type(initialization, runtime_type);
        detail::validate_pva_covariance_shape(initialization);
        detail::require_optional_unsigned_integer(initialization, "seed");
    }

    [[nodiscard]] static NavInitialization initialize(const nlohmann::json& cfg,
                                                      const TrajectoryRun& trajectory)
    {
        const auto& initialization = cfg.at("initialization");
        const auto covariance = detail::pva_covariance_from_json(initialization);
        const auto seed = initialization.value("seed", std::uint64_t{0});

        NavInitialization nav_init = detail::base_nav_initialization(trajectory);
        detail::apply_pva_error(nav_init, detail::sample_pva_error(covariance, seed));
        nav_init.pva_cov = covariance;
        return nav_init;
    }
};

} // namespace navkit::app_support
