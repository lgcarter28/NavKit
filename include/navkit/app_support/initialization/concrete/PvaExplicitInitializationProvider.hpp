// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

struct PvaExplicitInitializationProvider
{
    static constexpr const char* runtime_type = "pva_error";

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const auto& initialization = detail::require_object(cfg, "initialization");
        detail::require_initialization_type(initialization, runtime_type);
        detail::validate_pva_error_shape(initialization);
        detail::validate_pva_covariance_shape(initialization);
    }

    [[nodiscard]] static NavInitialization initialize(const nlohmann::json& cfg,
                                                      const TrajectoryRun& trajectory)
    {
        const auto& initialization = cfg.at("initialization");

        NavInitialization nav_init = detail::base_nav_initialization(trajectory);
        const core::Vec3 reference_p_e_m = core::estimation::pos_e_m(nav_init.pva);
        detail::apply_pva_error(nav_init,
                                detail::pva_error_from_json(initialization, reference_p_e_m));
        nav_init.pva_cov = detail::pva_covariance_from_json(initialization, reference_p_e_m);
        return nav_init;
    }
};

} // namespace navkit::app_support
