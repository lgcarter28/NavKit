// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

struct PvaExplicitInitializationProvider
{
    static constexpr const char* runtime_type = "pva_error";

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const nlohmann::json& initialization = detail::require_object(cfg, "pva_initialization");
        detail::require_pva_initialization_type(initialization, runtime_type);
        detail::validate_pva_error_shape(initialization);
    }

    [[nodiscard]] static PvaInitialization initialize(const nlohmann::json& cfg,
                                                      const sim::TruthSample& initial_truth)
    {
        const nlohmann::json& initialization = cfg.at("pva_initialization");

        PvaInitialization pva_init = detail::base_pva_initialization(initial_truth);
        const core::Vec3 reference_p_e_m = core::estimation::pos_e_m(pva_init.pva);
        detail::apply_pva_error(pva_init,
                                detail::pva_error_from_json(initialization, reference_p_e_m));
        return pva_init;
    }
};

} // namespace navkit::app_support
