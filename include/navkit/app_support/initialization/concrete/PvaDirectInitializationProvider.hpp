// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

struct PvaDirectInitializationProvider
{
    static constexpr const char* runtime_type = "pva_direct";

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const nlohmann::json& initialization = detail::require_object(cfg, "pva_initialization");
        detail::require_pva_initialization_type(initialization, runtime_type);
        detail::validate_pva_direct_shape(initialization);
    }

    [[nodiscard]] static PvaInitialization initialize(const nlohmann::json& cfg,
                                                      const TrajectoryRun&)
    {
        return detail::pva_direct_from_json(cfg.at("pva_initialization"));
    }
};

} // namespace navkit::app_support
