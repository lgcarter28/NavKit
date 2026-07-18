// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/app_support/initialization/concrete/PvaDirectInitializationProvider.hpp"
#include "navkit/app_support/initialization/concrete/PvaExplicitInitializationProvider.hpp"
#include "navkit/app_support/initialization/concrete/PvaRandomInitializationProvider.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace navkit::app_support
{

struct PvaRuntimeInitializationProvider
{
    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const nlohmann::json& initialization = detail::require_object(cfg, "pva_initialization");
        const std::string type = detail::pva_initialization_type_from_json(initialization);

        if (type == PvaRandomInitializationProvider::runtime_type) {
            PvaRandomInitializationProvider::validate_runtime_config(cfg);
            return;
        }
        if (type == PvaExplicitInitializationProvider::runtime_type) {
            PvaExplicitInitializationProvider::validate_runtime_config(cfg);
            return;
        }
        if (type == PvaDirectInitializationProvider::runtime_type) {
            PvaDirectInitializationProvider::validate_runtime_config(cfg);
            return;
        }

        detail::throw_runtime_config_error("unsupported pva_initialization.type '" + type + "'");
    }

    [[nodiscard]] static PvaInitialization initialize(const nlohmann::json& cfg,
                                                      const TrajectoryRun& trajectory)
    {
        const nlohmann::json& initialization = cfg.at("pva_initialization");
        const std::string type = detail::pva_initialization_type_from_json(initialization);

        if (type == PvaRandomInitializationProvider::runtime_type) {
            return PvaRandomInitializationProvider::initialize(cfg, trajectory);
        }
        if (type == PvaExplicitInitializationProvider::runtime_type) {
            return PvaExplicitInitializationProvider::initialize(cfg, trajectory);
        }
        if (type == PvaDirectInitializationProvider::runtime_type) {
            return PvaDirectInitializationProvider::initialize(cfg, trajectory);
        }

        detail::throw_runtime_config_error("unsupported pva_initialization.type '" + type + "'");
    }
};

} // namespace navkit::app_support
