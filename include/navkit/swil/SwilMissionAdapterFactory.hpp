// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/execution/ExecutionTarget.hpp"
#include "navkit/app_support/execution/ExecutionTargetJson.hpp"
#include "navkit/app_support/execution/MissionAdapter.hpp"
#include "navkit/app_support/logging/RuntimeLogger.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/swil/SwilAppConfigPolicy.hpp"
#include "navkit/swil/SwilMissionAdapter.hpp"
#include "navkit/swil/SwilRuntimeConfigValidation.hpp"

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::swil
{

/**
 * \brief Fixed-target factory selected by the `navkit_swil` executable.
 *
 * \details The factory owns the complete SWIL compile-time and runtime configuration boundary.
 * It constructs the concrete adapter behind the target-neutral `MissionAdapter` interface.
 * Target selection therefore lives in the thin executable rather than the product config.
 */
struct SwilMissionAdapterFactory
{
    inline static constexpr app_support::ExecutionTargetType target_type =
        app_support::ExecutionTargetType::Swil;
    inline static constexpr std::string_view application_name = "navkit_swil";

    template<SwilAppConfigPolicy Config>
    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        swil::validate_swil_runtime_config<Config>(cfg);
    }

    template<SwilAppConfigPolicy Config>
    [[nodiscard]] static std::unique_ptr<app_support::MissionAdapter<
        typename app_support::NavKitConfig_t<Config>::Navigator,
        app_support::RuntimeLogger<app_support::NavKitConfig_t<Config>>>>
    create(const nlohmann::json& cfg,
           const std::filesystem::path& source_base_dir,
           const app_support::RunSettings& run_settings)
    {
        return std::make_unique<SwilMissionAdapter<Config>>(cfg, source_base_dir, run_settings);
    }
};

} // namespace navkit::swil
