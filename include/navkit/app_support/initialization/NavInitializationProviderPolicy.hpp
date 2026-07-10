// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <concepts>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<typename Candidate>
concept NavInitializationProviderPolicy =
    requires(const nlohmann::json& cfg, const TrajectoryRun& trajectory) {
        { Candidate::validate_runtime_config(cfg) } -> std::same_as<void>;
        { Candidate::initialize(cfg, trajectory) } -> std::same_as<NavInitialization>;
    };

} // namespace navkit::app_support
