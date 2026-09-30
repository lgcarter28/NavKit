// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <concepts>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<typename Candidate>
concept NavInitializationProviderPolicy =
    requires(const nlohmann::json& cfg, const sim::TruthSample& initial_truth) {
        { Candidate::validate_runtime_config(cfg) } -> std::same_as<void>;
        { Candidate::initialize(cfg, initial_truth) } -> std::same_as<PvaInitialization>;
    };

} // namespace navkit::app_support
