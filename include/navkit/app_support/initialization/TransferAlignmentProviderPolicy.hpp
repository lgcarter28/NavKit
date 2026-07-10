// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <concepts>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<typename Candidate, typename Navigator>
concept TransferAlignmentProviderPolicy =
    requires(Navigator& navigator, const nlohmann::json& cfg, const TrajectoryRun& trajectory) {
        { Candidate::validate_runtime_config(cfg) } -> std::same_as<void>;
        {
            Candidate::template transfer_align<Navigator>(navigator, cfg, trajectory)
        } -> std::same_as<void>;
    };

} // namespace navkit::app_support
