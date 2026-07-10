// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

struct NoTransferAlignmentProvider
{
    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        detail::reject_object_if_present(
            cfg,
            "transfer_alignment",
            "transfer alignment is disabled by the selected compile-time app config");
    }

    template<typename Navigator>
    static void
    transfer_align(Navigator& navigator, const nlohmann::json& cfg, const TrajectoryRun& trajectory)
    {
        static_cast<void>(navigator);
        static_cast<void>(cfg);
        static_cast<void>(trajectory);
    }
};

} // namespace navkit::app_support
