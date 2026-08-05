// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <string_view>

namespace navkit::app_support
{

/** Application-selected source for source-agnostic Guidance and Autopilot state inputs. */
enum class ControlStateSourceMode
{
    NavigationEstimate,
    TruthPassthrough,
};

[[nodiscard]] inline bool control_state_source_mode_from_string(const std::string_view value,
                                                                ControlStateSourceMode& mode)
{
    if (value == "navigation_estimate") {
        mode = ControlStateSourceMode::NavigationEstimate;
        return true;
    }
    if (value == "truth_passthrough") {
        mode = ControlStateSourceMode::TruthPassthrough;
        return true;
    }
    return false;
}

} // namespace navkit::app_support
