// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/time/Clock.hpp"
#include "navkit/app_support/time/ClockMode.hpp"
#include "navkit/app_support/time/RealtimeClock.hpp"
#include "navkit/app_support/time/SimulatedClock.hpp"

#include <memory>

namespace navkit::app_support
{

/** Creates the selected app-support clock implementation. */
[[nodiscard]] inline std::unique_ptr<Clock> clock_from_mode(const ClockMode mode)
{
    switch (mode) {
    case ClockMode::Simulated:
        return std::make_unique<SimulatedClock>();
    case ClockMode::Realtime:
        return std::make_unique<RealtimeClock>();
    }
    return {};
}

} // namespace navkit::app_support
