// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/time/ClockMode.hpp"
#include "navkit/core/time/RationalRate.hpp"

namespace navkit::app_support
{

/**
 * \brief Identifies the application adapter that owns execution of a runtime scenario.
 *
 * \details A target selects one cohesive authority and interface arrangement. Only the
 * NavKit-owned software-in-the-loop adapter is currently implemented; future HWIL,
 * flight, or external-GNC adapters should add concrete target values when their actual
 * lifecycle and transport contracts exist.
 */
enum class ExecutionTargetType
{
    NavKitSwil,
};

/**
 * \brief Runtime settings for the selected application execution target.
 *
 * \details The execution target owns the planned application cadence and its mapping to
 * wall-clock time. Simulation truth, plant behavior, and controller-feedback selection
 * remain separate simulation concerns rather than execution-target settings.
 */
struct ExecutionTargetSettings
{
    ExecutionTargetType type{ExecutionTargetType::NavKitSwil};
    ClockMode clock_mode{ClockMode::Simulated};
    core::RationalRate application_rate{};
};

} // namespace navkit::app_support
