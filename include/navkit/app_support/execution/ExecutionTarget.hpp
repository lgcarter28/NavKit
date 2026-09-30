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
 * \details A target selects one cohesive authority and interface arrangement. The
 * software-in-the-loop adapter is implemented. HWIL is a recognized,
 * fail-closed selection until an application build supplies its concrete transport/runtime
 * adapter; future flight or external-GNC adapters should add concrete target values only
 * when their actual lifecycle and transport contracts exist.
 */
enum class ExecutionTargetType
{
    Swil,
    Hwil,
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
    ExecutionTargetType type{ExecutionTargetType::Swil};
    ClockMode clock_mode{ClockMode::Simulated};
    core::RationalRate application_rate{};
};

} // namespace navkit::app_support
