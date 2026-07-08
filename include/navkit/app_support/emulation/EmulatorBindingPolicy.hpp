// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/EmulatorPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"

#include <concepts>

namespace navkit::app_support
{

template<typename Candidate, typename Logger>
concept EmulatorBindingPolicy =
    requires {
        { Candidate::Id } -> std::convertible_to<SensorId>;
        typename Candidate::Emulator_t;
        typename Candidate::Sensor_t;
    } && navkit::core::estimation::SensorPolicy<typename Candidate::Sensor_t> &&
    EmulatorPolicy<typename Candidate::Emulator_t, typename Candidate::Sensor_t, Logger> &&
    (Candidate::Id == Candidate::Emulator_t::Id) && (Candidate::Id == Candidate::Sensor_t::Id);

} // namespace navkit::app_support
