// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/app_support/EmulatorPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"
#include "navkit/io/RunLogger.hpp"

#include <concepts>

namespace navkit::app_support
{

template<typename Candidate>
concept EmulatorBindingPolicy =
    requires {
        { Candidate::Id } -> std::convertible_to<SensorId>;
        typename Candidate::Emulator_t;
        typename Candidate::Sensor_t;
    } && navkit::core::estimation::SensorPolicy<typename Candidate::Sensor_t> &&
    EmulatorPolicy<typename Candidate::Emulator_t, typename Candidate::Sensor_t, io::RunLogger> &&
    (Candidate::Id == Candidate::Emulator_t::Id) && (Candidate::Id == Candidate::Sensor_t::Id);

} // namespace navkit::app_support
