// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorId.hpp"

namespace navkit::app_support
{

using SensorId = navkit::core::estimation::SensorId;

template<typename Emulator, typename Sensor>
struct EmulatorBinding
{
    static constexpr SensorId Id = Emulator::Id;
    using Emulator_t = Emulator;
    using Sensor_t = Sensor;

    static_assert(Emulator::Id == Sensor::Id,
                  "EmulatorBinding requires Emulator::Id to match Sensor::Id.");
};

} // namespace navkit::app_support
