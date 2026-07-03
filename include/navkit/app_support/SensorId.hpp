// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorId.hpp"

namespace navkit::app_support
{

using SensorId = navkit::core::estimation::SensorId;

template<SensorId IdValue, typename Emulator, typename Sensor>
struct EmulatorBinding
{
    static constexpr SensorId Id = IdValue;
    using Emulator_t = Emulator;
    using Sensor_t = Sensor;
};

} // namespace navkit::app_support
