// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"

namespace navkit::core::estimation
{

template<FilterCorrectionPolicy Filter>
struct UpdatePostFilter
{
    template<SensorPolicy Sensor>
    static void sensor_update(Filter&, const Sensor&)
    {
        // default: do nothing after each sensor
    }

    static void filter_update(Filter& filter)
    {
        filter.inject();
        filter.reset();
    }
};

template<FilterCorrectionPolicy Filter>
struct UpdateAfterEachSensor
{
    template<SensorPolicy Sensor>
    static void sensor_update(Filter& filter, const Sensor&)
    {
        if constexpr (requires { filter.pending_correction_valid(); }) {
            if (!filter.pending_correction_valid()) {
                return;
            }
        }
        filter.inject();
        filter.reset();
    }

    static void filter_update(Filter&)
    {
        // no-op
    }
};

} // namespace navkit::core::estimation
