// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::core::estimation
{

template<typename Filter>
struct UpdatePostFilter
{
    template<typename Sensor>
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

template<typename Filter>
struct UpdateAfterEachSensor
{
    template<typename Sensor>
    static void sensor_update(Filter& filter, const Sensor&)
    {
        filter.inject();
        filter.reset();
    }

    static void filter_update(Filter&)
    {
        // no-op
    }
};

} // namespace navkit::core::estimation
