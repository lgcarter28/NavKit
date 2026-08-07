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
    [[nodiscard]] static typename Filter::AppliedCorrection_t sensor_update(Filter&, const Sensor&)
    {
        return {};
    }

    [[nodiscard]] static typename Filter::AppliedCorrection_t filter_update(Filter& filter)
    {
        const typename Filter::AppliedCorrection_t correction = filter.inject();
        filter.reset();
        return correction;
    }
};

template<FilterCorrectionPolicy Filter>
struct UpdateAfterEachSensor
{
    template<SensorPolicy Sensor>
    [[nodiscard]] static typename Filter::AppliedCorrection_t sensor_update(Filter& filter,
                                                                            const Sensor&)
    {
        if (!filter.pending_correction_valid()) {
            return {};
        }
        const typename Filter::AppliedCorrection_t correction = filter.inject();
        filter.reset();
        return correction;
    }

    [[nodiscard]] static typename Filter::AppliedCorrection_t filter_update(Filter&)
    {
        return {};
    }
};

} // namespace navkit::core::estimation
