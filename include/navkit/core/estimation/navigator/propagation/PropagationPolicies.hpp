// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"

namespace navkit::core::estimation
{

struct NoOpPropagation
{
    template<FilterPolicy Filter, SensorCollectionPolicy SensorTuple>
    static void process_strapdown_integration(Filter&, SensorTuple&)
    {
        // default: no nominal IMU mechanization for measurement-only products
    }

    template<FilterPolicy Filter, SensorCollectionPolicy SensorTuple>
    static void process_covariance(Filter&, SensorTuple&)
    {
        // default: no covariance prediction for measurement-only products
    }
};

} // namespace navkit::core::estimation
