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
    static void propagate(Filter&, SensorTuple&)
    {
        // default: preserve current measurement-only Navigator behavior
    }
};

} // namespace navkit::core::estimation
