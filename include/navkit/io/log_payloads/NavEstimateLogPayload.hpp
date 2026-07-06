// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/TruthSample.hpp"

namespace navkit::io
{

template<typename StateDef, typename Filter>
struct NavEstimateLogPayload
{
    double time_s{};
    const Filter& filter;
    const sim::TruthSample& truth;
};

} // namespace navkit::io
