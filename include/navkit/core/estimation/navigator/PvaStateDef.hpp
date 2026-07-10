// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

namespace navkit::core::estimation
{

struct PvaStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Rpy = Segment<6, 3>;

    static constexpr int N = 9;
};

using PvaState = State<PvaStateDef>;
using PvaCovariance = StateCov<PvaStateDef>;

static_assert(StateDefPolicy<PvaStateDef>);

} // namespace navkit::core::estimation
