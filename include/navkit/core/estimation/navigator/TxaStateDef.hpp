// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

namespace navkit::core::estimation
{

struct TxaStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Rpy = Segment<6, 3>;
    using AngularRate = Segment<9, 3>;
    using SpecificForce = Segment<12, 3>;

    static constexpr int N = 15;
};

using TxaState = State<TxaStateDef>;
using TxaCovariance = StateCov<TxaStateDef>;

static_assert(StateDefPolicy<TxaStateDef>);

} // namespace navkit::core::estimation
