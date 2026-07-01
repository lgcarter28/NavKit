// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/common/Config.hpp"
#include "navkit/core/state/Segment.hpp"
#include "navkit/core/state/StateDefPolicy.hpp"

namespace navkit
{

struct InsStateDef
{
    using Scalar_t = navkit::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB = Segment<15, 3>;
    using AccSf = Segment<18, 3>;

    static constexpr int N = 21;
};

struct GnssTcStateDef
{
    using Scalar_t = navkit::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB = Segment<15, 3>;
    using AccSf = Segment<18, 3>;
    using ClkB = Segment<21, 1>;
    using ClkD = Segment<22, 1>;

    static constexpr int N = 23;
};

static_assert(StateDefPolicy<InsStateDef>);
static_assert(StateDefPolicy<GnssTcStateDef>);

} // namespace navkit
