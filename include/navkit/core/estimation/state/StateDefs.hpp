// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

namespace navkit::core::estimation
{

struct InsStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using AccB = Segment<12, 3>;

    static constexpr int N = 15;
};

struct DefaultInsStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using AccB = Segment<12, 3>;

    static constexpr int N = 15;

    using Quat = Segment<6, 4>;
    using GyroBias = Segment<10, 3>;
    using AccelBias = Segment<13, 3>;

    static constexpr int NominalN = 16;
};

struct GnssTcStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using AccB = Segment<12, 3>;
    using ClkB = Segment<15, 1>;
    using ClkD = Segment<16, 1>;

    static constexpr int N = 17;
};

static_assert(StateDefPolicy<InsStateDef>);
static_assert(StateDefPolicy<DefaultInsStateDef>);
static_assert(StateDefPolicy<GnssTcStateDef>);

} // namespace navkit::core::estimation
