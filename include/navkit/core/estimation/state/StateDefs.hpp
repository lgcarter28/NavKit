// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

namespace navkit::core::estimation
{

struct InsGyroAccelBiasNominalStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using AttQuat = Segment<6, 4>;
    using GyroB = Segment<10, 3>;
    using AccB = Segment<13, 3>;

    static constexpr int N = 16;
};

struct InsGyroAccelBiasErrorStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using AttRotVec = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using AccB = Segment<12, 3>;

    static constexpr int N = 15;
};

struct InsGyroAccelBiasStateDef
{
    using Nominal = InsGyroAccelBiasNominalStateDef;
    using Error = InsGyroAccelBiasErrorStateDef;
};

static_assert(StateDefPolicy<InsGyroAccelBiasNominalStateDef>);
static_assert(StateDefPolicy<InsGyroAccelBiasErrorStateDef>);
static_assert(StateSpaceDefPolicy<InsGyroAccelBiasStateDef>);

} // namespace navkit::core::estimation
