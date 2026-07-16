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

    // Current PVA convention is ECEF-only:
    // - position resolved in ECEF, meters
    // - velocity resolved in ECEF, meters/second
    // - roll/pitch/yaw parameterizing the body-to-ECEF transform, radians
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Rpy = Segment<6, 3>;

    static constexpr int N = 9;
};

struct PvaErrorStateDef
{
    using Scalar_t = navkit::core::Scalar_t;

    // Current PVA error convention is ECEF-only after runtime parsing:
    // - position error resolved in ECEF, meters
    // - velocity error resolved in ECEF, meters/second
    // - small-angle attitude-error vector resolved in ECEF, radians
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using AttRotVec = Segment<6, 3>;

    static constexpr int N = 9;
};

using PvaState = State<PvaStateDef>;
using PvaErrorState = State<PvaErrorStateDef>;
using PvaCovariance = StateCov<PvaErrorStateDef>;

inline auto pos_e_m(PvaState& pva)
{
    return segment<PvaStateDef::Pos>(pva);
}

inline auto pos_e_m(const PvaState& pva)
{
    return segment<PvaStateDef::Pos>(pva);
}

inline auto pos_error_e_m(PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::Pos>(pva);
}

inline auto pos_error_e_m(const PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::Pos>(pva);
}

inline auto vel_e_mps(PvaState& pva)
{
    return segment<PvaStateDef::Vel>(pva);
}

inline auto vel_e_mps(const PvaState& pva)
{
    return segment<PvaStateDef::Vel>(pva);
}

inline auto vel_error_e_mps(PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::Vel>(pva);
}

inline auto vel_error_e_mps(const PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::Vel>(pva);
}

inline auto rpy_b2e_rad(PvaState& pva)
{
    return segment<PvaStateDef::Rpy>(pva);
}

inline auto rpy_b2e_rad(const PvaState& pva)
{
    return segment<PvaStateDef::Rpy>(pva);
}

inline auto att_rotvec_e_rad(PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::AttRotVec>(pva);
}

inline auto att_rotvec_e_rad(const PvaErrorState& pva)
{
    return segment<PvaErrorStateDef::AttRotVec>(pva);
}

static_assert(StateDefPolicy<PvaStateDef>);
static_assert(StateDefPolicy<PvaErrorStateDef>);

} // namespace navkit::core::estimation
