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

    // Current TXA convention is ECEF/body:
    // - position resolved in ECEF, meters
    // - velocity resolved in ECEF, meters/second
    // - roll/pitch/yaw parameterizing the ECEF-to-body transform, radians
    // - angular rate of body with respect to inertial, resolved in body, radians/second
    // - specific force of body with respect to inertial, resolved in body, meters/second^2
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Rpy = Segment<6, 3>;
    using AngularRate = Segment<9, 3>;
    using SpecificForce = Segment<12, 3>;

    static constexpr int N = 15;
};

using TxaState = State<TxaStateDef>;
using TxaCovariance = StateCov<TxaStateDef>;

inline auto pos_e_m(TxaState& txa)
{
    return segment<TxaStateDef::Pos>(txa);
}

inline auto pos_e_m(const TxaState& txa)
{
    return segment<TxaStateDef::Pos>(txa);
}

inline auto vel_e_mps(TxaState& txa)
{
    return segment<TxaStateDef::Vel>(txa);
}

inline auto vel_e_mps(const TxaState& txa)
{
    return segment<TxaStateDef::Vel>(txa);
}

inline auto rpy_e2b_rad(TxaState& txa)
{
    return segment<TxaStateDef::Rpy>(txa);
}

inline auto rpy_e2b_rad(const TxaState& txa)
{
    return segment<TxaStateDef::Rpy>(txa);
}

inline auto angular_rate_ib_b_radps(TxaState& txa)
{
    return segment<TxaStateDef::AngularRate>(txa);
}

inline auto angular_rate_ib_b_radps(const TxaState& txa)
{
    return segment<TxaStateDef::AngularRate>(txa);
}

inline auto specific_force_ib_b_mps2(TxaState& txa)
{
    return segment<TxaStateDef::SpecificForce>(txa);
}

inline auto specific_force_ib_b_mps2(const TxaState& txa)
{
    return segment<TxaStateDef::SpecificForce>(txa);
}

static_assert(StateDefPolicy<TxaStateDef>);

} // namespace navkit::core::estimation
