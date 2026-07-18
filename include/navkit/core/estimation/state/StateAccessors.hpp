// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <Eigen/Geometry>

namespace navkit::core::estimation
{

template<StateSpaceDefPolicy StateDef>
[[nodiscard]] Eigen::Quaternion<Scalar_t> q_b2e(const NominalState<StateDef>& x)
{
    using Nominal = typename StateDef::Nominal;
    static_assert(
        requires { typename Nominal::AttQuat; },
        "q_b2e<StateDef>() requires StateDef::Nominal::AttQuat");

    const Eigen::Matrix<Scalar_t, 4, 1> q_segment = segment<typename Nominal::AttQuat>(x);
    Eigen::Quaternion<Scalar_t> quaternion{q_segment(0), q_segment(1), q_segment(2), q_segment(3)};
    if (quaternion.norm() <= 0.0) {
        quaternion.setIdentity();
    }
    return navkit::core::math::normalized_with_positive_scalar(quaternion);
}

template<StateSpaceDefPolicy StateDef>
void set_q_b2e(NominalState<StateDef>& x, const Eigen::Quaternion<Scalar_t>& quaternion)
{
    using Nominal = typename StateDef::Nominal;
    static_assert(
        requires { typename Nominal::AttQuat; },
        "set_q_b2e<StateDef>() requires StateDef::Nominal::AttQuat");

    const Eigen::Quaternion<Scalar_t> normalized =
        navkit::core::math::normalized_with_positive_scalar(quaternion.normalized());
    segment<typename Nominal::AttQuat>(x) << normalized.w(), normalized.x(), normalized.y(),
        normalized.z();
}

} // namespace navkit::core::estimation
