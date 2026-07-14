// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<typename Derived, StateSpaceDefPolicy StateDef, int M_>
class MeasurementModelBase
{
public:
    static constexpr int M = M_;
    using Error = typename StateDef::Error;
    using State_t = NominalState<StateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using H_t = Eigen::Matrix<Scalar_t, M, Error::N>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using K_t = Eigen::Matrix<Scalar_t, Error::N, M>;

    template<typename NoiseContext>
    static R_t compute_r(const NoiseContext& ctx)
    {
        return Derived::compute_r_impl(ctx);
    }

    static H_t compute_h(const State_t& x)
    {
        return Derived::compute_h_impl(x);
    }

    static O_t obs(const State_t& x)
    {
        return Derived::obs_impl(x);
    }
};

} // namespace navkit::core::estimation
