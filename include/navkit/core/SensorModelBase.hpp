#pragma once

#include "navkit/core/State.hpp"

#include <Eigen/Dense>

namespace navkit
{

template<typename Derived, typename StateDef, int M_>
class SensorModelBase
{
public:
    static constexpr int M = M_;
    using State_t = State<StateDef>;
    using O_t = Eigen::Matrix<Scalar_t, M, 1>;
    using H_t = Eigen::Matrix<Scalar_t, M, StateDef::N>;
    using R_t = Eigen::Matrix<Scalar_t, M, M>;
    using K_t = Eigen::Matrix<Scalar_t, StateDef::N, M>;

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

} // namespace navkit
