#pragma once

#include "navkit/core/SensorModelBase.hpp"

namespace navkit
{
template<typename StateDef>
class BaroAltModel : public SensorModelBase<BaroAltModel<StateDef>, StateDef, 1>
{
public:
    using Base = SensorModelBase<BaroAltModel<StateDef>, StateDef, 1>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;

    struct NoiseContext
    {
        Scalar_t sigma_alt{2.0};
    };

    static H_t compute_h_impl(const State_t&)
    {
        H_t H = H_t::Zero();
        H(0, StateDef::Pos::i + 2) = -1.0;
        return H;
    }

    static R_t compute_r_impl(const NoiseContext& ctx)
    {
        R_t R = R_t::Zero();
        R(0, 0) = ctx.sigma_alt * ctx.sigma_alt;
        return R;
    }

    static O_t obs_impl(const State_t& x)
    {
        O_t out{};
        out(0) = x(StateDef::Pos::i + 2);
        return out;
    }
};

} // namespace navkit
