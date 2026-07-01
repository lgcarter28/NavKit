// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/measurement/MeasurementModelBase.hpp"
#include "navkit/core/state/Segment.hpp"

#include <string_view>

namespace navkit
{

template<typename StateDef>
class GnssPosModel : public MeasurementModelBase<GnssPosModel<StateDef>, StateDef, 3>
{
public:
    static constexpr std::string_view Name = "gnss_pos";

    using Base = MeasurementModelBase<GnssPosModel<StateDef>, StateDef, 3>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;

    struct NoiseContext
    {
        Scalar_t sigma_h{3.0};
        Scalar_t sigma_v{5.0};
    };

    static H_t compute_h_impl(const State_t&)
    {
        H_t H = H_t::Zero();
        H.template block<3, 3>(0, StateDef::Pos::i) = -Eigen::Matrix<Scalar_t, 3, 3>::Identity();
        return H;
    }

    static R_t compute_r_impl(const NoiseContext& ctx)
    {
        R_t R = R_t::Zero();
        R(0, 0) = ctx.sigma_h * ctx.sigma_h;
        R(1, 1) = ctx.sigma_h * ctx.sigma_h;
        R(2, 2) = ctx.sigma_v * ctx.sigma_v;
        return R;
    }

    static O_t obs_impl(const State_t& x)
    {
        return segment<typename StateDef::Pos>(x);
    }
};

} // namespace navkit
