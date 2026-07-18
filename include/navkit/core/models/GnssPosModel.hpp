// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/measurement/MeasurementModelBase.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateAccessors.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/math/Skew.hpp"
#include "navkit/core/math/Types.hpp"

#include <string_view>

namespace navkit::core::models
{

using navkit::core::estimation::MeasurementModelBase;
using navkit::core::estimation::segment;

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
class GnssPosModel : public MeasurementModelBase<GnssPosModel<StateDef>, StateDef, 3>
{
public:
    static constexpr std::string_view Name = "gnss_pos";

    using Base = MeasurementModelBase<GnssPosModel<StateDef>, StateDef, 3>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;

    struct ObservationContext
    {
        Mat3 R_e_m2{(Vec3{9.0, 9.0, 25.0}).asDiagonal()};
        Vec3 p_b_ant_b_m{Vec3::Zero()};
    };

    static H_t compute_h_impl(const State_t& x, const ObservationContext& ctx)
    {
        H_t H = H_t::Zero();
        H.template block<3, 3>(0, Error::Pos::i) = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
        if constexpr (requires {
                          typename Nominal::AttQuat;
                          typename Error::AttRotVec;
                      }) {
            const Vec3 lever_arm_e_m =
                navkit::core::estimation::q_b2e<StateDef>(x) * ctx.p_b_ant_b_m;
            H.template block<3, 3>(0, Error::AttRotVec::i) =
                -navkit::core::math::skew_symmetric(lever_arm_e_m);
        }
        return H;
    }

    static R_t compute_r_impl(const ObservationContext& ctx)
    {
        return ctx.R_e_m2;
    }

    static O_t obs_impl(const State_t& x, const ObservationContext& ctx)
    {
        return segment<typename Nominal::Pos>(x) +
               (navkit::core::estimation::q_b2e<StateDef>(x) * ctx.p_b_ant_b_m);
    }
};

} // namespace navkit::core::models
