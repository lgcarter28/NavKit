// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/measurement/MeasurementModelBase.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <string_view>

namespace navkit::core::models
{

using navkit::core::estimation::MeasurementModelBase;
using navkit::core::estimation::segment;

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
class BaroAltModel : public MeasurementModelBase<BaroAltModel<StateDef>, StateDef, 1>
{
public:
    static constexpr std::string_view Name = "baro_alt";

    using Base = MeasurementModelBase<BaroAltModel<StateDef>, StateDef, 1>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;

    struct ObservationContext
    {
        Scalar_t sigma_alt{2.0};
    };

    static H_t compute_h_impl(const State_t&, const ObservationContext&)
    {
        H_t H = H_t::Zero();
        H(0, Error::Pos::i + 2) = -1.0;
        return H;
    }

    static R_t compute_r_impl(const ObservationContext& ctx)
    {
        R_t R = R_t::Zero();
        R(0, 0) = ctx.sigma_alt * ctx.sigma_alt;
        return R;
    }

    static O_t obs_impl(const State_t& x, const ObservationContext&)
    {
        O_t out{};
        out(0) = segment<typename Nominal::Pos>(x).z();
        return out;
    }
};

} // namespace navkit::core::models
