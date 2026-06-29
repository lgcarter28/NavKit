#pragma once

#include "navkit/core/Segment.hpp"
#include "navkit/core/SensorModelBase.hpp"

#include <string_view>

namespace navkit
{
template<typename StateDef>
class GnssVelModel : public SensorModelBase<GnssVelModel<StateDef>, StateDef, 3>
{
public:
    static constexpr std::string_view Name = "gnss_vel";

    using Base = SensorModelBase<GnssVelModel<StateDef>, StateDef, 3>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;

    struct NoiseContext
    {
        Scalar_t sigma_v{0.2};
    };

    static H_t compute_h_impl(const State_t&)
    {
        H_t H = H_t::Zero();
        H.template block<3, 3>(0, StateDef::Vel::i) = -Eigen::Matrix<Scalar_t, 3, 3>::Identity();
        return H;
    }

    static R_t compute_r_impl(const NoiseContext& ctx)
    {
        return (ctx.sigma_v * ctx.sigma_v) * R_t::Identity();
    }

    static O_t obs_impl(const State_t& x)
    {
        return segment<typename StateDef::Vel>(x);
    }
};

} // namespace navkit
