// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/measurement/MeasurementModelBase.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateAccessors.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/math/Skew.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <string_view>

namespace navkit::core::models
{

using navkit::core::estimation::MeasurementModelBase;
using navkit::core::estimation::segment;

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
class GnssVelModel : public MeasurementModelBase<GnssVelModel<StateDef>, StateDef, 3>
{
public:
    static constexpr std::string_view Name = "gnss_vel";

    using Base = MeasurementModelBase<GnssVelModel<StateDef>, StateDef, 3>;
    using State_t = typename Base::State_t;
    using H_t = typename Base::H_t;
    using R_t = typename Base::R_t;
    using O_t = typename Base::O_t;
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;

    struct ObservationContext
    {
        Mat3 R_e_m2ps2{(Vec3{0.04, 0.04, 0.04}).asDiagonal()};
        Vec3 p_b_ant_b_m{Vec3::Zero()};
        Vec3 omega_ie_e_radps{Vec3::Zero()};
        Vec3 omega_ib_b_meas_radps{Vec3::Zero()};
        bool imu_angular_rate_valid{false};
    };

    static H_t compute_h_impl(const State_t& x, const ObservationContext& ctx)
    {
        H_t H = H_t::Zero();
        H.template block<3, 3>(0, Error::Vel::i) = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
        if constexpr (requires { typename Nominal::AttQuat; }) {
            const Eigen::Quaternion<Scalar_t> q_b2e = navkit::core::estimation::q_b2e<StateDef>(x);
            const Mat3 C_b2e = q_b2e.toRotationMatrix();
            const Vec3 omega_eb_b_radps = estimated_omega_eb_b_radps(x, ctx);
            const Vec3 lever_arm_velocity_e_mps = C_b2e * omega_eb_b_radps.cross(ctx.p_b_ant_b_m);

            if constexpr (requires { typename Error::AttRotVec; }) {
                H.template block<3, 3>(0, Error::AttRotVec::i) =
                    -navkit::core::math::skew_symmetric(lever_arm_velocity_e_mps);
                if (ctx.imu_angular_rate_valid) {
                    H.template block<3, 3>(0, Error::AttRotVec::i) +=
                        C_b2e * navkit::core::math::skew_symmetric(ctx.p_b_ant_b_m) *
                        C_b2e.transpose() *
                        navkit::core::math::skew_symmetric(ctx.omega_ie_e_radps);
                }
            }

            if constexpr (requires { typename Error::GyroB; }) {
                if (ctx.imu_angular_rate_valid) {
                    H.template block<3, 3>(0, Error::GyroB::i) =
                        C_b2e * navkit::core::math::skew_symmetric(ctx.p_b_ant_b_m);
                }
            }
        }
        return H;
    }

    static R_t compute_r_impl(const ObservationContext& ctx)
    {
        return ctx.R_e_m2ps2;
    }

    static O_t obs_impl(const State_t& x, const ObservationContext& ctx)
    {
        const Eigen::Quaternion<Scalar_t> q_b2e = navkit::core::estimation::q_b2e<StateDef>(x);
        const Vec3 omega_eb_b_radps = estimated_omega_eb_b_radps(x, ctx);
        return segment<typename Nominal::Vel>(x) +
               (q_b2e * omega_eb_b_radps.cross(ctx.p_b_ant_b_m));
    }

private:
    static Vec3 estimated_omega_eb_b_radps(const State_t& x, const ObservationContext& ctx)
    {
        if (!ctx.imu_angular_rate_valid) {
            return Vec3::Zero();
        }

        Vec3 gyro_bias_b_radps = Vec3::Zero();
        if constexpr (requires { typename Nominal::GyroB; }) {
            gyro_bias_b_radps = segment<typename Nominal::GyroB>(x);
        }
        const Eigen::Quaternion<Scalar_t> q_b2e = navkit::core::estimation::q_b2e<StateDef>(x);
        return ctx.omega_ib_b_meas_radps - gyro_bias_b_radps -
               (q_b2e.conjugate() * ctx.omega_ie_e_radps);
    }
};

} // namespace navkit::core::models
