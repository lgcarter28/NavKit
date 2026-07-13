// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/math/Quaternion.hpp"

namespace navkit::core::estimation
{

template<typename Seg>
struct AdditiveInjection
{
    template<typename State_t>
    static void apply(State_t& x, const State_t& dx)
    {
        segment<Seg>(x) -= segment<Seg>(dx);
    }
};

template<typename StateDef>
struct InsInjectionPolicy
{
    template<typename NominalState_t, typename ErrorState_t>
    static void apply(NominalState_t& x, const ErrorState_t& dx)
    {
        apply_position(x, dx);
        apply_velocity(x, dx);
        apply_attitude(x, dx);
        if constexpr (requires { typename StateDef::GyroB; }) {
            apply_gyro_bias(x, dx);
        }
        if constexpr (requires { typename StateDef::GyroSf; }) {
            apply_if_present<typename StateDef::GyroSf>(x, dx);
        }
        if constexpr (requires { typename StateDef::AccB; }) {
            apply_accel_bias(x, dx);
        }
        if constexpr (requires { typename StateDef::AccSf; }) {
            apply_if_present<typename StateDef::AccSf>(x, dx);
        }
        if constexpr (requires { typename StateDef::ClkB; }) {
            apply_if_present<typename StateDef::ClkB>(x, dx);
        }
        if constexpr (requires { typename StateDef::ClkD; }) {
            apply_if_present<typename StateDef::ClkD>(x, dx);
        }
    }

private:
    template<typename NominalSegment,
             typename ErrorSegment,
             typename NominalState_t,
             typename ErrorState_t>
    static void apply_pair(NominalState_t& x, const ErrorState_t& dx)
    {
        segment<NominalSegment>(x) -= segment<ErrorSegment>(dx);
    }

    template<typename Segment, typename NominalState_t, typename ErrorState_t>
    static void apply_if_present(NominalState_t& x, const ErrorState_t& dx)
    {
        apply_pair<Segment, Segment>(x, dx);
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_position(NominalState_t& x, const ErrorState_t& dx)
    {
        apply_pair<typename StateDef::Pos, typename StateDef::Pos>(x, dx);
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_velocity(NominalState_t& x, const ErrorState_t& dx)
    {
        apply_pair<typename StateDef::Vel, typename StateDef::Vel>(x, dx);
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_attitude(NominalState_t& x, const ErrorState_t& dx)
    {
        if constexpr (requires { typename StateDef::Quat; }) {
            const auto q_segment = segment<typename StateDef::Quat>(x);
            const Eigen::Quaternion<Scalar_t> q_e2b{
                q_segment(0), q_segment(1), q_segment(2), q_segment(3)};
            const auto delta_q = navkit::core::math::quaternion_from_rotvec_rad(
                -segment<typename StateDef::Att>(dx));
            const auto q_next =
                navkit::core::math::normalized_with_positive_scalar(delta_q * q_e2b);
            segment<typename StateDef::Quat>(x) << q_next.w(), q_next.x(), q_next.y(), q_next.z();
        }
        else {
            apply_pair<typename StateDef::Att, typename StateDef::Att>(x, dx);
        }
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_gyro_bias(NominalState_t& x, const ErrorState_t& dx)
    {
        if constexpr (requires { typename StateDef::GyroBias; }) {
            apply_pair<typename StateDef::GyroBias, typename StateDef::GyroB>(x, dx);
        }
        else {
            apply_pair<typename StateDef::GyroB, typename StateDef::GyroB>(x, dx);
        }
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_accel_bias(NominalState_t& x, const ErrorState_t& dx)
    {
        if constexpr (requires { typename StateDef::AccelBias; }) {
            apply_pair<typename StateDef::AccelBias, typename StateDef::AccB>(x, dx);
        }
        else {
            apply_pair<typename StateDef::AccB, typename StateDef::AccB>(x, dx);
        }
    }
};

template<typename StateDef>
using DefaultInjectionPolicy = InsInjectionPolicy<StateDef>;

} // namespace navkit::core::estimation
