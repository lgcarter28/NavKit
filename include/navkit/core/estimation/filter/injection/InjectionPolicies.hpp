// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<typename Seg>
struct AdditiveInjection
{
    template<typename State_t>
    static void apply(State_t& x, const State_t& dx)
    {
        segment<Seg>(x) += segment<Seg>(dx);
    }
};

template<typename StateDef>
struct InsInjectionPolicy
{
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;

    template<typename NominalState_t, typename ErrorState_t>
    static void apply(NominalState_t& x, const ErrorState_t& dx)
    {
        apply_additive_pair<typename Nominal::Pos, typename Error::Pos>(x, dx);
        apply_additive_pair<typename Nominal::Vel, typename Error::Vel>(x, dx);
        apply_attitude(x, dx);
        if constexpr (requires {
                          typename Nominal::GyroB;
                          typename Error::GyroB;
                      }) {
            apply_additive_pair<typename Nominal::GyroB, typename Error::GyroB>(x, dx);
        }
        if constexpr (requires {
                          typename Nominal::GyroSf;
                          typename Error::GyroSf;
                      }) {
            apply_additive_pair<typename Nominal::GyroSf, typename Error::GyroSf>(x, dx);
        }
        if constexpr (requires {
                          typename Nominal::AccB;
                          typename Error::AccB;
                      }) {
            apply_additive_pair<typename Nominal::AccB, typename Error::AccB>(x, dx);
        }
        if constexpr (requires {
                          typename Nominal::AccSf;
                          typename Error::AccSf;
                      }) {
            apply_additive_pair<typename Nominal::AccSf, typename Error::AccSf>(x, dx);
        }
        if constexpr (requires {
                          typename Nominal::ClkB;
                          typename Error::ClkB;
                      }) {
            apply_additive_pair<typename Nominal::ClkB, typename Error::ClkB>(x, dx);
        }
        if constexpr (requires {
                          typename Nominal::ClkD;
                          typename Error::ClkD;
                      }) {
            apply_additive_pair<typename Nominal::ClkD, typename Error::ClkD>(x, dx);
        }
    }

private:
    template<typename NominalSegment,
             typename ErrorSegment,
             typename NominalState_t,
             typename ErrorState_t>
    static void apply_additive_pair(NominalState_t& x, const ErrorState_t& dx)
    {
        segment<NominalSegment>(x) += segment<ErrorSegment>(dx);
    }

    template<typename NominalState_t, typename ErrorState_t>
    static void apply_attitude(NominalState_t& x, const ErrorState_t& dx)
    {
        if constexpr (requires {
                          typename Nominal::AttQuat;
                          typename Error::AttRotVec;
                      }) {
            const Eigen::Matrix<Scalar_t, 4, 1> q_segment = segment<typename Nominal::AttQuat>(x);
            const Eigen::Quaternion<Scalar_t> q_b2e{
                q_segment(0), q_segment(1), q_segment(2), q_segment(3)};
            const Eigen::Quaternion<Scalar_t> delta_q =
                navkit::core::math::quaternion_from_rotvec_rad(
                    segment<typename Error::AttRotVec>(dx));
            const Eigen::Quaternion<Scalar_t> q_next =
                navkit::core::math::normalized_with_positive_scalar(delta_q * q_b2e);
            segment<typename Nominal::AttQuat>(x) << q_next.w(), q_next.x(), q_next.y(), q_next.z();
        }
        else {
            apply_additive_pair<typename Nominal::AttRpy, typename Error::AttRotVec>(x, dx);
        }
    }
};

template<typename StateDef>
using DefaultInjectionPolicy = InsInjectionPolicy<StateDef>;

} // namespace navkit::core::estimation
