// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/math/Skew.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<typename StateDef>
struct DefaultResetPolicy
{
    template<typename NominalState_t, typename ErrorState_t, typename P_t>
    static void reset_covariance(NominalState_t&, const ErrorState_t& dx, P_t& P)
    {
        using Error = typename StateDef::Error;

        if constexpr (requires { typename Error::AttRotVec; }) {
            using AttRotVec = typename Error::AttRotVec;
            static_assert(AttRotVec::sz == 3,
                          "DefaultResetPolicy requires a three-component attitude rotation "
                          "vector");

            const core::Vec3 attitude_correction_rad = segment<AttRotVec>(dx);
            const core::Mat3 attitude_reset =
                core::Mat3::Identity() +
                (core::Scalar_t{0.5} * navkit::core::math::skew_symmetric(attitude_correction_rad));

            P_t reset_jacobian = P_t::Identity();
            reset_jacobian.template block<3, 3>(AttRotVec::i, AttRotVec::i) = attitude_reset;

            const P_t reset_covariance = reset_jacobian * P * reset_jacobian.transpose();
            P = reset_covariance;
        }
    }

    template<typename State_t>
    static void reset_dx(State_t& dx)
    {
        dx.setZero();
    }
};

} // namespace navkit::core::estimation
