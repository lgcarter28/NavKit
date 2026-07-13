// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::core::estimation
{

template<typename StateDef>
struct DefaultResetPolicy
{
    template<typename NominalState_t, typename ErrorState_t, typename P_t>
    static void reset_covariance(NominalState_t&, const ErrorState_t&, P_t&)
    {
        // no-op in V1; covariance reset mapping belongs in a later attitude-aware version.
    }

    template<typename State_t>
    static void reset_dx(State_t& dx)
    {
        dx.setZero();
    }
};

} // namespace navkit::core::estimation
