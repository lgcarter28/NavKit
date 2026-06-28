#pragma once

namespace navkit {

template <typename StateDef>
struct DefaultResetPolicy {
    template <typename State_t, typename P_t>
    static void reset_covariance(State_t&, const State_t&, P_t&) {
        // no-op in V1; covariance reset mapping belongs in a later attitude-aware version.
    }

    template <typename State_t>
    static void reset_dx(State_t& dx) {
        dx.setZero();
    }
};

} // namespace navkit
