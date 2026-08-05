// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateAccessors.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

namespace navkit::app_support
{

namespace detail
{

[[nodiscard]] inline bool
ecef_pva_to_trajectory_control_state(const core::Timestamp& t,
                                     const core::Timestamp& t_epoch,
                                     const core::Vec3& p_e_m,
                                     const core::Vec3& v_eb_e_mps,
                                     const Eigen::Quaternion<core::Scalar_t>& q_b2e,
                                     sim::TrajectoryControlState& output)
{
    core::Mat3 C_e2i{};
    if (!p_e_m.allFinite() || !v_eb_e_mps.allFinite() || !q_b2e.coeffs().allFinite() ||
        !core::frames::fixed_to_inertial_matrix<core::environment::Wgs84>(t, t_epoch, C_e2i)) {
        return false;
    }

    const core::Vec3 w_ie_e_radps =
        core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
    output = {};
    output.t = t;
    output.p_i_m = C_e2i * p_e_m;
    output.v_i_mps = C_e2i * (v_eb_e_mps + w_ie_e_radps.cross(p_e_m));
    output.q_b2i = core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<core::Scalar_t>{C_e2i} * q_b2e);
    return output.p_i_m.allFinite() && output.v_i_mps.allFinite() &&
           output.q_b2i.coeffs().allFinite();
}

} // namespace detail

/** Converts current trajectory truth into the source-agnostic ECI control-state contract. */
[[nodiscard]] inline bool trajectory_control_state_from_truth(const sim::TruthSample& truth,
                                                              const core::Timestamp& t_epoch,
                                                              sim::TrajectoryControlState& output)
{
    if (!detail::ecef_pva_to_trajectory_control_state(
            truth.t, t_epoch, truth.p_e, truth.v_e, truth.q_b2e, output)) {
        return false;
    }
    output.w_ib_b_radps = truth.w_ib_b_radps;
    return output.w_ib_b_radps.allFinite();
}

/** Converts the current Navigator nominal PVA estimate into the same ECI control-state contract. */
template<core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] bool
trajectory_control_state_from_navigation(const core::Timestamp& t,
                                         const core::Timestamp& t_epoch,
                                         const core::estimation::NominalState<StateDef>& state,
                                         sim::TrajectoryControlState& output)
{
    using Nominal = typename StateDef::Nominal;

    const core::Vec3 p_e_m = core::estimation::segment<typename Nominal::Pos>(state);
    const core::Vec3 v_eb_e_mps = core::estimation::segment<typename Nominal::Vel>(state);
    const Eigen::Quaternion<core::Scalar_t> q_b2e = core::estimation::q_b2e<StateDef>(state);
    return detail::ecef_pva_to_trajectory_control_state(
        t, t_epoch, p_e_m, v_eb_e_mps, q_b2e, output);
}

} // namespace navkit::app_support
