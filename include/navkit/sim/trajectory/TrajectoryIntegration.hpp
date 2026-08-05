// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace navkit::sim
{

enum class TranslationalIntegrationMethod
{
    SemiImplicitEuler,
    TrapezoidalPredictorCorrector,
};

[[nodiscard]] inline bool integrate_translation_eci(const core::Vec3& a_previous_i_mps2,
                                                    const core::Vec3& a_current_i_mps2,
                                                    const core::Time_t dt_s,
                                                    const TranslationalIntegrationMethod method,
                                                    core::Vec3& p_i_m,
                                                    core::Vec3& v_i_mps)
{
    if (!p_i_m.allFinite() || !v_i_mps.allFinite() || !a_previous_i_mps2.allFinite() ||
        !a_current_i_mps2.allFinite() || !std::isfinite(dt_s) || dt_s <= 0.0) {
        return false;
    }
    switch (method) {
    case TranslationalIntegrationMethod::SemiImplicitEuler:
        v_i_mps += a_current_i_mps2 * dt_s;
        p_i_m += v_i_mps * dt_s;
        return true;
    case TranslationalIntegrationMethod::TrapezoidalPredictorCorrector:
        break;
    default:
        return false;
    }

    const core::Vec3 v_next_i_mps = v_i_mps + (0.5 * (a_previous_i_mps2 + a_current_i_mps2) * dt_s);
    p_i_m += 0.5 * (v_i_mps + v_next_i_mps) * dt_s;
    v_i_mps = v_next_i_mps;
    return true;
}

[[nodiscard]] inline bool integrate_attitude_eci(const core::Vec3& w_previous_ib_b_radps,
                                                 const core::Vec3& w_current_ib_b_radps,
                                                 const core::Time_t dt_s,
                                                 Eigen::Quaternion<core::Scalar_t>& q_b2i)
{
    if (!w_previous_ib_b_radps.allFinite() || !w_current_ib_b_radps.allFinite() ||
        !q_b2i.coeffs().allFinite() || !std::isfinite(dt_s) || dt_s <= 0.0) {
        return false;
    }
    const core::Vec3 delta_theta_ib_b_rad =
        0.5 * (w_previous_ib_b_radps + w_current_ib_b_radps) * dt_s;
    q_b2i = core::math::normalized_with_positive_scalar(
        q_b2i * core::math::quaternion_from_rotvec_rad(delta_theta_ib_b_rad));
    return true;
}

} // namespace navkit::sim
