// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace navkit::sim
{

/**
 * Builds body-to-NED attitude with body x along velocity.
 *
 * The retained body-right reference removes the singularity when velocity is
 * parallel to the requested down direction. The returned right axis is the
 * orthonormalized body-y axis of the resulting command attitude.
 */
[[nodiscard]] inline bool velocity_aligned_attitude_b2n(const core::Vec3& v_n_mps,
                                                        const core::Vec3& down_reference_n,
                                                        const core::Vec3& prior_right_n,
                                                        Eigen::Quaternion<core::Scalar_t>& q_b2n,
                                                        core::Vec3& right_n)
{
    constexpr core::Scalar_t minimum_norm = 1.0e-12;
    if (!v_n_mps.allFinite() || !down_reference_n.allFinite() || !prior_right_n.allFinite() ||
        v_n_mps.norm() <= minimum_norm || down_reference_n.norm() <= minimum_norm) {
        return false;
    }

    const core::Vec3 forward_n = v_n_mps.normalized();
    const core::Vec3 down_n = down_reference_n.normalized();
    right_n = down_n.cross(forward_n);
    if (right_n.norm() <= minimum_norm) {
        right_n = prior_right_n - (forward_n.dot(prior_right_n) * forward_n);
    }
    if (right_n.norm() <= minimum_norm) {
        const core::Vec3 fallback_axis =
            std::abs(forward_n.x()) < std::abs(forward_n.y())
                ? (std::abs(forward_n.x()) < std::abs(forward_n.z()) ? core::Vec3::UnitX()
                                                                     : core::Vec3::UnitZ())
                : (std::abs(forward_n.y()) < std::abs(forward_n.z()) ? core::Vec3::UnitY()
                                                                     : core::Vec3::UnitZ());
        right_n = fallback_axis - (forward_n.dot(fallback_axis) * forward_n);
    }
    if (right_n.norm() <= minimum_norm) {
        return false;
    }
    right_n.normalize();
    const core::Vec3 corrected_down_n = forward_n.cross(right_n).normalized();
    core::Mat3 C_b2n{};
    C_b2n.col(0) = forward_n;
    C_b2n.col(1) = right_n;
    C_b2n.col(2) = corrected_down_n;
    q_b2n = Eigen::Quaternion<core::Scalar_t>{C_b2n}.normalized();
    return q_b2n.coeffs().allFinite();
}

/** Builds a velocity-aligned attitude using NED right as the initial fallback axis. */
[[nodiscard]] inline bool velocity_aligned_attitude_b2n(const core::Vec3& v_n_mps,
                                                        const core::Vec3& down_reference_n,
                                                        Eigen::Quaternion<core::Scalar_t>& q_b2n)
{
    core::Vec3 right_n{};
    return velocity_aligned_attitude_b2n(
        v_n_mps, down_reference_n, core::Vec3::UnitY(), q_b2n, right_n);
}

} // namespace navkit::sim
