// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace navkit::core::math
{

/**
 * Validates and normalizes a quaternion while selecting the nonnegative-scalar
 * representation of the same rotation.
 */
[[nodiscard]] inline bool normalize_quaternion(const Eigen::Quaternion<Scalar_t>& q,
                                               Eigen::Quaternion<Scalar_t>& q_normalized)
{
    if (!q.coeffs().allFinite()) {
        return false;
    }
    const Scalar_t squared_norm = q.squaredNorm();
    if (!std::isfinite(squared_norm) || squared_norm <= 1.0e-24) {
        return false;
    }

    q_normalized = q;
    q_normalized.normalize();
    if (q_normalized.w() < 0.0) {
        q_normalized.coeffs() *= -1.0;
    }
    return true;
}

[[nodiscard]] inline Eigen::Quaternion<Scalar_t>
normalized_with_positive_scalar(Eigen::Quaternion<Scalar_t> q)
{
    q.normalize();
    if (q.w() < 0.0) {
        q.coeffs() *= -1.0;
    }
    return q;
}

/**
 * Returns whether a matrix is a finite, proper, orthonormal direction cosine
 * matrix within the supplied numerical tolerance.
 */
[[nodiscard]] inline bool dcm_is_valid(const Mat3& C_start2end, const Scalar_t tolerance = 1.0e-8)
{
    if (!C_start2end.allFinite() || !std::isfinite(tolerance) || tolerance < 0.0) {
        return false;
    }
    const Mat3 orthogonality_error = (C_start2end.transpose() * C_start2end) - Mat3::Identity();
    return orthogonality_error.cwiseAbs().maxCoeff() <= tolerance &&
           std::abs(C_start2end.determinant() - 1.0) <= tolerance;
}

/**
 * Converts a validated passive direction cosine matrix to a unit quaternion.
 *
 * Both representations map components from the named start frame into the
 * named end frame.
 */
[[nodiscard]] inline bool quaternion_from_dcm(const Mat3& C_start2end,
                                              Eigen::Quaternion<Scalar_t>& q_start2end)
{
    if (!dcm_is_valid(C_start2end)) {
        return false;
    }
    return normalize_quaternion(Eigen::Quaternion<Scalar_t>{C_start2end}, q_start2end);
}

[[nodiscard]] inline Vec3 rotvec_rad_from_quaternion(Eigen::Quaternion<Scalar_t> q)
{
    q = normalized_with_positive_scalar(q);
    const auto sin_half_angle = q.vec().norm();
    if (sin_half_angle <= 1.0e-15) {
        return 2.0 * q.vec();
    }

    const auto angle = 2.0 * std::atan2(sin_half_angle, q.w());
    return (angle / sin_half_angle) * q.vec();
}

[[nodiscard]] inline Eigen::Quaternion<Scalar_t> quaternion_from_rotvec_rad(const Vec3& phi)
{
    const auto angle = phi.norm();
    if (angle <= 1.0e-15) {
        Eigen::Quaternion<Scalar_t> q{1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()};
        return normalized_with_positive_scalar(q);
    }

    const auto axis = phi / angle;
    return normalized_with_positive_scalar(
        Eigen::Quaternion<Scalar_t>{Eigen::AngleAxis<Scalar_t>{angle, axis}});
}

/**
 * Constructs the passive start-to-end transform encoded by aerospace
 * roll/pitch/yaw angles.
 *
 * `rpy_rad = [roll, pitch, yaw]` uses the 3-2-1 yaw-pitch-roll composition
 * `C_start2end = Rz(yaw) * Ry(pitch) * Rx(roll)`. The Euler angles of the
 * inverse transform are not, in general, the componentwise negatives.
 */
[[nodiscard]] inline Eigen::Quaternion<Scalar_t> quaternion_from_rpy_rad(const Vec3& rpy_rad)
{
    const Eigen::AngleAxis<Scalar_t> roll{rpy_rad.x(), Vec3::UnitX()};
    const Eigen::AngleAxis<Scalar_t> pitch{rpy_rad.y(), Vec3::UnitY()};
    const Eigen::AngleAxis<Scalar_t> yaw{rpy_rad.z(), Vec3::UnitZ()};
    return normalized_with_positive_scalar(Eigen::Quaternion<Scalar_t>{yaw * pitch * roll});
}

[[nodiscard]] inline Vec3 rpy_rad_from_quaternion(const Eigen::Quaternion<Scalar_t>& q)
{
    const Mat3 C_start2end = q.normalized().toRotationMatrix();
    const Scalar_t sin_pitch = std::clamp(-C_start2end(2, 0), -1.0, 1.0);
    const Scalar_t pitch_rad = std::asin(sin_pitch);

    Scalar_t roll_rad = 0.0;
    Scalar_t yaw_rad = 0.0;
    if (std::abs(std::cos(pitch_rad)) > 1.0e-12) {
        roll_rad = std::atan2(C_start2end(2, 1), C_start2end(2, 2));
        yaw_rad = std::atan2(C_start2end(1, 0), C_start2end(0, 0));
    }
    else {
        // At gimbal lock, roll and yaw are not independently observable.
        // Select roll = 0 and retain the equivalent combined rotation in yaw.
        yaw_rad = std::atan2(-C_start2end(0, 1), C_start2end(1, 1));
    }

    return Vec3{roll_rad, pitch_rad, yaw_rad};
}

[[nodiscard]] inline Vec3 rotation_vector_from_quaternion(Eigen::Quaternion<Scalar_t> q)
{
    return rotvec_rad_from_quaternion(q);
}

[[nodiscard]] inline Eigen::Quaternion<Scalar_t> quaternion_from_rotation_vector(const Vec3& phi)
{
    return quaternion_from_rotvec_rad(phi);
}

[[nodiscard]] inline Eigen::Quaternion<Scalar_t> quaternion_from_rpy_zyx_rad(const Vec3& rpy_rad)
{
    return quaternion_from_rpy_rad(rpy_rad);
}

[[nodiscard]] inline Vec3 rpy_zyx_rad_from_quaternion(const Eigen::Quaternion<Scalar_t>& q)
{
    return rpy_rad_from_quaternion(q);
}

} // namespace navkit::core::math
