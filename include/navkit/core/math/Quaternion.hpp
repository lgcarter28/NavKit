// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace navkit::core::math
{

[[nodiscard]] inline Eigen::Quaternion<Scalar_t>
normalized_with_positive_scalar(Eigen::Quaternion<Scalar_t> q)
{
    q.normalize();
    if (q.w() < 0.0) {
        q.coeffs() *= -1.0;
    }
    return q;
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

[[nodiscard]] inline Eigen::Quaternion<Scalar_t> quaternion_from_rpy_rad(const Vec3& rpy_rad)
{
    const Eigen::AngleAxis<Scalar_t> roll{rpy_rad.x(), Vec3::UnitX()};
    const Eigen::AngleAxis<Scalar_t> pitch{rpy_rad.y(), Vec3::UnitY()};
    const Eigen::AngleAxis<Scalar_t> yaw{rpy_rad.z(), Vec3::UnitZ()};
    return normalized_with_positive_scalar(Eigen::Quaternion<Scalar_t>{yaw * pitch * roll});
}

[[nodiscard]] inline Vec3 rpy_rad_from_quaternion(const Eigen::Quaternion<Scalar_t>& q)
{
    const auto euler_zyx = q.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
    return Vec3{euler_zyx.z(), euler_zyx.y(), euler_zyx.x()};
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
