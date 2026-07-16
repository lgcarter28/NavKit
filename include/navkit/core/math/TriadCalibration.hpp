// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Skew.hpp"
#include "navkit/core/math/Types.hpp"

#include <Eigen/Dense>

namespace navkit::core::math
{

[[nodiscard]] inline Eigen::Matrix<Scalar_t, 3, 3> scale_matrix(const Vec3& scale_factor)
{
    Eigen::Matrix<Scalar_t, 3, 3> scale = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    scale.diagonal() += scale_factor;
    return scale;
}

[[nodiscard]] inline Eigen::Matrix<Scalar_t, 3, 3>
nonorthogonality_matrix(const Vec3& nonorthogonality)
{
    Eigen::Matrix<Scalar_t, 3, 3> matrix = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    matrix(1, 0) = nonorthogonality.x();
    matrix(2, 0) = nonorthogonality.y();
    matrix(2, 1) = nonorthogonality.z();
    return matrix;
}

[[nodiscard]] inline Eigen::Matrix<Scalar_t, 3, 3> misalignment_matrix(const Vec3& misalignment_rad)
{
    return Eigen::Matrix<Scalar_t, 3, 3>::Identity() - skew_symmetric(misalignment_rad);
}

[[nodiscard]] inline Vec3 apply_triad_calibration(const Vec3& input,
                                                  const Vec3& scale_factor,
                                                  const Vec3& misalignment_rad,
                                                  const Vec3& nonorthogonality)
{
    return scale_matrix(scale_factor) * nonorthogonality_matrix(nonorthogonality) *
           misalignment_matrix(misalignment_rad) * input;
}

} // namespace navkit::core::math
