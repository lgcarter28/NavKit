// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace navkit::sim
{

using navkit::core::Scalar_t;
using navkit::core::Time_t;

struct TruthSample
{
    core::Timestamp t{};
    Eigen::Matrix<Scalar_t, 3, 1> p_e{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
    Eigen::Matrix<Scalar_t, 3, 1> v_e{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
    // Body-to-ECEF attitude. Applying q_b2e to a body-resolved vector resolves it in ECEF.
    Eigen::Quaternion<Scalar_t> q_b2e{Eigen::Quaternion<Scalar_t>::Identity()};
    Eigen::Matrix<Scalar_t, 3, 1> w_ib_b_radps{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
};

} // namespace navkit::sim
