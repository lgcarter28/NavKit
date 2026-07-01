// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/common/Config.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace navkit::sim
{

using navkit::core::Scalar_t;
using navkit::core::Time_t;

struct TruthSample
{
    Time_t time{0.0};
    Eigen::Matrix<Scalar_t, 3, 1> p_e{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
    Eigen::Matrix<Scalar_t, 3, 1> v_e{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
    Eigen::Matrix<Scalar_t, 3, 1> a_e{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
    Eigen::Quaternion<Scalar_t> q_eb{Eigen::Quaternion<Scalar_t>::Identity()};
    Eigen::Matrix<Scalar_t, 3, 1> w_ib_b{Eigen::Matrix<Scalar_t, 3, 1>::Zero()};
};

} // namespace navkit::sim
