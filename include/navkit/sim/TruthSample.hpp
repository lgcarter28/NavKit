#pragma once

#include "navkit/core/Config.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace navkit::sim
{

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
