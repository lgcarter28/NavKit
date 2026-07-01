// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/common/Config.hpp"

#include <Eigen/Dense>

namespace navkit
{

template<int M>
struct Measurement
{
    Time_t time{0.0};
    Eigen::Matrix<Scalar_t, M, 1> z{Eigen::Matrix<Scalar_t, M, 1>::Zero()};
};

} // namespace navkit
