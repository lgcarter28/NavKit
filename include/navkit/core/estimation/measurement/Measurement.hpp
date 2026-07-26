// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<int M>
struct Measurement
{
    Timestamp t{};
    Eigen::Matrix<Scalar_t, M, 1> z{Eigen::Matrix<Scalar_t, M, 1>::Zero()};
};

} // namespace navkit::core::estimation
