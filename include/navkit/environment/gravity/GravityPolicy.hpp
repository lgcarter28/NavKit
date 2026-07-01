// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/common/Config.hpp"

#include <Eigen/Dense>

namespace navkit::gravity
{

template<typename T>
concept GravityPolicy = requires(const Eigen::Matrix<Scalar_t, 3, 1>& p) {
    typename T::Planet_t;
    typename T::Frame_t;
    T::acceleration(p);
};

} // namespace navkit::gravity
