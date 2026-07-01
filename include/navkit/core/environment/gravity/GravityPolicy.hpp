// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/common/Config.hpp"

#include <Eigen/Dense>

namespace navkit::core::environment
{

using navkit::core::Scalar_t;

template<typename T>
concept GravityPolicy = requires(const Eigen::Matrix<Scalar_t, 3, 1>& p) {
    typename T::Planet_t;
    typename T::Frame_t;
    T::acceleration(p);
};

} // namespace navkit::core::environment
