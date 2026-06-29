// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/Config.hpp"

#include <Eigen/Dense>

namespace navkit::frames
{

struct Eci
{};
struct Ecef
{};
struct Ned
{};
struct Body
{};
struct Sensor
{};

template<typename From, typename To, typename Scalar = navkit::Scalar_t>
struct Dcm
{
    Eigen::Matrix<Scalar, 3, 3> C{Eigen::Matrix<Scalar, 3, 3>::Identity()};
};

template<typename From, typename To, typename Scalar>
Eigen::Matrix<Scalar, 3, 1> operator*(const Dcm<From, To, Scalar>& dcm,
                                      const Eigen::Matrix<Scalar, 3, 1>& v_from)
{
    return dcm.C * v_from;
}

} // namespace navkit::frames
