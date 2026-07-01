// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <Eigen/Dense>
#include <ratio>
#include <type_traits>

namespace navkit::core::units
{

template<int L, int T, int A>
struct Dim
{
    static constexpr int length = L;
    static constexpr int time = T;
    static constexpr int angle = A;
};

using Dimensionless = Dim<0, 0, 0>;
using Length = Dim<1, 0, 0>;
using Time = Dim<0, 1, 0>;
using Angle = Dim<0, 0, 1>;
using Velocity = Dim<1, -1, 0>;
using Acceleration = Dim<1, -2, 0>;
using Jerk = Dim<1, -3, 0>;
using AngularRate = Dim<0, -1, 1>;
using AngularAcceleration = Dim<0, -2, 1>;

template<typename D1, typename D2>
using DimMul = Dim<D1::length + D2::length, D1::time + D2::time, D1::angle + D2::angle>;

template<typename D1, typename D2>
using DimDiv = Dim<D1::length - D2::length, D1::time - D2::time, D1::angle - D2::angle>;

template<typename Dimension,
         typename Ratio = std::ratio<1>,
         typename Scalar = navkit::core::Scalar_t>
struct Unit
{
    using dim = Dimension;
    using ratio = Ratio;
    using scalar = Scalar;
    static constexpr Scalar scale_to_si =
        static_cast<Scalar>(Ratio::num) / static_cast<Scalar>(Ratio::den);
};

using Meter = Unit<Length>;
using Kilometer = Unit<Length, std::kilo>;
using Foot = Unit<Length, std::ratio<381, 1250>>; // 0.3048 m
using Second = Unit<Time>;
using Millisecond = Unit<Time, std::milli>;
using Radian = Unit<Angle>;
using Degree = Unit<Angle, std::ratio<174532925199433, 10000000000000000LL>>; // approx pi/180

template<typename UnitT, typename Frame, int N, typename Scalar = navkit::core::Scalar_t>
struct Vec
{
    using unit = UnitT;
    using dim = typename UnitT::dim;
    using frame = Frame;
    using scalar = Scalar;
    Eigen::Matrix<Scalar, N, 1> v{Eigen::Matrix<Scalar, N, 1>::Zero()};
};

template<typename U1, typename U2, typename Frame, int N, typename Scalar>
auto operator+(const Vec<U1, Frame, N, Scalar>& a, const Vec<U2, Frame, N, Scalar>& b)
{
    static_assert(std::is_same_v<typename U1::dim, typename U2::dim>,
                  "Cannot add vectors with different dimensions");
    constexpr Scalar scale = U2::scale_to_si / U1::scale_to_si;
    return Vec<U1, Frame, N, Scalar>{a.v + scale * b.v};
}

template<typename U1, typename U2, typename Frame, int N, typename Scalar>
auto unit_cast(const Vec<U1, Frame, N, Scalar>& x)
{
    static_assert(std::is_same_v<typename U1::dim, typename U2::dim>,
                  "Cannot cast between units with different dimensions");
    constexpr Scalar scale = U1::scale_to_si / U2::scale_to_si;
    return Vec<U2, Frame, N, Scalar>{scale * x.v};
}

} // namespace navkit::core::units
