// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/common/Config.hpp"

#include <Eigen/Dense>

namespace navkit::frames
{

// -----------------------------------------------------------------------------
// Generic planet-centered frame terminology
// -----------------------------------------------------------------------------

struct PlanetCenteredInertial
{};

struct PlanetCenteredPlanetFixed
{};

// -----------------------------------------------------------------------------
// Earth frames
// -----------------------------------------------------------------------------

struct EarthCenteredInertial
{};

struct EarthCenteredEarthFixed
{};

using Eci = EarthCenteredInertial;
using Ecef = EarthCenteredEarthFixed;

// -----------------------------------------------------------------------------
// Moon frames
// -----------------------------------------------------------------------------

struct MoonCenteredInertial
{};

struct MoonCenteredMoonFixed
{};

using Mci = MoonCenteredInertial;
using Mcmf = MoonCenteredMoonFixed;

// -----------------------------------------------------------------------------
// Mars frames
// -----------------------------------------------------------------------------

struct MarsCenteredInertial
{};

struct MarsCenteredMarsFixed
{};

using MarsCi = MarsCenteredInertial;
using MarsCmf = MarsCenteredMarsFixed;

// -----------------------------------------------------------------------------
// Local / vehicle frames
// -----------------------------------------------------------------------------

struct Ned
{};

struct Body
{};

struct Sensor
{};

// -----------------------------------------------------------------------------
// Direction cosine matrix resolved-frame type helper
// -----------------------------------------------------------------------------

template<typename From, typename To, typename Scalar = navkit::Scalar_t>
struct Dcm
{
    using FromFrame = From;
    using ToFrame = To;
    using Scalar_t = Scalar;

    Eigen::Matrix<Scalar, 3, 3> C{Eigen::Matrix<Scalar, 3, 3>::Identity()};
};

template<typename From, typename To, typename Scalar>
Eigen::Matrix<Scalar, 3, 1> operator*(const Dcm<From, To, Scalar>& dcm,
                                      const Eigen::Matrix<Scalar, 3, 1>& v_from)
{
    return dcm.C * v_from;
}

} // namespace navkit::frames
