// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/frames/Frames.hpp"
#include "navkit/planet/PlanetPolicyBase.hpp"

namespace navkit::planet
{

// Stub reserved for future lunar navigation support.
// Intentionally incomplete until authoritative constants and frame conventions
// are selected.
struct Moon : PlanetPolicyBase<Moon>
{
    using InertialFrame = frames::MoonCenteredInertial;
    using FixedFrame = frames::MoonCenteredMoonFixed;
};

} // namespace navkit::planet
