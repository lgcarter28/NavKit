// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/planet/PlanetPolicyBase.hpp"
#include "navkit/core/frames/Frames.hpp"

namespace navkit::core::environment
{

namespace frames = navkit::core::frames;

// Stub reserved for future lunar navigation support.
// Intentionally incomplete until authoritative constants and frame conventions
// are selected.
struct Moon : PlanetPolicyBase<Moon>
{
    using InertialFrame = frames::MoonCenteredInertial;
    using FixedFrame = frames::MoonCenteredMoonFixed;
};

} // namespace navkit::core::environment
