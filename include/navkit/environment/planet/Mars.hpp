// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/environment/planet/PlanetPolicyBase.hpp"
#include "navkit/frames/Frames.hpp"

namespace navkit::planet
{

// Stub reserved for future Mars navigation support.
// Intentionally incomplete until authoritative constants and frame conventions
// are selected.
struct Mars : PlanetPolicyBase<Mars>
{
    using InertialFrame = frames::MarsCenteredInertial;
    using FixedFrame = frames::MarsCenteredMarsFixed;
};

} // namespace navkit::planet
