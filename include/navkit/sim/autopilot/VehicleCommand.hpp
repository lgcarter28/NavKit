// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

namespace navkit::sim
{

/** Complete plant-facing command produced by an Autopilot implementation. */
struct VehicleCommand
{
    core::Vec3 w_command_ib_b_radps{core::Vec3::Zero()};
    core::Vec3 specific_force_command_ib_b_mps2{core::Vec3::Zero()};
};

} // namespace navkit::sim
