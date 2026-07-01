// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::gravity
{

template<typename Derived>
struct GravityPolicyBase
{
    // Intentionally empty for now.
    //
    // Do not reference Derived::Planet_t or Derived::Frame_t here.
    // During CRTP base instantiation, Derived is still incomplete.
};

} // namespace navkit::gravity