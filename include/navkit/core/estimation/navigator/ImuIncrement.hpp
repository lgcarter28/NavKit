// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::core::estimation
{

struct ImuIncrement
{
    Time_t time_s{0.0};
    Time_t dt_s{0.0};
    Vec3 delta_theta_ib_b_rad{Vec3::Zero()};
    Vec3 delta_v_ib_b_mps{Vec3::Zero()};
};

} // namespace navkit::core::estimation
