// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/sim/ImuSimulator.hpp"

namespace navkit::io
{

struct ImuIncrementLogPayload
{
    core::estimation::ImuIncrement truth;
    core::estimation::ImuIncrement measured;
    core::Vec3 truth_cumsum_delta_theta_ib_b_rad{core::Vec3::Zero()};
    core::Vec3 truth_cumsum_delta_v_ib_b_mps{core::Vec3::Zero()};
    core::Vec3 measured_cumsum_delta_theta_ib_b_rad{core::Vec3::Zero()};
    core::Vec3 measured_cumsum_delta_v_ib_b_mps{core::Vec3::Zero()};
    core::Vec3 gyro_bias_truth_radps{core::Vec3::Zero()};
    core::Vec3 accel_bias_truth_mps2{core::Vec3::Zero()};
};

} // namespace navkit::io
