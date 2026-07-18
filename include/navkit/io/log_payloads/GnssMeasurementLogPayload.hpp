// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::io
{

struct GnssPositionLogPayload
{
    core::estimation::Measurement<3> measurement{};
};

struct GnssVelocityLogPayload
{
    core::estimation::Measurement<3> measurement{};
};

struct GnssPositionDebugLogPayload
{
    core::Time_t time_s{0.0};
    core::Vec3 truth_p_e_m{core::Vec3::Zero()};
    core::Vec3 measured_p_e_m{core::Vec3::Zero()};
    core::Vec3 sigma_p_e_m{core::Vec3::Zero()};
};

struct GnssVelocityDebugLogPayload
{
    core::Time_t time_s{0.0};
    core::Vec3 truth_v_e_mps{core::Vec3::Zero()};
    core::Vec3 measured_v_e_mps{core::Vec3::Zero()};
    core::Vec3 sigma_v_e_mps{core::Vec3::Zero()};
};

} // namespace navkit::io
