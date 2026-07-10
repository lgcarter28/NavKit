// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::app_support
{

struct NavInitialization
{
    core::Time_t time_s{0.0};
    core::Vec3 p_e_m{core::Vec3::Zero()};
    core::Vec3 v_e_mps{core::Vec3::Zero()};
    core::Vec3 rpy_be_rad{core::Vec3::Zero()};
    core::estimation::PvaCovariance pva_cov{core::estimation::PvaCovariance::Zero()};
};

} // namespace navkit::app_support
