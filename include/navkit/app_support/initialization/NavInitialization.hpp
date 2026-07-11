// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"

namespace navkit::app_support
{

struct NavInitialization
{
    core::Time_t time_s{0.0};
    core::estimation::PvaState pva{core::estimation::PvaState::Zero()};
    core::estimation::PvaCovariance pva_cov{core::estimation::PvaCovariance::Zero()};
};

} // namespace navkit::app_support
