// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/InitialCovariance.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

namespace navkit::app_support
{

struct PvaInitialization
{
    core::Time_t time_s{0.0};
    core::estimation::PvaState pva{core::estimation::PvaState::Zero()};
};

template<core::estimation::StateSpaceDefPolicy StateDef>
struct NavInitialization
{
    core::Time_t time_s{0.0};
    core::estimation::PvaState pva{core::estimation::PvaState::Zero()};
    core::estimation::InitialCovariance<StateDef> covariance{
        core::estimation::InitialCovariance<StateDef>::Zero()};
};

} // namespace navkit::app_support
