// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/InitialCovariance.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/time/Timestamp.hpp"

namespace navkit::app_support
{

struct PvaInitialization
{
    core::Timestamp t{};
    core::estimation::PvaState pva{core::estimation::PvaState::Zero()};
};

template<core::estimation::StateSpaceDefPolicy StateDef>
struct NavInitialization
{
    core::Timestamp t{};
    core::estimation::PvaState pva{core::estimation::PvaState::Zero()};
    core::estimation::InitialCovariance<StateDef> covariance{
        core::estimation::InitialCovariance<StateDef>::Zero()};
};

} // namespace navkit::app_support
