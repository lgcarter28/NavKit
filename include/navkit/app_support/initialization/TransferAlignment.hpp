// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/core/estimation/navigator/TxaStateDef.hpp"
#include "navkit/core/math/Types.hpp"

namespace navkit::app_support
{

struct TransferAlignmentSample
{
    core::Time_t time_s{0.0};
    core::estimation::TxaState aiding{core::estimation::TxaState::Zero()};
    core::estimation::TxaCovariance covariance{core::estimation::TxaCovariance::Zero()};
    bool pos_valid{false};
    bool vel_valid{false};
    bool rpy_valid{false};
    bool angular_rate_valid{false};
    bool specific_force_valid{false};
};

} // namespace navkit::app_support
