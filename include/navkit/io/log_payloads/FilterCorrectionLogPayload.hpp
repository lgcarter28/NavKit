// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/State.hpp"

namespace navkit::io
{

template<typename StateDef>
struct FilterCorrectionLogPayload
{
    core::Time_t time_s{};
    const core::estimation::ErrorState<StateDef>& correction;
};

} // namespace navkit::io
