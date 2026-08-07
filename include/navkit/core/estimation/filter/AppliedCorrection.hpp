// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"

namespace navkit::core::estimation
{

/**
 * \brief Error-state correction applied to a filter nominal state.
 *
 * \details This value object carries the correction produced by one completed
 * filter injection. It does not own cycle-level accumulation or logging state.
 */
template<StateSpaceDefPolicy StateDef>
struct AppliedCorrection
{
    ErrorState<StateDef> value{ErrorState<StateDef>::Zero()};
    bool valid{false};
};

} // namespace navkit::core::estimation
