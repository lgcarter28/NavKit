// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept ResetPolicy =
    StateSpaceDefPolicy<StateDef> &&
    requires(NominalState<StateDef>& x, ErrorState<StateDef>& dx, ErrorStateCov<StateDef>& P) {
        { Candidate::reset_covariance(x, dx, P) } -> std::same_as<void>;
        { Candidate::reset_dx(dx) } -> std::same_as<void>;
    };

} // namespace navkit::core::estimation
