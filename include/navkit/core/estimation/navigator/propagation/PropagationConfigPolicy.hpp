// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"

namespace navkit::core::estimation
{

/// Compile-time selection that supplies a propagation policy for a state space.
template<typename Candidate, typename StateDef>
concept PropagationConfigPolicy = StateSpaceDefPolicy<StateDef> && requires {
    typename Candidate::Propagation;
    requires PropagationPolicy<typename Candidate::Propagation, StateDef>;
};

} // namespace navkit::core::estimation
