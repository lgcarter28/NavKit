// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept InjectionPolicy =
    StateSpaceDefPolicy<StateDef> && requires(NominalState<StateDef>& x,
                                              const ErrorState<StateDef>& first,
                                              const ErrorState<StateDef>& second,
                                              ErrorState<StateDef>& composed) {
        { Candidate::apply(x, first) } -> std::same_as<void>;
        { Candidate::compose(first, second, composed) } -> std::same_as<void>;
    };

} // namespace navkit::core::estimation
