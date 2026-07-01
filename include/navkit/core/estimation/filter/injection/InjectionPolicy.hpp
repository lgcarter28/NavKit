// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept InjectionPolicy =
    StateDefPolicy<StateDef> && requires(State<StateDef>& x, const State<StateDef>& dx) {
        { Candidate::apply(x, dx) } -> std::same_as<void>;
    };

} // namespace navkit::core::estimation
