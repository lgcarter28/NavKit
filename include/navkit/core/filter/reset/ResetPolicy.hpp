// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/state/State.hpp"

#include <concepts>

namespace navkit
{

template<typename Candidate, typename StateDef>
concept ResetPolicy = StateDefPolicy<StateDef> &&
                      requires(State<StateDef>& x, State<StateDef>& dx, StateCov<StateDef>& P) {
                          { Candidate::reset_covariance(x, dx, P) } -> std::same_as<void>;
                          { Candidate::reset_dx(dx) } -> std::same_as<void>;
                      };

} // namespace navkit
