// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/InitialCovariance.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept InitialCovarianceConfigPolicy = StateSpaceDefPolicy<StateDef> && requires {
    typename Candidate::InitialCovariance_t;
    requires std::same_as<typename Candidate::InitialCovariance_t, InitialCovariance<StateDef>>;
    requires std::same_as<std::remove_cvref_t<decltype(Candidate::initial_covariance)>,
                          InitialCovariance<StateDef>>;
};

} // namespace navkit::core::estimation
