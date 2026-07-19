// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/CovarianceFloor.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept CovarianceFloorConfigPolicy = StateSpaceDefPolicy<StateDef> && requires {
    typename Candidate::CovarianceFloor_t;
    requires std::same_as<typename Candidate::CovarianceFloor_t, CovarianceFloor<StateDef>>;
    requires std::same_as<std::remove_cvref_t<decltype(Candidate::covariance_floor)>,
                          CovarianceFloor<StateDef>>;
};

} // namespace navkit::core::estimation
