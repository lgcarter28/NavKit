// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/NavigatorPolicyCompatibility.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"

namespace navkit::core::estimation
{

template<typename Candidate, typename Filter, typename SensorTuple>
concept NavigatorUpdatePolicy =
    SensorCollectionPolicy<SensorTuple> &&
    detail::navigator_policy_compatible_v<Filter, Candidate, SensorTuple>;

} // namespace navkit::core::estimation
