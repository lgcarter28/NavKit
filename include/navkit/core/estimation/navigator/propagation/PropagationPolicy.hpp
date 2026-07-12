// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate, typename Filter, typename SensorTuple>
concept PropagationPolicy = FilterPolicy<Filter> && SensorCollectionPolicy<SensorTuple> &&
                            requires(Filter& filter, SensorTuple& sensors) {
                                {
                                    Candidate::process_strapdown_integration(filter, sensors)
                                } -> std::same_as<void>;
                                {
                                    Candidate::process_covariance(filter, sensors)
                                } -> std::same_as<void>;
                            };

} // namespace navkit::core::estimation
