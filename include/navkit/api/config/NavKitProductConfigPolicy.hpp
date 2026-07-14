// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"

#include <concepts>

namespace navkit::api::config
{

template<typename Candidate>
concept NavKitProductConfigPolicy = requires {
    typename Candidate::Numeric;
    typename Candidate::StateDef;
    typename Candidate::Sensors;
    typename Candidate::Profiler;
    typename Candidate::Filter;
    typename Candidate::Propagation;
    typename Candidate::NavigatorUpdate;
    typename Candidate::Navigator;

    requires navkit::core::config::ConfigPolicy<Candidate>;
    requires navkit::core::estimation::StateSpaceDefPolicy<typename Candidate::StateDef>;
    requires navkit::core::estimation::SensorCollectionPolicy<typename Candidate::Sensors>;
    requires navkit::core::estimation::sensor_ids_unique_v<typename Candidate::Sensors>;
    requires navkit::core::estimation::FilterPolicy<typename Candidate::Filter>;
    requires std::same_as<typename Candidate::Filter::Sensors_t, typename Candidate::Sensors>;
    requires navkit::core::profiling::ProfilerPolicy<typename Candidate::Profiler>;
    requires navkit::core::estimation::PropagationPolicy<typename Candidate::Propagation,
                                                         typename Candidate::StateDef>;
    requires navkit::core::estimation::NavigatorUpdatePolicy<typename Candidate::NavigatorUpdate,
                                                             typename Candidate::Filter,
                                                             typename Candidate::Sensors>;
    requires std::same_as<typename Candidate::Navigator::Filter_t, typename Candidate::Filter>;
    requires std::same_as<typename Candidate::Navigator::Sensors_t, typename Candidate::Sensors>;
    requires std::same_as<typename Candidate::Navigator::Propagation_t,
                          typename Candidate::Propagation>;
    requires std::same_as<typename Candidate::Navigator::Update_t,
                          typename Candidate::NavigatorUpdate>;
    requires std::same_as<typename Candidate::Navigator::Profiler_t, typename Candidate::Profiler>;
};

} // namespace navkit::api::config
