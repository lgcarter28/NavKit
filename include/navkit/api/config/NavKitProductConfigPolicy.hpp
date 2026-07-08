// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
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
    typename Candidate::NavigatorUpdate;
    typename Candidate::Navigator;

    requires navkit::core::config::ConfigPolicy<Candidate>;
    requires navkit::core::estimation::StateDefPolicy<typename Candidate::StateDef>;
    requires navkit::core::estimation::SensorCollectionPolicy<typename Candidate::Sensors>;
    requires navkit::core::estimation::sensor_ids_unique_v<typename Candidate::Sensors>;
    requires navkit::core::estimation::FilterPolicy<typename Candidate::Filter>;
    requires std::same_as<typename Candidate::Filter::Sensors_t, typename Candidate::Sensors>;
    requires navkit::core::profiling::ProfilerPolicy<typename Candidate::Profiler>;
    requires navkit::core::estimation::NavigatorUpdatePolicy<typename Candidate::NavigatorUpdate,
                                                             typename Candidate::Filter,
                                                             typename Candidate::Sensors>;
};

} // namespace navkit::api::config
