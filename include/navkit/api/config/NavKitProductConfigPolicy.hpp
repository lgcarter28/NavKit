// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigTraits.hpp"
#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/filter/FilterConfigPolicy.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"

namespace navkit::api::config
{

template<typename Candidate>
concept NavKitProductConfigPolicy = requires {
    typename Candidate::Numeric;
    typename Candidate::StateDef;
    typename Candidate::Sensors;
    typename Candidate::MeasurementStatisticsTuple;
    typename Candidate::Profiler;
    typename Candidate::Filter;
    typename Candidate::NavigatorUpdate;
    typename Candidate::Navigator;

    requires navkit::core::config::ConfigPolicy<Candidate>;
    requires navkit::core::estimation::StateDefPolicy<typename Candidate::StateDef>;
    requires navkit::core::estimation::SensorCollectionPolicy<typename Candidate::Sensors>;
    requires navkit::core::estimation::sensor_ids_unique_v<typename Candidate::Sensors>;
    requires navkit::core::estimation::MeasurementStatisticsCollectionPolicy<
        typename Candidate::MeasurementStatisticsTuple>;
    requires detail::measurement_statistics_sources_configured_v<
        typename Candidate::MeasurementStatisticsTuple,
        typename Candidate::Sensors>;
    requires navkit::core::profiling::ProfilerPolicy<typename Candidate::Profiler>;
    requires navkit::core::estimation::NavigatorUpdatePolicy<typename Candidate::NavigatorUpdate,
                                                             typename Candidate::Filter,
                                                             typename Candidate::Sensors>;
};

} // namespace navkit::api::config
