// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/SensorGraphConfigPolicy.hpp"
#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"

namespace navkit::api::config
{

template<typename Candidate>
concept NavKitProductConfigPolicy = requires {
    typename Candidate::Numeric;
    typename Candidate::StateDef;
    typename Candidate::Sensors;
    typename Candidate::MeasurementModels;
    typename Candidate::Profiler;
    typename Candidate::Filter;
    typename Candidate::Navigator;

    requires navkit::core::config::ConfigPolicy<Candidate>;
    requires navkit::core::estimation::StateDefPolicy<typename Candidate::StateDef>;
    requires SensorGraphConfigPolicy<Candidate>;
    requires navkit::core::profiling::ProfilerPolicy<typename Candidate::Profiler>;
};

} // namespace navkit::api::config
