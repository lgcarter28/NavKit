// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/NavigatorUpdatePolicy.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicies.hpp"
#include "navkit/core/estimation/navigator/propagation/PropagationPolicy.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

template<FilterPolicy Filter,
         SensorCollectionPolicy SensorTuple,
         PropagationPolicy<Filter, SensorTuple> Propagation = NoOpPropagation,
         NavigatorUpdatePolicy<Filter, SensorTuple> Update = UpdatePostFilter<Filter>,
         navkit::core::profiling::ProfilerPolicy Profiler = navkit::core::profiling::NullProfiler>
class Navigator
{
public:
    using Filter_t = Filter;
    using Sensors_t = SensorTuple;
    using Propagation_t = Propagation;
    using Update_t = Update;
    using Profiler_t = Profiler;

    Filter_t& filter()
    {
        return m_filter;
    }
    [[nodiscard]] const Filter_t& filter() const
    {
        return m_filter;
    }

    Sensors_t& sensors()
    {
        return m_sensors;
    }
    [[nodiscard]] const Sensors_t& sensors() const
    {
        return m_sensors;
    }

    template<std::size_t I>
    auto& sensor()
    {
        return std::get<I>(m_sensors);
    }

    template<SensorPolicy Sensor>
    void process_one_sensor(Sensor& sensor_obj)
    {
        m_filter.process_sensor(sensor_obj);
        Update_t::template sensor_update<Sensor>(m_filter, sensor_obj);
    }

    void propagate()
    {
        Propagation_t::propagate(m_filter, m_sensors);
    }

    void process_measurements()
    {
        auto profile_scope =
            Profiler::profile(navkit::core::profiling::ProfilePoint::NavigatorProcessMeasurements);
        static_cast<void>(profile_scope);

        propagate();
        std::apply([this](auto&... sensor_obj) { (this->process_one_sensor(sensor_obj), ...); },
                   m_sensors);
        Update_t::filter_update(m_filter);
    }

private:
    Filter_t m_filter{};
    Sensors_t m_sensors{};
};

} // namespace navkit::core::estimation
