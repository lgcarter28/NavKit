// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/NavigatorPolicyCompatibility.hpp"
#include "navkit/core/estimation/navigator/SensorCollectionPolicy.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

template<typename Filter,
         SensorCollectionPolicy SensorTuple,
         template<typename> typename UpdatePolicyTemplate = UpdatePostFilter,
         navkit::core::profiling::ProfilerPolicy Profiler = navkit::core::profiling::NullProfiler>
    requires detail::
        navigator_policy_compatible_v<Filter, UpdatePolicyTemplate<Filter>, SensorTuple>
    class Navigator
{
public:
    using Filter_t = Filter;
    using Sensors_t = SensorTuple;
    using UpdatePolicy_t = UpdatePolicyTemplate<Filter_t>;
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

    template<typename Sensor>
    void process_one_sensor(Sensor& sensor_obj)
    {
        using SensorNoRef = std::remove_reference_t<Sensor>;
        m_filter.process_sensor(sensor_obj);
        UpdatePolicy_t::template sensor_update<SensorNoRef>(m_filter, sensor_obj);
    }

    void process_measurements()
    {
        auto profile_scope =
            Profiler::profile(navkit::core::profiling::ProfilePoint::NavigatorProcessMeasurements);
        static_cast<void>(profile_scope);

        std::apply([this](auto&... sensor_obj) { (process_one_sensor(sensor_obj), ...); },
                   m_sensors);
        UpdatePolicy_t::filter_update(m_filter);
    }

private:
    Filter_t m_filter{};
    Sensors_t m_sensors{};
};

} // namespace navkit::core::estimation
