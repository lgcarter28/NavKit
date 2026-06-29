#pragma once

#include "navkit/core/KalmanFilter.hpp"
#include "navkit/core/policies/UpdatePolicies.hpp"

#include <tuple>
#include <type_traits>

namespace navkit
{

template<typename Filter,
         typename SensorTuple,
         template<typename> typename UpdatePolicyTemplate = UpdatePostFilter>
class Navigator
{
public:
    using Filter_t = Filter;
    using Sensors_t = SensorTuple;
    using UpdatePolicy = UpdatePolicyTemplate<Filter_t>;

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
        UpdatePolicy::template sensor_update<SensorNoRef>(m_filter, sensor_obj);
    }

    void process_measurements()
    {
        std::apply([this](auto&... sensor_obj) { (process_one_sensor(sensor_obj), ...); },
                   m_sensors);
        UpdatePolicy::filter_update(m_filter);
    }

private:
    Filter_t m_filter{};
    Sensors_t m_sensors{};
};

} // namespace navkit
