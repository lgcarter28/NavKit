// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/FilterPolicy.hpp"
#include "navkit/core/KalmanFilter.hpp"
#include "navkit/core/SensorCollectionPolicy.hpp"
#include "navkit/core/UpdatePolicy.hpp"
#include "navkit/core/policies/UpdatePolicies.hpp"

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace navkit
{

namespace detail
{

template<typename Filter, typename Update, typename SensorTuple, typename Indices>
struct NavigatorPolicyCompatibility;

template<typename Filter, typename Update, typename SensorTuple, std::size_t... Is>
struct NavigatorPolicyCompatibility<Filter, Update, SensorTuple, std::index_sequence<Is...>>
{
    using Tuple = std::remove_cvref_t<SensorTuple>;

    static constexpr bool value =
        ((FilterPolicy<Filter, std::tuple_element_t<Is, Tuple>> &&
          UpdatePolicy<Update, Filter, std::tuple_element_t<Is, Tuple>>) &&
         ...);
};

template<typename Filter, typename Update, SensorCollectionPolicy SensorTuple>
inline constexpr bool navigator_policy_compatible_v = NavigatorPolicyCompatibility<
    Filter,
    Update,
    SensorTuple,
    std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<SensorTuple>>>>::value;

} // namespace detail

template<typename Filter,
         SensorCollectionPolicy SensorTuple,
         template<typename> typename UpdatePolicyTemplate = UpdatePostFilter>
    requires detail::
        navigator_policy_compatible_v<Filter, UpdatePolicyTemplate<Filter>, SensorTuple>
    class Navigator
{
public:
    using Filter_t = Filter;
    using Sensors_t = SensorTuple;
    using UpdatePolicy_t = UpdatePolicyTemplate<Filter_t>;

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
        std::apply([this](auto&... sensor_obj) { (process_one_sensor(sensor_obj), ...); },
                   m_sensors);
        UpdatePolicy_t::filter_update(m_filter);
    }

private:
    Filter_t m_filter{};
    Sensors_t m_sensors{};
};

} // namespace navkit
