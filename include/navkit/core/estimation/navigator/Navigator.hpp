// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/RingBuffer.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
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
         PropagationPolicy<typename Filter::StateDef_t> Propagation = NoOpPropagation,
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
    using ImuBuffer_t =
        navkit::core::containers::RingBuffer<ImuIncrement, Propagation_t::imu_buffer_capacity>;
    struct CovarianceStep
    {
        typename Filter_t::P_t phi{Filter_t::P_t::Identity()};
        typename Filter_t::P_t qd{Filter_t::P_t::Zero()};
    };

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

    [[nodiscard]] bool push_imu(const ImuIncrement& increment)
    {
        return m_imu_buffer.push(increment);
    }

    [[nodiscard]] std::size_t imu_buffer_size() const
    {
        return m_imu_buffer.size();
    }

    [[nodiscard]] bool last_propagation_success() const
    {
        return m_last_propagation_success;
    }

    [[nodiscard]] std::size_t pending_covariance_step_count() const
    {
        return m_has_pending_covariance_step ? 1U : 0U;
    }

    bool process_strapdown_integration()
    {
        m_last_propagation_success = true;

        while (m_imu_buffer.size() >= 2U) {
            ImuIncrement first{};
            ImuIncrement second{};
            const bool popped_pair = m_imu_buffer.pop(first) && m_imu_buffer.pop(second);
            if (!popped_pair || !process_imu_pair(first, second)) {
                m_last_propagation_success = false;
                return false;
            }
        }

        if (!m_imu_buffer.empty()) {
            ImuIncrement increment{};
            if (!m_imu_buffer.pop(increment) || !process_imu_single(increment)) {
                m_last_propagation_success = false;
                return false;
            }
        }

        return true;
    }

    bool propagate_covariance()
    {
        if (m_has_pending_covariance_step) {
            m_filter.propagate_covariance(m_pending_covariance_step.phi,
                                          m_pending_covariance_step.qd);
            m_pending_covariance_step = {};
            m_has_pending_covariance_step = false;
        }
        return true;
    }

    void process_measurements()
    {
        auto profile_scope =
            Profiler::profile(navkit::core::profiling::ProfilePoint::NavigatorProcessMeasurements);
        static_cast<void>(profile_scope);

        std::apply([this](auto&... sensor_obj) { (this->process_one_sensor(sensor_obj), ...); },
                   m_sensors);
        Update_t::filter_update(m_filter);
    }

    bool update()
    {
        const bool propagation_ok = process_strapdown_integration();
        const bool covariance_ok = propagate_covariance();
        process_measurements();
        return propagation_ok && covariance_ok;
    }

private:
    bool process_imu_single(const ImuIncrement& increment)
    {
        using StateDef = typename Filter_t::StateDef_t;
        const auto state_before = m_filter.state();
        CovarianceStep step{};
        if (!Propagation_t::template covariance_step_from_increment<StateDef>(
                state_before, increment, step.phi, step.qd)) {
            return false;
        }
        if (!Propagation_t::template process_imu_increment<StateDef>(m_filter.state(), increment)) {
            return false;
        }
        accumulate_covariance_step(step);
        return true;
    }

    bool process_imu_pair(const ImuIncrement& first, const ImuIncrement& second)
    {
        using StateDef = typename Filter_t::StateDef_t;
        const auto state_before = m_filter.state();
        CovarianceStep step{};
        if (!Propagation_t::template covariance_step_from_increment_pair<StateDef>(
                state_before, first, second, step.phi, step.qd)) {
            return false;
        }
        if (!Propagation_t::template process_imu_increment_pair<StateDef>(
                m_filter.state(), first, second)) {
            return false;
        }
        accumulate_covariance_step(step);
        return true;
    }

    void accumulate_covariance_step(const CovarianceStep& step)
    {
        if (!m_has_pending_covariance_step) {
            m_pending_covariance_step = step;
            m_has_pending_covariance_step = true;
            return;
        }

        m_pending_covariance_step.qd =
            (step.phi * m_pending_covariance_step.qd * step.phi.transpose()) + step.qd;
        m_pending_covariance_step.qd =
            0.5 * (m_pending_covariance_step.qd + m_pending_covariance_step.qd.transpose());
        m_pending_covariance_step.phi = step.phi * m_pending_covariance_step.phi;
    }

    Filter_t m_filter{};
    Sensors_t m_sensors{};
    ImuBuffer_t m_imu_buffer{};
    CovarianceStep m_pending_covariance_step{};
    bool m_has_pending_covariance_step{false};
    bool m_last_propagation_success{true};
};

} // namespace navkit::core::estimation
