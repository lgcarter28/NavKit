// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/RingBuffer.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"

#include <cmath>
#include <cstddef>

namespace navkit::sim
{

/**
 * Fixed-capacity moving-window gyro observation formed from real IMU increments.
 *
 * The interval-average rate is the accumulated angular increment divided by the
 * accumulated elapsed time. This remains correct when rational scheduling yields
 * slightly nonuniform scalar-second intervals.
 */
template<std::size_t Capacity>
class ImuIncrementMovingAverage
{
public:
    [[nodiscard]] bool initialize(const std::size_t window_samples)
    {
        if (m_initialized || window_samples == 0U || window_samples > Capacity) {
            return false;
        }
        m_window_samples = window_samples;
        m_initialized = true;
        return true;
    }

    [[nodiscard]] bool push(const core::estimation::ImuIncrement& increment)
    {
        if (!m_initialized || !std::isfinite(increment.dt_s) || increment.dt_s <= 0.0 ||
            !increment.delta_theta_ib_b_rad.allFinite()) {
            return false;
        }

        if (m_increments.size() == m_window_samples) {
            core::estimation::ImuIncrement oldest{};
            if (!m_increments.pop(oldest)) {
                return false;
            }
            m_delta_theta_sum_rad -= oldest.delta_theta_ib_b_rad;
            m_dt_sum_s -= oldest.dt_s;
        }
        if (!m_increments.push(increment)) {
            return false;
        }
        m_delta_theta_sum_rad += increment.delta_theta_ib_b_rad;
        m_dt_sum_s += increment.dt_s;
        return true;
    }

    [[nodiscard]] bool average_rate(core::Vec3& w_ib_b_radps) const
    {
        w_ib_b_radps = core::Vec3::Zero();
        if (!m_initialized || m_increments.empty() || !std::isfinite(m_dt_sum_s) ||
            m_dt_sum_s <= 0.0) {
            return false;
        }
        w_ib_b_radps = m_delta_theta_sum_rad / m_dt_sum_s;
        return w_ib_b_radps.allFinite();
    }

    void clear()
    {
        m_increments.clear();
        m_delta_theta_sum_rad.setZero();
        m_dt_sum_s = 0.0;
    }

    [[nodiscard]] std::size_t size() const
    {
        return m_increments.size();
    }

    [[nodiscard]] std::size_t window_samples() const
    {
        return m_window_samples;
    }

private:
    core::containers::RingBuffer<core::estimation::ImuIncrement, Capacity> m_increments{};
    core::Vec3 m_delta_theta_sum_rad{core::Vec3::Zero()};
    core::Time_t m_dt_sum_s{};
    std::size_t m_window_samples{};
    bool m_initialized{false};
};

} // namespace navkit::sim
