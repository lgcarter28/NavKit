// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePolicy.hpp"

namespace navkit::core::profiling
{

template<ClockPolicy Clock, ProfileSinkPolicy<Clock> Sink>
class ProfileScope
{
public:
    using Tick = typename Clock::Tick;

    explicit ProfileScope(ProfilePoint point)
        : m_point(point)
        , m_start_tick(Clock::now())
    {}

    ProfileScope(const ProfileScope&) = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;

    ProfileScope(ProfileScope&& other) noexcept
        : m_point(other.m_point)
        , m_start_tick(other.m_start_tick)
        , m_active(other.m_active)
    {
        other.m_active = false;
    }

    ProfileScope& operator=(ProfileScope&& other) noexcept
    {
        if (this != &other) {
            close();
            m_point = other.m_point;
            m_start_tick = other.m_start_tick;
            m_active = other.m_active;
            other.m_active = false;
        }
        return *this;
    }

    ~ProfileScope()
    {
        close();
    }

private:
    void close()
    {
        if (!m_active) {
            return;
        }

        const Tick end_tick = Clock::now();
        Sink::record(ProfileRecord<Tick>{
            .point = m_point,
            .start_tick = m_start_tick,
            .elapsed_ticks = end_tick - m_start_tick,
        });
        m_active = false;
    }

    ProfilePoint m_point{};
    Tick m_start_tick{};
    bool m_active{true};
};

template<ClockPolicy Clock, ProfileSinkPolicy<Clock> Sink>
struct ScopedProfiler
{
    using Scope = ProfileScope<Clock, Sink>;

    [[nodiscard]] static Scope profile(ProfilePoint point)
    {
        return Scope{point};
    }
};

} // namespace navkit::core::profiling
