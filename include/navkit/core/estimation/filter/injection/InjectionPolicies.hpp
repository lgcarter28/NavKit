// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/Segment.hpp"

namespace navkit::core::estimation
{

template<typename Seg>
struct AdditiveInjection
{
    template<typename State_t>
    static void apply(State_t& x, const State_t& dx)
    {
        segment<Seg>(x) -= segment<Seg>(dx);
    }
};

template<typename StateDef>
struct InsInjectionPolicy
{
    template<typename State_t>
    static void apply(State_t& x, const State_t& dx)
    {
        apply_if_present<typename StateDef::Pos>(x, dx);
        apply_if_present<typename StateDef::Vel>(x, dx);
        apply_if_present<typename StateDef::Att>(x, dx);
        if constexpr (requires { typename StateDef::GyroB; }) {
            apply_if_present<typename StateDef::GyroB>(x, dx);
        }
        if constexpr (requires { typename StateDef::GyroSf; }) {
            apply_if_present<typename StateDef::GyroSf>(x, dx);
        }
        if constexpr (requires { typename StateDef::AccB; }) {
            apply_if_present<typename StateDef::AccB>(x, dx);
        }
        if constexpr (requires { typename StateDef::AccSf; }) {
            apply_if_present<typename StateDef::AccSf>(x, dx);
        }
        if constexpr (requires { typename StateDef::ClkB; }) {
            apply_if_present<typename StateDef::ClkB>(x, dx);
        }
        if constexpr (requires { typename StateDef::ClkD; }) {
            apply_if_present<typename StateDef::ClkD>(x, dx);
        }
    }

private:
    template<typename Segment, typename State_t>
    static void apply_if_present(State_t& x, const State_t& dx)
    {
        AdditiveInjection<Segment>::apply(x, dx);
    }
};

template<typename StateDef>
using DefaultInjectionPolicy = InsInjectionPolicy<StateDef>;

} // namespace navkit::core::estimation
