#pragma once

#include "navkit/core/Segment.hpp"

namespace navkit
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
        AdditiveInjection<typename StateDef::Pos>::apply(x, dx);
        AdditiveInjection<typename StateDef::Vel>::apply(x, dx);
        AdditiveInjection<typename StateDef::Att>::apply(x, dx);
        AdditiveInjection<typename StateDef::GyroB>::apply(x, dx);
        AdditiveInjection<typename StateDef::GyroSf>::apply(x, dx);
        AdditiveInjection<typename StateDef::AccB>::apply(x, dx);
        AdditiveInjection<typename StateDef::AccSf>::apply(x, dx);
    }
};

template<typename StateDef>
using DefaultInjectionPolicy = InsInjectionPolicy<StateDef>;

} // namespace navkit
