// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicy.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicy.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"

#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::core::estimation::test
{

struct PolicyTestStateDef
{
    using Scalar_t = double;
    static constexpr int N = 3;
};

struct ValidInjection
{
    static void apply(State<PolicyTestStateDef>& x, const State<PolicyTestStateDef>& dx)
    {
        x -= dx;
    }
};

struct MissingInjectionApply
{};

struct MutableErrorInjection
{
    static void apply(State<PolicyTestStateDef>&, State<PolicyTestStateDef>&) {}
};

struct ValidReset
{
    static void reset_covariance(State<PolicyTestStateDef>&,
                                 const State<PolicyTestStateDef>&,
                                 StateCov<PolicyTestStateDef>&)
    {}

    static void reset_dx(State<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingCovarianceReset
{
    static void reset_dx(State<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingErrorStateReset
{
    static void reset_covariance(State<PolicyTestStateDef>&,
                                 const State<PolicyTestStateDef>&,
                                 StateCov<PolicyTestStateDef>&)
    {}
};

TEST_CASE("InjectionPolicy accepts compatible policies")
{
    static_assert(InjectionPolicy<ValidInjection, PolicyTestStateDef>);
    static_assert(InjectionPolicy<DefaultInjectionPolicy<InsStateDef>, InsStateDef>);

    CHECK(true);
}

TEST_CASE("InjectionPolicy rejects incompatible policies")
{
    static_assert(!InjectionPolicy<MissingInjectionApply, PolicyTestStateDef>);
    static_assert(!InjectionPolicy<MutableErrorInjection, PolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("ResetPolicy accepts compatible policies")
{
    static_assert(ResetPolicy<ValidReset, PolicyTestStateDef>);
    static_assert(ResetPolicy<DefaultResetPolicy<InsStateDef>, InsStateDef>);

    CHECK(true);
}

TEST_CASE("ResetPolicy rejects incomplete policies")
{
    static_assert(!ResetPolicy<MissingCovarianceReset, PolicyTestStateDef>);
    static_assert(!ResetPolicy<MissingErrorStateReset, PolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("KalmanFilter accepts constrained injection and reset policies")
{
    using Filter = KalmanFilter<PolicyTestStateDef, ValidInjection, ValidReset>;
    static_assert(std::is_default_constructible_v<Filter>);

    Filter filter;
    CHECK(filter.state().isZero());
}

} // namespace navkit::core::estimation::test
