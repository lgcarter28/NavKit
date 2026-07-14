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

struct PolicyTestNominalStateDef
{
    using Scalar_t = double;
    static constexpr int N = 3;
};

struct PolicyTestErrorStateDef
{
    using Scalar_t = double;
    static constexpr int N = 3;
};

struct PolicyTestStateDef
{
    using Nominal = PolicyTestNominalStateDef;
    using Error = PolicyTestErrorStateDef;
};

struct ValidInjection
{
    static void apply(NominalState<PolicyTestStateDef>& x, const ErrorState<PolicyTestStateDef>& dx)
    {
        x -= dx;
    }
};

struct MissingInjectionApply
{};

struct MutableErrorInjection
{
    static void apply(NominalState<PolicyTestStateDef>& unused_state,
                      ErrorState<PolicyTestStateDef>& unused_dx)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
    }
};

struct ValidReset
{
    static void reset_covariance(NominalState<PolicyTestStateDef>& unused_state,
                                 ErrorState<PolicyTestStateDef>& unused_dx,
                                 ErrorStateCov<PolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }

    static void reset_dx(ErrorState<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingCovarianceReset
{
    static void reset_dx(ErrorState<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingErrorStateReset
{
    static void reset_covariance(NominalState<PolicyTestStateDef>& unused_state,
                                 ErrorState<PolicyTestStateDef>& unused_dx,
                                 ErrorStateCov<PolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }
};

TEST_CASE("InjectionPolicy accepts compatible policies")
{
    static_assert(InjectionPolicy<ValidInjection, PolicyTestStateDef>);
    static_assert(InjectionPolicy<DefaultInjectionPolicy<DefaultInsStateDef>, DefaultInsStateDef>);

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
    static_assert(ResetPolicy<DefaultResetPolicy<DefaultInsStateDef>, DefaultInsStateDef>);

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
