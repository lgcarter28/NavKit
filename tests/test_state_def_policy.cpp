// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/State.hpp"
#include "navkit/core/StateDefPolicy.hpp"
#include "navkit/core/StateDefs.hpp"

#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::test
{

struct ValidStateDef
{
    using Scalar_t = double;
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;

    static constexpr int N = 6;
};

struct MissingScalarStateDef
{
    static constexpr int N = 3;
};

struct MissingDimensionStateDef
{
    using Scalar_t = double;
};

struct InvalidDimensionStateDef
{
    using Scalar_t = double;

    static constexpr int N = 0;
};

struct ValidSegment
{
    static constexpr int i = 2;
    static constexpr int sz = 3;
};

struct InvalidSegmentSize
{
    static constexpr int i = 2;
    static constexpr int sz = 0;
};

struct InvalidSegmentIndex
{
    static constexpr int i = -1;
    static constexpr int sz = 3;
};

TEST_CASE("StateDefPolicy accepts valid state definitions")
{
    static_assert(StateDefPolicy<ValidStateDef>);
    static_assert(StateDefPolicy<InsStateDef>);
    static_assert(StateDefPolicy<GnssTcStateDef>);

    CHECK(ValidStateDef::N == 6);
    CHECK(InsStateDef::N == 21);
    CHECK(GnssTcStateDef::N == 23);
}

TEST_CASE("StateDefPolicy rejects invalid state definitions")
{
    // Missing Scalar_t means NavKit cannot form State<StateDef> with the
    // configured numeric scalar type.
    static_assert(!StateDefPolicy<MissingScalarStateDef>);

    // Missing N means NavKit cannot allocate a fixed-size state vector.
    static_assert(!StateDefPolicy<MissingDimensionStateDef>);

    // N must be strictly positive.
    static_assert(!StateDefPolicy<InvalidDimensionStateDef>);

    CHECK(true);
}

TEST_CASE("SegmentPolicy accepts and rejects segment metadata")
{
    static_assert(SegmentPolicy<ValidSegment>);
    static_assert(SegmentPolicy<ValidStateDef::Pos>);
    static_assert(SegmentPolicy<ValidStateDef::Vel>);

    // Segment size must be positive.
    static_assert(!SegmentPolicy<InvalidSegmentSize>);

    // Segment start index must be non-negative.
    static_assert(!SegmentPolicy<InvalidSegmentIndex>);

    CHECK(true);
}

TEST_CASE("State and StateCov use StateDefPolicy-constrained fixed sizes")
{
    using X = State<ValidStateDef>;
    using P = StateCov<ValidStateDef>;

    static_assert(X::RowsAtCompileTime == ValidStateDef::N);
    static_assert(X::ColsAtCompileTime == 1);
    static_assert(P::RowsAtCompileTime == ValidStateDef::N);
    static_assert(P::ColsAtCompileTime == ValidStateDef::N);
    static_assert(std::is_same_v<X::Scalar, ValidStateDef::Scalar_t>);

    X x = X::Zero();
    P p = P::Identity();

    CHECK(x.size() == ValidStateDef::N);
    CHECK(p.rows() == ValidStateDef::N);
    CHECK(p.cols() == ValidStateDef::N);
}

} // namespace navkit::test
