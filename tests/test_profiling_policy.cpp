// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"
#include "test_main.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace navkit::core::profiling::test
{

struct FakeClock
{
    using Tick = std::uint32_t;

    static inline Tick tick{0U};

    static Tick now()
    {
        return tick;
    }
};

struct FixedSink
{
    static inline std::array<ProfileRecord<FakeClock::Tick>, 4> records{};
    static inline std::size_t count{0U};

    static void reset()
    {
        records = {};
        count = 0U;
    }

    static void record(ProfileRecord<FakeClock::Tick> record)
    {
        if (count < records.size()) {
            records[count] = record;
            ++count;
        }
    }
};

struct MissingTickClock
{
    static std::uint32_t now();
};

struct WrongNowClock
{
    using Tick = std::uint32_t;

    static double now();
};

struct MissingRecordSink
{};

struct WrongRecordSink
{
    static int record(ProfileRecord<FakeClock::Tick>);
};

struct MissingScopeProfiler
{
    static NullProfileScope profile(ProfilePoint);
};

struct MissingProfileFunctionProfiler
{
    using Scope = NullProfileScope;
};

using FakeProfiler = ScopedProfiler<FakeClock, FixedSink>;

TEST_CASE("Profiler concepts accept valid clock, sink, scope, and profiler policies")
{
    static_assert(ClockPolicy<FakeClock>);
    static_assert(ProfileSinkPolicy<FixedSink, FakeClock>);
    static_assert(ProfileScopePolicy<NullProfileScope>);
    static_assert(ProfileScopePolicy<ProfileScope<FakeClock, FixedSink>>);
    static_assert(ProfilerPolicy<NullProfiler>);
    static_assert(ProfilerPolicy<FakeProfiler>);

    CHECK(true);
}

TEST_CASE("Profiler concepts reject missing capabilities")
{
    static_assert(!ClockPolicy<MissingTickClock>);
    static_assert(!ClockPolicy<WrongNowClock>);
    static_assert(!ProfileSinkPolicy<MissingRecordSink, FakeClock>);
    static_assert(!ProfileSinkPolicy<WrongRecordSink, FakeClock>);
    static_assert(!ProfilerPolicy<MissingScopeProfiler>);
    static_assert(!ProfilerPolicy<MissingProfileFunctionProfiler>);

    CHECK(true);
}

TEST_CASE("ScopedProfiler records elapsed ticks through the configured sink")
{
    FakeClock::tick = 100U;
    FixedSink::reset();

    {
        auto scope = FakeProfiler::profile(ProfilePoint::KalmanObservationUpdate);
        FakeClock::tick = 137U;
    }

    REQUIRE(FixedSink::count == 1U);
    CHECK(FixedSink::records[0].point == ProfilePoint::KalmanObservationUpdate);
    CHECK(FixedSink::records[0].start_tick == 100U);
    CHECK(FixedSink::records[0].elapsed_ticks == 37U);
    CHECK(FixedSink::records[0].sequence == 0U);
    CHECK(FixedSink::records[0].parent_sequence == 0U);
    CHECK(FixedSink::records[0].depth == 0U);
    CHECK(FixedSink::records[0].flags == ProfileRecordFlags::None);
}

TEST_CASE("Moved profile scope records exactly once")
{
    FakeClock::tick = 10U;
    FixedSink::reset();

    {
        auto scope = FakeProfiler::profile(ProfilePoint::NavigatorProcessMeasurements);
        FakeClock::tick = 20U;
        auto moved_scope = static_cast<ProfileScope<FakeClock, FixedSink>&&>(scope);
        FakeClock::tick = 25U;
    }

    REQUIRE(FixedSink::count == 1U);
    CHECK(FixedSink::records[0].point == ProfilePoint::NavigatorProcessMeasurements);
    CHECK(FixedSink::records[0].start_tick == 10U);
    CHECK(FixedSink::records[0].elapsed_ticks == 15U);
    CHECK(FixedSink::records[0].sequence == 0U);
    CHECK(FixedSink::records[0].parent_sequence == 0U);
    CHECK(FixedSink::records[0].depth == 0U);
    CHECK(FixedSink::records[0].flags == ProfileRecordFlags::None);
}

TEST_CASE("ProfileRecord carries optional visualization metadata")
{
    ProfileRecord<FakeClock::Tick> record{
        .point = ProfilePoint::PropagationUpdate,
        .start_tick = 4U,
        .elapsed_ticks = 9U,
        .sequence = 11U,
        .parent_sequence = 7U,
        .depth = 2U,
        .flags = ProfileRecordFlags::DroppedBefore,
    };

    CHECK(record.point == ProfilePoint::PropagationUpdate);
    CHECK(record.start_tick == 4U);
    CHECK(record.elapsed_ticks == 9U);
    CHECK(record.sequence == 11U);
    CHECK(record.parent_sequence == 7U);
    CHECK(record.depth == 2U);
    CHECK(record.flags == ProfileRecordFlags::DroppedBefore);
}

TEST_CASE("NullProfiler satisfies the profiler contract without recording")
{
    auto scope = NullProfiler::profile(ProfilePoint::PropagationUpdate);
    static_cast<void>(scope);

    static_assert(ProfilerPolicy<NullProfiler>);
    CHECK(true);
}

} // namespace navkit::core::profiling::test
