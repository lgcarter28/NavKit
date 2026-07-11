// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/profiling/ClockPolicy.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfileScopePolicy.hpp"
#include "navkit/core/profiling/ProfileSinkPolicy.hpp"
#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"
#include "navkit/core/profiling/ScopedProfiler.hpp"
#include "test_main.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>

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

struct SteppingClock
{
    using Tick = std::uint32_t;

    static inline std::array<Tick, 8> ticks{};
    static inline std::size_t index{0U};

    static void reset(std::array<Tick, 8> values)
    {
        ticks = values;
        index = 0U;
    }

    static Tick now()
    {
        const Tick value = ticks.at(index);
        if (index + 1U < ticks.size()) {
            ++index;
        }
        return value;
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
            records.at(count) = record;
            ++count;
        }
    }

    static const ProfileRecord<FakeClock::Tick>& first_record()
    {
        return records.at(0U);
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
using SteppingProfiler = ScopedProfiler<SteppingClock, FixedSink>;
using ProfiledStateDef = navkit::core::estimation::InsStateDef;
using ProfiledModel = navkit::core::models::GnssPosModel<ProfiledStateDef>;
using ProfiledFilter = navkit::core::estimation::KalmanFilter<
    ProfiledStateDef,
    navkit::core::estimation::DefaultInjectionPolicy<ProfiledStateDef>,
    navkit::core::estimation::DefaultResetPolicy<ProfiledStateDef>,
    std::tuple<>,
    SteppingProfiler>;
using ProfiledSensor = navkit::core::estimation::Sensor<0U, ProfiledModel, 4>;
using ProfiledNavigatorFilter = navkit::core::estimation::KalmanFilter<ProfiledStateDef>;
using ProfiledNavigatorUpdate = navkit::core::estimation::UpdatePostFilter<ProfiledNavigatorFilter>;
using ProfiledNavigator = navkit::core::estimation::Navigator<ProfiledNavigatorFilter,
                                                              std::tuple<ProfiledSensor>,
                                                              ProfiledNavigatorUpdate,
                                                              SteppingProfiler>;

TEST_CASE("Profiler concepts accept valid clock, sink, scope, and profiler policies")
{
    using RejectSink = RingBufferProfileSink<FakeClock::Tick, 2U>;

    static_assert(ClockPolicy<FakeClock>);
    static_assert(ProfileSinkPolicy<FixedSink, FakeClock>);
    static_assert(ProfileSinkPolicy<NullProfileSink, FakeClock>);
    static_assert(ProfileSinkPolicy<RejectSink, FakeClock>);
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
    const auto& record = FixedSink::first_record();
    CHECK(record.point == ProfilePoint::KalmanObservationUpdate);
    CHECK(record.start_tick == 100U);
    CHECK(record.elapsed_ticks == 37U);
    CHECK(record.sequence == 0U);
    CHECK(record.parent_sequence == 0U);
    CHECK(record.depth == 0U);
    CHECK(record.flags == ProfileRecordFlags::None);
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
    const auto& record = FixedSink::first_record();
    CHECK(record.point == ProfilePoint::NavigatorProcessMeasurements);
    CHECK(record.start_tick == 10U);
    CHECK(record.elapsed_ticks == 15U);
    CHECK(record.sequence == 0U);
    CHECK(record.parent_sequence == 0U);
    CHECK(record.depth == 0U);
    CHECK(record.flags == ProfileRecordFlags::None);
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
    CHECK(has_profile_record_flag(record.flags, ProfileRecordFlags::DroppedBefore));
    CHECK(!has_profile_record_flag(record.flags, ProfileRecordFlags::Incomplete));

    using CompactRecord = ProfileRecord<std::uint32_t, std::uint16_t, std::uint8_t>;
    static_assert(std::is_same_v<CompactRecord::Tick_t, std::uint32_t>);
    static_assert(std::is_same_v<CompactRecord::Sequence_t, std::uint16_t>);
    static_assert(std::is_same_v<CompactRecord::Depth_t, std::uint8_t>);
    static_assert(std::is_trivially_copyable_v<CompactRecord>);
    static_assert(std::is_standard_layout_v<CompactRecord>);
}

TEST_CASE("NullProfiler satisfies the profiler contract without recording")
{
    auto scope = NullProfiler::profile(ProfilePoint::PropagationUpdate);
    static_cast<void>(scope);

    static_assert(ProfilerPolicy<NullProfiler>);
    static_assert(std::is_empty_v<NullProfiler>);
    static_assert(std::is_empty_v<NullProfileScope>);
    static_assert(std::is_trivially_default_constructible_v<NullProfileScope>);
    static_assert(std::is_trivially_destructible_v<NullProfileScope>);
    static_assert(std::is_nothrow_destructible_v<NullProfileScope>);
    static_assert(noexcept(NullProfiler::profile(ProfilePoint::PropagationUpdate)));
    static_assert(std::is_same_v<decltype(NullProfiler::profile(ProfilePoint::PropagationUpdate)),
                                 NullProfileScope>);
    CHECK(true);
}

TEST_CASE("Core profile sinks expose fixed-capacity embedded resource contracts")
{
    using RejectSink = RingBufferProfileSink<std::uint32_t, 4U>;
    using OverwriteSink =
        RingBufferProfileSink<std::uint32_t, 8U, containers::OverflowPolicy::OverwriteOldest>;

    static_assert(ProfileSinkPolicy<RejectSink, FakeClock>);
    static_assert(std::is_same_v<RejectSink::Record, ProfileRecord<std::uint32_t>>);
    static_assert(std::is_trivially_copyable_v<RejectSink::Record>);
    static_assert(std::is_standard_layout_v<RejectSink::Record>);
    static_assert(!std::is_polymorphic_v<RejectSink>);
    static_assert(RejectSink::capacity() == 4U);
    static_assert(RejectSink::overflow_policy() == containers::OverflowPolicy::Reject);
    static_assert(OverwriteSink::capacity() == 8U);
    static_assert(OverwriteSink::overflow_policy() == containers::OverflowPolicy::OverwriteOldest);

    RejectSink::reset();
    CHECK(RejectSink::empty());
    CHECK(RejectSink::size() == 0U);
    CHECK(RejectSink::dropped_count() == 0U);
}

TEST_CASE("RingBufferProfileSink rejects overflow and records dropped count")
{
    using RejectSink = RingBufferProfileSink<FakeClock::Tick, 2U>;

    RejectSink::reset();
    RejectSink::record({.point = ProfilePoint::NavigatorProcessMeasurements, .start_tick = 1U});
    RejectSink::record({.point = ProfilePoint::KalmanObservationUpdate, .start_tick = 2U});
    RejectSink::record({.point = ProfilePoint::PropagationUpdate, .start_tick = 3U});

    CHECK(RejectSink::size() == 2U);
    CHECK(RejectSink::dropped_count() == 1U);

    ProfileRecord<FakeClock::Tick> record{};
    REQUIRE(RejectSink::pop(record));
    CHECK(record.point == ProfilePoint::NavigatorProcessMeasurements);
    CHECK(record.start_tick == 1U);

    REQUIRE(RejectSink::pop(record));
    CHECK(record.point == ProfilePoint::KalmanObservationUpdate);
    CHECK(record.start_tick == 2U);

    CHECK(RejectSink::empty());
}

TEST_CASE("RingBufferProfileSink overwrite policy keeps newest records and flags gaps")
{
    using OverwriteSink =
        RingBufferProfileSink<FakeClock::Tick, 2U, containers::OverflowPolicy::OverwriteOldest>;

    OverwriteSink::reset();
    OverwriteSink::record({.point = ProfilePoint::NavigatorProcessMeasurements, .start_tick = 1U});
    OverwriteSink::record({.point = ProfilePoint::KalmanObservationUpdate, .start_tick = 2U});
    OverwriteSink::record({.point = ProfilePoint::PropagationUpdate, .start_tick = 3U});

    CHECK(OverwriteSink::size() == 2U);
    CHECK(OverwriteSink::dropped_count() == 1U);

    ProfileRecord<FakeClock::Tick> record{};
    REQUIRE(OverwriteSink::pop(record));
    CHECK(record.point == ProfilePoint::KalmanObservationUpdate);
    CHECK(record.start_tick == 2U);
    CHECK(record.flags == ProfileRecordFlags::None);

    REQUIRE(OverwriteSink::pop(record));
    CHECK(record.point == ProfilePoint::PropagationUpdate);
    CHECK(record.start_tick == 3U);
    CHECK(has_profile_record_flag(record.flags, ProfileRecordFlags::DroppedBefore));
}

TEST_CASE("KalmanFilter observation update emits the configured profile point")
{
    SteppingClock::reset({211U, 225U});
    FixedSink::reset();

    ProfiledFilter filter;
    ProfiledModel::O_t z;
    z.setZero();

    ProfiledModel::NoiseContext ctx;
    ctx.sigma_h = 1.0;
    ctx.sigma_v = 1.0;

    filter.observation_update<ProfiledModel>(z, 1.0, ctx, true);

    REQUIRE(FixedSink::count == 1U);
    const auto& record = FixedSink::first_record();
    CHECK(record.point == ProfilePoint::KalmanObservationUpdate);
    CHECK(record.start_tick == 211U);
    CHECK(record.elapsed_ticks == 14U);
}

TEST_CASE("Navigator process_measurements emits the configured profile point")
{
    SteppingClock::reset({310U, 333U});
    FixedSink::reset();

    ProfiledNavigator navigator;

    navigator.process_measurements();

    REQUIRE(FixedSink::count == 1U);
    const auto& record = FixedSink::first_record();
    CHECK(record.point == ProfilePoint::NavigatorProcessMeasurements);
    CHECK(record.start_tick == 310U);
    CHECK(record.elapsed_ticks == 23U);
}

TEST_CASE("Estimator algorithms default to NullProfiler")
{
    using DefaultFilter = navkit::core::estimation::KalmanFilter<ProfiledStateDef>;
    using DefaultNavigator =
        navkit::core::estimation::Navigator<DefaultFilter, std::tuple<ProfiledSensor>>;

    static_assert(std::is_same_v<DefaultFilter::Profiler_t, NullProfiler>);
    static_assert(std::is_same_v<DefaultNavigator::Profiler_t, NullProfiler>);
    CHECK(true);
}

} // namespace navkit::core::profiling::test
