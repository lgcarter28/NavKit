// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/app_support/time/ClockFactory.hpp"
#include "navkit/core/time/Time.hpp"
#include "test_main.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>

namespace navkit::core::test
{

TEST_CASE("Timestamp elapsed time uses integer nanosecond arithmetic")
{
    const Timestamp t_start{.s = 7U, .ns = 900'000'000U};
    const Timestamp t_end{.s = 8U, .ns = 100'000'000U};
    Duration elapsed{};

    REQUIRE(elapsed_time(t_end, t_start, elapsed));
    CHECK(elapsed == Duration{.s = 0U, .ns = 200'000'000U});
    CHECK(duration_seconds(elapsed) == doctest::Approx(0.2));
}

TEST_CASE("Timestamp elapsed time rejects mismatched scales and reverse ordering")
{
    const Timestamp t_monotonic{.scale = TimeScale::Monotonic, .s = 2U};
    const Timestamp t_utc{.scale = TimeScale::Utc, .s = 3U};
    Duration elapsed{.s = 99U, .ns = 99U};

    CHECK_FALSE(elapsed_time(t_utc, t_monotonic, elapsed));
    CHECK(elapsed == Duration{});
    CHECK_FALSE(elapsed_time(t_monotonic, Timestamp{.s = 3U}, elapsed));
    CHECK(elapsed == Duration{});

    const Timestamp t_invalid_ns{.ns = nanoseconds_per_second};
    CHECK_FALSE(elapsed_time(t_invalid_ns, Timestamp{}, elapsed));
    CHECK(elapsed == Duration{});

    const Timestamp t_unsupported_version{.version = timestamp_version + 1U};
    CHECK_FALSE(elapsed_time(t_unsupported_version, Timestamp{}, elapsed));
    CHECK(elapsed == Duration{});
}

TEST_CASE("Rational-rate timestamps preserve exact long-term phase")
{
    constexpr RationalRate rate{.samples = 600U, .s = 1U};
    Timestamp t_one_second{};
    Timestamp t_one_hour{};

    REQUIRE(timestamp_at_sample_index(Timestamp{}, rate, 600U, t_one_second));
    REQUIRE(timestamp_at_sample_index(Timestamp{}, rate, 2'160'000U, t_one_hour));
    CHECK(t_one_second == Timestamp{.s = 1U});
    CHECK(t_one_hour == Timestamp{.s = 3'600U});
}

TEST_CASE("Rational-rate whole-second denominator uses public Seconds storage")
{
    constexpr Seconds extended_denominator =
        static_cast<Seconds>(std::numeric_limits<std::uint32_t>::max()) + 1U;
    constexpr RationalRate rate{.samples = 1U, .s = extended_denominator};
    Timestamp timestamp{};

    REQUIRE(timestamp_at_sample_index(Timestamp{}, rate, 1U, timestamp));
    CHECK(timestamp == Timestamp{.s = extended_denominator});
}

TEST_CASE("Rational-rate timestamp rejects overflowing fractional-cycle multiplication")
{
    constexpr RationalRate rate{
        .samples = 3U,
        .s = std::numeric_limits<Seconds>::max(),
    };
    Timestamp timestamp{};

    CHECK_FALSE(timestamp_at_sample_index(Timestamp{}, rate, 2U, timestamp));
    CHECK(timestamp == Timestamp{});
}

TEST_CASE("Rational schedule requires explicit initialization and resets only on request")
{
    constexpr RationalRate rate{.samples = 2U, .s = 1U};
    RationalSchedule schedule{};

    CHECK(RationalSchedule::is_valid(Timestamp{}, rate));
    CHECK_FALSE(schedule.due(Timestamp{}));
    REQUIRE(schedule.initialize(Timestamp{}, rate));
    CHECK(schedule.is_initialized());
    CHECK_FALSE(schedule.initialize(Timestamp{}, rate));
    CHECK(schedule.due(Timestamp{}));

    REQUIRE(schedule.reset(Timestamp{.s = 1U}, rate));
    CHECK_FALSE(schedule.due(Timestamp{}));
    CHECK(schedule.due(Timestamp{.s = 1U}));
}

TEST_CASE("Rational schedule remains phase-stable across a faster producer cadence")
{
    constexpr RationalRate producer_rate{.samples = 1'000U, .s = 1U};
    constexpr RationalRate consumer_rate{.samples = 600U, .s = 1U};
    RationalSchedule schedule{};
    REQUIRE(schedule.initialize(Timestamp{}, consumer_rate));

    std::size_t due_count{};
    for (std::uint64_t index = 0U; index <= 1'000U; ++index) {
        Timestamp t_producer{};
        REQUIRE(timestamp_at_sample_index(Timestamp{}, producer_rate, index, t_producer));
        if (schedule.due(t_producer)) {
            ++due_count;
        }
    }

    CHECK(due_count == 601U);
}

TEST_CASE("Rational timeline produces exact planned timestamps after its epoch")
{
    constexpr RationalRate rate{.samples = 600U, .s = 1U};
    const Timestamp t_epoch{.s = 7U, .ns = 500'000'000U};
    RationalTimeline timeline{};
    Timestamp first{};
    Timestamp second{};

    REQUIRE(timeline.initialize(t_epoch, rate));
    REQUIRE(timeline.next(first));
    REQUIRE(timeline.next(second));

    CHECK(timestamp_seconds(first) == doctest::Approx(7.5 + (1.0 / 600.0)));
    CHECK(timestamp_seconds(second) == doctest::Approx(7.5 + (2.0 / 600.0)));
    CHECK(timestamp_less(t_epoch, first));
    CHECK(timestamp_less(first, second));
}

TEST_CASE("Rational-rate integer-multiple validation protects planned application cadence")
{
    CHECK(rational_rate_is_integer_multiple(RationalRate{.samples = 3'000U, .s = 1U},
                                            RationalRate{.samples = 600U, .s = 1U}));
    CHECK(rational_rate_is_integer_multiple(RationalRate{.samples = 1'000U, .s = 1U},
                                            RationalRate{.samples = 1U, .s = 1U}));
    CHECK_FALSE(rational_rate_is_integer_multiple(RationalRate{.samples = 1'000U, .s = 1U},
                                                  RationalRate{.samples = 600U, .s = 1U}));
}

TEST_CASE("Simulation clock adopts planned time without waiting")
{
    std::unique_ptr<navkit::app_support::Clock> clock =
        navkit::app_support::clock_from_mode(navkit::app_support::ClockMode::Simulated);
    const Timestamp t_epoch{.s = 3U};
    const Timestamp t_next{.s = 3U, .ns = 500'000'000U};

    REQUIRE(clock);
    REQUIRE(clock->initialize(t_epoch));
    CHECK(clock->now() == t_epoch);
    REQUIRE(clock->wait_until(t_next));
    CHECK(clock->now() == t_next);
    CHECK_FALSE(clock->wait_until(t_epoch));
}

TEST_CASE("Realtime clock shares the planned-time boundary without accepting reverse time")
{
    std::unique_ptr<navkit::app_support::Clock> clock =
        navkit::app_support::clock_from_mode(navkit::app_support::ClockMode::Realtime);
    const Timestamp t_epoch{.s = 3U};
    const Timestamp t_other_scale{.scale = TimeScale::Gps, .s = 3U};

    REQUIRE(clock);
    REQUIRE(clock->initialize(t_epoch));
    REQUIRE(clock->wait_until(t_epoch));
    CHECK(clock->now() == t_epoch);
    CHECK_FALSE(clock->wait_until(t_other_scale));
}

TEST_CASE("Runtime rate parser canonicalizes rate and period JSON forms")
{
    const nlohmann::json rate_config{{"rate_hz", 600.0}};
    const nlohmann::json period_config{{"dt_s", 0.001}};
    const nlohmann::json fractional_config{{"rate_hz", 59.94}};
    const nlohmann::json extended_period_config{{"dt_s", 4'294'967'297.0}};

    CHECK(navkit::app_support::rational_rate_from_required_runtime_rate(rate_config, "imu") ==
          RationalRate{.samples = 600U, .s = 1U});
    CHECK(navkit::app_support::rational_rate_from_required_runtime_rate(period_config, "imu") ==
          RationalRate{.samples = 1'000U, .s = 1U});
    CHECK(navkit::app_support::rational_rate_from_required_runtime_rate(fractional_config, "imu") ==
          RationalRate{.samples = 2'997U, .s = 50U});
    CHECK(navkit::app_support::rational_rate_from_required_runtime_rate(
              extended_period_config, "imu") == RationalRate{.samples = 1U, .s = 4'294'967'297U});
}

} // namespace navkit::core::test
