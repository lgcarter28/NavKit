// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "apps/navkit_swil/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
#include "navkit/app_support/execution/MissionAdapter.hpp"
#include "navkit/swil/SwilMissionAdapterFactory.hpp"
#include "test_main.hpp"

#include <concepts>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace navkit::app_support::test
{

namespace
{

struct TestNavigator
{
    [[nodiscard]] bool update()
    {
        ++update_count;
        if (trace != nullptr) {
            trace->push_back("navigator_update");
        }
        return true;
    }

    std::size_t update_count{};
    std::vector<std::string>* trace{};
};

struct TestLogger
{};

using FactoryConfig = navkit::config::apps::navkit_swil::EcefInsGnssLcGyroAccelBiasDefaultConfig;
using FactoryNavKit = typename FactoryConfig::NavKit;
using FactoryLogger = RuntimeLogger<FactoryNavKit>;
using FactoryAdapter = MissionAdapter<typename FactoryNavKit::Navigator, FactoryLogger>;
using FactoryResult = decltype(navkit::swil::SwilMissionAdapterFactory::create<FactoryConfig>(
    std::declval<const nlohmann::json&>(),
    std::declval<const std::filesystem::path&>(),
    std::declval<const RunSettings&>()));

static_assert(std::same_as<FactoryResult, std::unique_ptr<FactoryAdapter>>);

class TraceMissionAdapter final : public MissionAdapter<TestNavigator, TestLogger>
{
public:
    explicit TraceMissionAdapter(std::vector<std::string>& trace)
        : m_trace{trace}
    {}

    [[nodiscard]] core::Timestamp t_start() const override
    {
        return {};
    }

    [[nodiscard]] bool is_complete() const override
    {
        return m_state == State::Finalized;
    }

    [[nodiscard]] bool initialize(TestNavigator& navigator, TestLogger&) override
    {
        if (m_state != State::Constructed) {
            return fail("initialize called out of order");
        }
        m_trace.push_back("initialize");
        navigator.trace = &m_trace;
        m_state = State::Initialized;
        return true;
    }

    [[nodiscard]] bool prepare_before_deadline(const core::Timestamp& requested_t,
                                               core::Timestamp& prepared_t) override
    {
        if (m_state != State::Initialized && m_state != State::FeedbackApplied) {
            return fail("prepare_before_deadline called out of order");
        }
        m_prepared_t = requested_t;
        prepared_t = requested_t;
        m_trace.push_back("prepare");
        m_state = State::Prepared;
        return true;
    }

    [[nodiscard]] bool
    publish_at_deadline(const core::Timestamp& t, TestNavigator& navigator, TestLogger&) override
    {
        if (m_state != State::Prepared || t != m_prepared_t) {
            return fail("publish_at_deadline called without matching prepare");
        }
        m_trace.push_back("publish");
        m_update_count_at_publish = navigator.update_count;
        m_state = State::Published;
        return true;
    }

    [[nodiscard]] bool after_navigation_update(const core::Timestamp& t,
                                               const TestNavigator& navigator) override
    {
        if (m_state != State::Published || t != m_prepared_t ||
            navigator.update_count != m_update_count_at_publish + 1U) {
            return fail("after_navigation_update called before matching Navigator update");
        }
        m_trace.push_back("after_navigation_update");
        m_state = State::FeedbackApplied;
        return true;
    }

    [[nodiscard]] bool active_mission_phase_id(std::string& phase_id) const override
    {
        if (m_state == State::Constructed) {
            return false;
        }
        phase_id = "startup";
        return true;
    }

    [[nodiscard]] bool finalize() override
    {
        if (m_state == State::Constructed || m_state == State::Finalized) {
            return fail("finalize called outside an initialized lifecycle");
        }
        m_trace.push_back("finalize");
        m_state = State::Finalized;
        return true;
    }

    [[nodiscard]] std::string_view last_error() const override
    {
        return m_last_error;
    }

private:
    enum class State
    {
        Constructed,
        Initialized,
        Prepared,
        Published,
        FeedbackApplied,
        Finalized,
    };

    [[nodiscard]] bool fail(const std::string_view error)
    {
        m_last_error = error;
        return false;
    }

    std::vector<std::string>& m_trace;
    State m_state{State::Constructed};
    core::Timestamp m_prepared_t{};
    std::size_t m_update_count_at_publish{};
    std::string m_last_error{};
};

} // namespace

TEST_CASE("SWIL mission adapter factory fixes target identity and erases concrete type")
{
    CHECK(navkit::swil::SwilMissionAdapterFactory::target_type == ExecutionTargetType::Swil);
    CHECK(navkit::swil::SwilMissionAdapterFactory::application_name == "navkit_swil");
}

TEST_CASE("Mission adapter lifecycle keeps preparation publication and feedback ordered")
{
    std::vector<std::string> trace{};
    TraceMissionAdapter adapter{trace};
    TestNavigator navigator{};
    TestLogger logger{};
    core::Timestamp t{};
    t.s = 4U;

    REQUIRE(adapter.initialize(navigator, logger));
    core::Timestamp prepared_t{};
    REQUIRE(adapter.prepare_before_deadline(t, prepared_t));
    CHECK(prepared_t == t);
    trace.push_back("wait_until");
    REQUIRE(adapter.publish_at_deadline(t, navigator, logger));
    REQUIRE(navigator.update());
    REQUIRE(adapter.after_navigation_update(t, navigator));
    REQUIRE(adapter.finalize());

    const std::vector<std::string> expected{"initialize",
                                            "prepare",
                                            "wait_until",
                                            "publish",
                                            "navigator_update",
                                            "after_navigation_update",
                                            "finalize"};
    REQUIRE(trace.size() == expected.size());
    for (std::size_t index = 0U; index < expected.size(); ++index) {
        CHECK(trace.at(index) == expected.at(index));
    }
}

TEST_CASE("Mission adapter lifecycle rejects out-of-order work and permits partial cleanup")
{
    std::vector<std::string> trace{};
    TraceMissionAdapter adapter{trace};
    TestNavigator navigator{};
    TestLogger logger{};
    core::Timestamp t{};
    t.s = 4U;

    CHECK_FALSE(adapter.publish_at_deadline(t, navigator, logger));
    CHECK(std::string{adapter.last_error()} ==
          "publish_at_deadline called without matching prepare");
    REQUIRE(adapter.initialize(navigator, logger));
    core::Timestamp prepared_t{};
    REQUIRE(adapter.prepare_before_deadline(t, prepared_t));
    REQUIRE(adapter.publish_at_deadline(t, navigator, logger));
    CHECK_FALSE(adapter.after_navigation_update(t, navigator));
    CHECK(std::string{adapter.last_error()} ==
          "after_navigation_update called before matching Navigator update");
    CHECK(adapter.finalize());
}

} // namespace navkit::app_support::test
