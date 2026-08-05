// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"
#include "navkit/sim/guidance/GuidanceStateMachine.hpp"
#include "test_main.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

namespace navkit::sim::test
{

namespace
{

[[nodiscard]] GuidanceStateDefinition
direct_force_state(const std::string& id, const core::Vec3& force_b_mps2, const bool terminal)
{
    GuidanceStateDefinition state{};
    state.id = id;
    state.translation.reference = std::make_unique<CurrentStateGuidanceReference>();
    state.translation.acceleration.push_back(
        std::make_unique<BodySpecificForceGuidanceAcceleration>(
            BodySpecificForceGuidanceAccelerationConfig{.specific_force_ib_b_mps2 = force_b_mps2}));
    state.bank = std::make_unique<ZeroBankGuidancePolicy>();
    state.terminal = terminal;
    return state;
}

[[nodiscard]] TrajectoryEnvironment identity_environment()
{
    TrajectoryEnvironment environment{};
    environment.p_e_m = core::Vec3{6378137.0, 0.0, 0.0};
    environment.gravity_i_mps2 = core::Vec3{0.0, 0.0, -9.0};
    return environment;
}

} // namespace

TEST_CASE("Guidance state machine validates and executes deterministic elapsed transitions")
{
    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = "boost";
    GuidanceStateDefinition boost = direct_force_state("boost", core::Vec3{5.0, 0.0, 0.0}, false);
    boost.transitions.push_back(
        GuidanceElapsedTransition{.to = "coast", .elapsed_in_state_s = 1.0, .priority = 0U});
    GuidanceStateDefinition coast = direct_force_state("coast", core::Vec3::Zero(), true);
    coast.on_entry_guidance_command_filter.enabled = true;
    coast.on_entry_guidance_command_filter.specific_force_time_constant_b_s =
        core::Vec3::Constant(2.0);
    coast.on_entry_guidance_command_filter.bank_time_constant_s = 2.0;
    coast.on_entry_guidance_command_filter.duration_s = 0.5;
    definition.states.push_back(std::move(boost));
    definition.states.push_back(std::move(coast));

    std::string diagnostic{};
    REQUIRE(guidance_state_machine_definition_is_valid(definition, diagnostic));
    GuidanceStateMachine model{std::move(definition)};
    TrajectoryControlState state{};
    TrajectoryEnvironment environment = identity_environment();
    REQUIRE(model.initialize(state, environment));

    GuidanceOutput boost_output{};
    state.t = core::Timestamp{.s = 0U, .ns = 100'000'000U};
    REQUIRE(model.advance(state, environment, 0.1, boost_output));
    CHECK(boost_output.execution.state_index == 0U);
    CHECK(boost_output.execution.state_entered);
    CHECK(boost_output.diagnostics.a_cmd_i_mps2.isApprox(core::Vec3{5.0, 0.0, -9.0}));

    GuidanceOutput coast_output{};
    state.t = core::Timestamp{.s = 1U};
    REQUIRE(model.advance(state, environment, 0.1, coast_output));
    CHECK(coast_output.execution.state_index == 1U);
    CHECK(coast_output.execution.state_entered);
    CHECK(coast_output.execution.filter_on_entry.enabled);
    CHECK(coast_output.diagnostics.a_cmd_i_mps2.isApprox(environment.gravity_i_mps2));
    CHECK(model.active_state_is_terminal());
}

TEST_CASE("Guidance state machine rejects orphan states and mixed direct-force pipelines")
{
    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = "bad";
    GuidanceStateDefinition bad = direct_force_state("bad", core::Vec3::Zero(), false);
    bad.translation.acceleration.push_back(std::make_unique<VelocityTrackingGuidanceAcceleration>(
        VelocityTrackingGuidanceAccelerationConfig{.gain_n_1ps = core::Vec3::Ones()}));
    bad.transitions.push_back(
        GuidanceElapsedTransition{.to = "done", .elapsed_in_state_s = 1.0, .priority = 0U});
    definition.states.push_back(std::move(bad));
    definition.states.push_back(direct_force_state("done", core::Vec3::Zero(), true));
    definition.states.push_back(direct_force_state("orphan", core::Vec3::Zero(), true));

    std::string diagnostic{};
    CHECK_FALSE(guidance_state_machine_definition_is_valid(definition, diagnostic));
    CHECK_FALSE(diagnostic.empty());
}

TEST_CASE("Guidance state machine rejects cycles unless explicitly allowed")
{
    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = "first";
    GuidanceStateDefinition first = direct_force_state("first", core::Vec3::Zero(), false);
    first.transitions.push_back(
        GuidanceElapsedTransition{.to = "second", .elapsed_in_state_s = 1.0, .priority = 0U});
    GuidanceStateDefinition second = direct_force_state("second", core::Vec3::Zero(), false);
    second.transitions.push_back(
        GuidanceElapsedTransition{.to = "first", .elapsed_in_state_s = 1.0, .priority = 0U});
    definition.states.push_back(std::move(first));
    definition.states.push_back(std::move(second));

    std::string diagnostic{};
    CHECK_FALSE(guidance_state_machine_definition_is_valid(definition, diagnostic));
}

TEST_CASE("Guidance state machine selects the lowest-priority eligible transition")
{
    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = "initial";
    GuidanceStateDefinition initial = direct_force_state("initial", core::Vec3::Zero(), false);
    initial.transitions.push_back(GuidanceElapsedTransition{
        .to = "higher_priority_number", .elapsed_in_state_s = 1.0, .priority = 8U});
    initial.transitions.push_back(GuidanceElapsedTransition{
        .to = "lower_priority_number", .elapsed_in_state_s = 1.0, .priority = 2U});
    definition.states.push_back(std::move(initial));
    definition.states.push_back(
        direct_force_state("higher_priority_number", core::Vec3::Zero(), true));
    definition.states.push_back(
        direct_force_state("lower_priority_number", core::Vec3::Zero(), true));

    std::string diagnostic{};
    REQUIRE(guidance_state_machine_definition_is_valid(definition, diagnostic));
    GuidanceStateMachine model{std::move(definition)};
    TrajectoryControlState state{};
    const TrajectoryEnvironment environment = identity_environment();
    REQUIRE(model.initialize(state, environment));

    GuidanceOutput output{};
    state.t = core::Timestamp{.s = 1U};
    REQUIRE(model.advance(state, environment, 0.1, output));
    CHECK(model.active_state_id() == "lower_priority_number");
    CHECK(output.execution.state_index == 2U);
    CHECK(output.execution.state_entered);
}

TEST_CASE("Guidance state machine permits a cycle with a reachable terminal state")
{
    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = "first";
    definition.cycle_policy = GuidanceCyclePolicy::Allow;
    GuidanceStateDefinition first = direct_force_state("first", core::Vec3::Zero(), false);
    first.transitions.push_back(
        GuidanceElapsedTransition{.to = "second", .elapsed_in_state_s = 1.0, .priority = 0U});
    GuidanceStateDefinition second = direct_force_state("second", core::Vec3::Zero(), false);
    second.transitions.push_back(
        GuidanceElapsedTransition{.to = "first", .elapsed_in_state_s = 2.0, .priority = 1U});
    second.transitions.push_back(
        GuidanceElapsedTransition{.to = "done", .elapsed_in_state_s = 1.0, .priority = 0U});
    definition.states.push_back(std::move(first));
    definition.states.push_back(std::move(second));
    definition.states.push_back(direct_force_state("done", core::Vec3::Zero(), true));

    std::string diagnostic{};
    REQUIRE(guidance_state_machine_definition_is_valid(definition, diagnostic));
    CHECK(diagnostic.empty());

    GuidanceStateMachine model{std::move(definition)};
    TrajectoryControlState state{};
    const TrajectoryEnvironment environment = identity_environment();
    REQUIRE(model.initialize(state, environment));

    GuidanceOutput output{};
    state.t = core::Timestamp{.s = 1U};
    REQUIRE(model.advance(state, environment, 0.1, output));
    CHECK(model.active_state_id() == "second");
    state.t = core::Timestamp{.s = 2U};
    REQUIRE(model.advance(state, environment, 0.1, output));
    CHECK(model.active_state_id() == "done");
    CHECK(model.active_state_is_terminal());
}

TEST_CASE("Guidance command filter preserves state through an entry override and restores nominal "
          "constants")
{
    GuidanceCommandFilterConfig initial_config{};
    initial_config.specific_force_time_constant_b_s = core::Vec3::Constant(1.0);
    initial_config.bank_time_constant_s = 1.0;
    GuidanceCommandFilter filter{};
    REQUIRE(filter.initialize(core::Vec3::Zero(), 0.0, initial_config));

    GuidanceCommandFilterStateEntryConfig no_entry{};
    GuidanceCommandFilterOutput before_transition{};
    REQUIRE(filter.advance(
        core::Vec3::Constant(1.0), 1.0, initial_config, no_entry, false, 1.0, before_transition));
    const core::Scalar_t before_transition_expected = 1.0 - std::exp(-1.0);
    CHECK(before_transition.specific_force_filtered_ib_b_mps2.x() ==
          doctest::Approx(before_transition_expected));

    GuidanceCommandFilterConfig nominal_after_transition{};
    nominal_after_transition.specific_force_time_constant_b_s = core::Vec3::Constant(0.5);
    nominal_after_transition.bank_time_constant_s = 0.5;
    GuidanceCommandFilterStateEntryConfig entry_override{};
    entry_override.enabled = true;
    entry_override.specific_force_time_constant_b_s = core::Vec3::Constant(2.0);
    entry_override.bank_time_constant_s = 2.0;
    entry_override.duration_s = 0.5;

    GuidanceCommandFilterOutput first_entry_step{};
    REQUIRE(filter.advance(core::Vec3::Constant(2.0),
                           2.0,
                           nominal_after_transition,
                           entry_override,
                           true,
                           0.25,
                           first_entry_step));
    const core::Scalar_t first_entry_expected =
        before_transition_expected +
        (2.0 - before_transition_expected) * (1.0 - std::exp(-0.25 / 2.0));
    CHECK(first_entry_step.specific_force_filtered_ib_b_mps2.x() ==
          doctest::Approx(first_entry_expected));
    CHECK(first_entry_step.bank_filtered_n_rad == doctest::Approx(first_entry_expected));
    CHECK(filter.active_config().specific_force_time_constant_b_s.x() == doctest::Approx(2.0));

    GuidanceCommandFilterOutput final_entry_step{};
    REQUIRE(filter.advance(core::Vec3::Constant(2.0),
                           2.0,
                           nominal_after_transition,
                           entry_override,
                           false,
                           0.25,
                           final_entry_step));
    const core::Scalar_t final_entry_expected =
        first_entry_expected + (2.0 - first_entry_expected) * (1.0 - std::exp(-0.25 / 2.0));
    CHECK(final_entry_step.specific_force_filtered_ib_b_mps2.x() ==
          doctest::Approx(final_entry_expected));
    CHECK(filter.active_config().specific_force_time_constant_b_s.x() == doctest::Approx(0.5));

    GuidanceCommandFilterOutput nominal_step{};
    REQUIRE(filter.advance(core::Vec3::Constant(2.0),
                           2.0,
                           nominal_after_transition,
                           entry_override,
                           false,
                           0.25,
                           nominal_step));
    const core::Scalar_t nominal_expected =
        final_entry_expected + (2.0 - final_entry_expected) * (1.0 - std::exp(-0.25 / 0.5));
    CHECK(nominal_step.specific_force_filtered_ib_b_mps2.x() == doctest::Approx(nominal_expected));
    CHECK(nominal_step.bank_filtered_n_rad == doctest::Approx(nominal_expected));
}

} // namespace navkit::sim::test
