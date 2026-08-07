// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/guidance/GuidanceStateMachine.hpp"

#include "navkit/core/time/Duration.hpp"
#include "navkit/sim/guidance/GuidanceAlgorithms.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace navkit::sim
{

namespace
{

[[nodiscard]] bool elapsed_seconds(const core::Timestamp& first,
                                   const core::Timestamp& second,
                                   core::Time_t& elapsed_s)
{
    core::Duration elapsed{};
    if (!core::elapsed_time(second, first, elapsed)) {
        return false;
    }
    elapsed_s = core::duration_seconds(elapsed);
    return std::isfinite(elapsed_s) && elapsed_s >= 0.0;
}

[[nodiscard]] bool find_state_index(const GuidanceStateMachineDefinition& definition,
                                    const std::string& id,
                                    std::size_t& state_index)
{
    for (std::size_t index = 0U; index < definition.states.size(); ++index) {
        if (definition.states.at(index).id == id) {
            state_index = index;
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool state_pipeline_is_valid(const GuidanceStateDefinition& state,
                                           std::string& diagnostic)
{
    if (!guidance_command_filter_config_is_valid(state.guidance_command_filter) ||
        !guidance_command_filter_state_entry_config_is_valid(
            state.on_entry_guidance_command_filter)) {
        diagnostic = "state '" + state.id + "' has an invalid Guidance command-filter config";
        return false;
    }

    if (!state.guidance_enabled) {
        if ((state.translation.reference && !state.translation.reference->config_is_valid()) ||
            (state.bank && !state.bank->config_is_valid())) {
            diagnostic = "disabled Guidance state '" + state.id +
                         "' contains an invalid optional Guidance block";
            return false;
        }
        for (const std::unique_ptr<GuidanceReferenceModifier>& modifier :
             state.translation.reference_modifiers) {
            if (!modifier || !modifier->config_is_valid()) {
                diagnostic = "disabled Guidance state '" + state.id +
                             "' contains an invalid optional reference modifier";
                return false;
            }
        }
        for (const std::unique_ptr<GuidanceAccelerationModel>& acceleration :
             state.translation.acceleration) {
            if (!acceleration || !acceleration->config_is_valid()) {
                diagnostic = "disabled Guidance state '" + state.id +
                             "' contains an invalid optional acceleration block";
                return false;
            }
        }
        return true;
    }

    if (!state.bank || !state.bank->config_is_valid()) {
        diagnostic = "active Guidance state '" + state.id + "' requires one valid bank policy";
        return false;
    }

    std::size_t direct_specific_force_count = 0U;
    for (const std::unique_ptr<GuidanceReferenceModifier>& modifier :
         state.translation.reference_modifiers) {
        if (!modifier || !modifier->config_is_valid()) {
            diagnostic = "state '" + state.id + "' has an invalid reference modifier";
            return false;
        }
    }
    for (const std::unique_ptr<GuidanceAccelerationModel>& acceleration :
         state.translation.acceleration) {
        if (!acceleration || !acceleration->config_is_valid()) {
            diagnostic = "state '" + state.id + "' has an invalid acceleration block";
            return false;
        }
        if (acceleration->role() == GuidanceAccelerationRole::DirectBodySpecificForce) {
            ++direct_specific_force_count;
        }
    }

    if (direct_specific_force_count > 0U) {
        const bool direct_pipeline_valid =
            direct_specific_force_count == 1U && state.translation.acceleration.size() == 1U &&
            state.translation.reference && state.translation.reference->config_is_valid() &&
            state.translation.reference->role() == GuidanceReferenceRole::CurrentState &&
            state.translation.reference_modifiers.empty();
        if (!direct_pipeline_valid) {
            diagnostic = "state '" + state.id +
                         "' must use direct body specific force as its sole translation block";
            return false;
        }
        return true;
    }

    if (!state.translation.reference || !state.translation.reference->config_is_valid()) {
        diagnostic = "state '" + state.id + "' requires one valid primary reference";
        return false;
    }
    return true;
}

[[nodiscard]] bool
graph_has_cycle_from(const GuidanceStateMachineDefinition& definition,
                     const std::unordered_map<std::string, std::size_t>& state_index_by_id,
                     const std::size_t state_index,
                     std::vector<unsigned char>& visit_state)
{
    visit_state.at(state_index) = 1U;
    const GuidanceStateDefinition& state = definition.states.at(state_index);
    for (const GuidanceElapsedTransition& transition : state.transitions) {
        const std::size_t target_index = state_index_by_id.at(transition.to);
        if (visit_state.at(target_index) == 1U) {
            return true;
        }
        if (visit_state.at(target_index) == 0U &&
            graph_has_cycle_from(definition, state_index_by_id, target_index, visit_state)) {
            return true;
        }
    }
    visit_state.at(state_index) = 2U;
    return false;
}

} // namespace

bool guidance_state_machine_definition_is_valid(const GuidanceStateMachineDefinition& definition,
                                                std::string& diagnostic)
{
    diagnostic.clear();
    if (definition.initial_state_id.empty() || definition.states.empty()) {
        diagnostic = "Guidance state machine requires an initial_state_id and at least one state";
        return false;
    }

    std::unordered_map<std::string, std::size_t> state_index_by_id{};
    state_index_by_id.reserve(definition.states.size());
    for (std::size_t index = 0U; index < definition.states.size(); ++index) {
        const GuidanceStateDefinition& state = definition.states.at(index);
        if (state.id.empty() || !state_index_by_id.emplace(state.id, index).second) {
            diagnostic = "Guidance state IDs must be nonempty and unique";
            return false;
        }
        if (!state_pipeline_is_valid(state, diagnostic)) {
            return false;
        }
        if (state.terminal && !state.transitions.empty()) {
            diagnostic = "terminal state '" + state.id + "' must not define transitions";
            return false;
        }
        if (!state.terminal && state.transitions.empty()) {
            diagnostic = "nonterminal state '" + state.id + "' requires a transition";
            return false;
        }
        std::unordered_set<std::size_t> priorities{};
        for (const GuidanceElapsedTransition& transition : state.transitions) {
            if (transition.to.empty() || !std::isfinite(transition.elapsed_in_state_s) ||
                transition.elapsed_in_state_s <= 0.0 ||
                !priorities.emplace(transition.priority).second) {
                diagnostic = "state '" + state.id +
                             "' has an invalid or nondeterministic elapsed-state transition";
                return false;
            }
        }
    }

    const std::unordered_map<std::string, std::size_t>::const_iterator initial_iterator =
        state_index_by_id.find(definition.initial_state_id);
    if (initial_iterator == state_index_by_id.end()) {
        diagnostic = "initial_state_id does not name a configured state";
        return false;
    }
    for (const GuidanceStateDefinition& state : definition.states) {
        for (const GuidanceElapsedTransition& transition : state.transitions) {
            if (!state_index_by_id.contains(transition.to)) {
                diagnostic =
                    "state '" + state.id + "' transitions to unknown state '" + transition.to + "'";
                return false;
            }
        }
    }

    std::vector<bool> reachable(definition.states.size(), false);
    std::vector<std::size_t> pending{initial_iterator->second};
    reachable.at(initial_iterator->second) = true;
    bool reachable_terminal = false;
    while (!pending.empty()) {
        const std::size_t state_index = pending.back();
        pending.pop_back();
        const GuidanceStateDefinition& state = definition.states.at(state_index);
        reachable_terminal = reachable_terminal || state.terminal;
        for (const GuidanceElapsedTransition& transition : state.transitions) {
            const std::size_t target_index = state_index_by_id.at(transition.to);
            if (!reachable.at(target_index)) {
                reachable.at(target_index) = true;
                pending.push_back(target_index);
            }
        }
    }
    if (std::ranges::find(reachable, false) != reachable.end()) {
        diagnostic = "Guidance state graph contains an unreachable/orphan state";
        return false;
    }
    if (!reachable_terminal) {
        diagnostic = "Guidance state graph requires at least one reachable terminal state";
        return false;
    }

    if (definition.cycle_policy == GuidanceCyclePolicy::Reject) {
        std::vector<unsigned char> visit_state(definition.states.size(), 0U);
        if (graph_has_cycle_from(
                definition, state_index_by_id, initial_iterator->second, visit_state)) {
            diagnostic = "Guidance state graph contains a cycle while cycle_policy is reject";
            return false;
        }
    }
    return true;
}

GuidanceStateMachine::GuidanceStateMachine(GuidanceStateMachineDefinition definition)
    : m_definition(std::move(definition))
{}

bool GuidanceStateMachine::initialize(const TrajectoryControlState& initial_state,
                                      const TrajectoryEnvironment& environment)
{
    std::string diagnostic{};
    std::size_t initial_state_index{};
    if (!core::timestamp_is_valid(initial_state.t) ||
        !guidance_state_machine_definition_is_valid(m_definition, diagnostic) ||
        !find_state_index(m_definition, m_definition.initial_state_id, initial_state_index)) {
        return false;
    }

    for (GuidanceStateDefinition& state : m_definition.states) {
        if (state.translation.reference &&
            !state.translation.reference->initialize(initial_state, environment)) {
            return false;
        }
        for (std::unique_ptr<GuidanceReferenceModifier>& modifier :
             state.translation.reference_modifiers) {
            if (!modifier->initialize(initial_state, environment)) {
                return false;
            }
        }
        for (std::unique_ptr<GuidanceAccelerationModel>& acceleration :
             state.translation.acceleration) {
            if (!acceleration->initialize(initial_state, environment)) {
                return false;
            }
        }
    }
    if (!enter_state(initial_state_index, initial_state, environment)) {
        return false;
    }
    m_initialized = true;
    m_first_output_pending = true;
    return true;
}

bool GuidanceStateMachine::advance(const TrajectoryControlState& state,
                                   const TrajectoryEnvironment& environment,
                                   const core::Time_t dt_s,
                                   GuidanceOutput& output)
{
    output = {};
    bool state_entered = m_first_output_pending;
    if (!m_initialized || !std::isfinite(dt_s) || dt_s <= 0.0 ||
        !evaluate_transitions(state, environment, state_entered)) {
        return false;
    }
    core::Time_t elapsed_in_state_s{};
    if (!elapsed_seconds(m_state_entry_time, state.t, elapsed_in_state_s) ||
        !evaluate_active_state(
            state, environment, elapsed_in_state_s, dt_s, state_entered, output)) {
        return false;
    }
    m_first_output_pending = false;
    return true;
}

std::size_t GuidanceStateMachine::active_state_index() const
{
    return m_active_state_index;
}

const std::string& GuidanceStateMachine::active_state_id() const
{
    // Local static value constants use snake_case by project convention.
    // NOLINTNEXTLINE(readability-identifier-naming)
    static const std::string empty_state_id{};
    if (!m_initialized || m_active_state_index >= m_definition.states.size()) {
        return empty_state_id;
    }
    return m_definition.states.at(m_active_state_index).id;
}

bool GuidanceStateMachine::active_state_is_terminal() const
{
    return m_initialized && m_active_state_index < m_definition.states.size() &&
           m_definition.states.at(m_active_state_index).terminal;
}

bool GuidanceStateMachine::enter_state(const std::size_t state_index,
                                       const TrajectoryControlState& state,
                                       const TrajectoryEnvironment& environment)
{
    if (state_index >= m_definition.states.size()) {
        return false;
    }
    GuidanceStateDefinition& entered_state = m_definition.states.at(state_index);
    if (entered_state.translation.reference &&
        !entered_state.translation.reference->enter(state, environment)) {
        return false;
    }
    for (std::unique_ptr<GuidanceReferenceModifier>& modifier :
         entered_state.translation.reference_modifiers) {
        if (!modifier->enter(state, environment)) {
            return false;
        }
    }
    for (std::unique_ptr<GuidanceAccelerationModel>& acceleration :
         entered_state.translation.acceleration) {
        if (!acceleration->enter(state, environment)) {
            return false;
        }
    }
    m_active_state_index = state_index;
    m_state_entry_time = state.t;
    return true;
}

bool GuidanceStateMachine::evaluate_transitions(const TrajectoryControlState& state,
                                                const TrajectoryEnvironment& environment,
                                                bool& state_entered)
{
    if (m_active_state_index >= m_definition.states.size()) {
        return false;
    }
    const GuidanceStateDefinition& active_state = m_definition.states.at(m_active_state_index);
    if (active_state.terminal) {
        return true;
    }
    core::Time_t elapsed_in_state_s{};
    if (!elapsed_seconds(m_state_entry_time, state.t, elapsed_in_state_s)) {
        return false;
    }

    const GuidanceElapsedTransition* selected_transition = nullptr;
    for (const GuidanceElapsedTransition& transition : active_state.transitions) {
        if (elapsed_in_state_s >= transition.elapsed_in_state_s &&
            (selected_transition == nullptr ||
             transition.priority < selected_transition->priority)) {
            selected_transition = &transition;
        }
    }
    if (selected_transition == nullptr) {
        return true;
    }

    std::size_t target_state_index{};
    if (!find_state_index(m_definition, selected_transition->to, target_state_index) ||
        !enter_state(target_state_index, state, environment)) {
        return false;
    }
    state_entered = true;
    return true;
}

bool GuidanceStateMachine::evaluate_active_state(const TrajectoryControlState& state,
                                                 const TrajectoryEnvironment& environment,
                                                 const core::Time_t elapsed_in_state_s,
                                                 const core::Time_t dt_s,
                                                 const bool state_entered,
                                                 GuidanceOutput& output)
{
    if (m_active_state_index >= m_definition.states.size()) {
        return false;
    }
    GuidanceStateDefinition& active_state = m_definition.states.at(m_active_state_index);
    output.execution.state_index = m_active_state_index;
    output.execution.state_entered = state_entered;
    output.execution.filter_config = active_state.guidance_command_filter;
    output.execution.filter_on_entry = active_state.on_entry_guidance_command_filter;
    output.execution.guidance_active = active_state.guidance_enabled;
    output.execution.autopilot_active = active_state.autopilot_enabled;
    output.execution.pad_constraint_active =
        active_state.plant_constraint == GuidancePlantConstraint::StaticLaunchPad;
    output.diagnostics.v_reference_i_mps = state.v_i_mps;
    output.diagnostics.body_y_specific_force_enabled = active_state.body_y_specific_force_enabled;
    if (!active_state.guidance_enabled) {
        return true;
    }

    GuidanceReferenceOutput reference{};
    const bool direct_specific_force = !active_state.translation.acceleration.empty() &&
                                       active_state.translation.acceleration.front()->role() ==
                                           GuidanceAccelerationRole::DirectBodySpecificForce;
    if (direct_specific_force) {
        core::Vec3 specific_force_ib_b_mps2{};
        if (!active_state.translation.reference ||
            !active_state.translation.reference->advance(
                state, environment, elapsed_in_state_s, dt_s, reference) ||
            !active_state.translation.acceleration.front()->advance(state,
                                                                    environment,
                                                                    reference,
                                                                    elapsed_in_state_s,
                                                                    dt_s,
                                                                    specific_force_ib_b_mps2)) {
            return false;
        }
        output.diagnostics.a_cmd_i_mps2 =
            environment.gravity_i_mps2 + (state.q_b2i * specific_force_ib_b_mps2);
        output.diagnostics.a_cmd_n_mps2 = environment.C_i2n * output.diagnostics.a_cmd_i_mps2;
    }
    else {
        if (!active_state.translation.reference ||
            !active_state.translation.reference->advance(
                state, environment, elapsed_in_state_s, dt_s, reference)) {
            return false;
        }
        for (std::unique_ptr<GuidanceReferenceModifier>& modifier :
             active_state.translation.reference_modifiers) {
            if (!modifier->apply(state, environment, elapsed_in_state_s, dt_s, reference)) {
                return false;
            }
        }
        const core::Vec3 velocity_reference_n_mps = guidance::velocity_n_mps(reference.kinematics);
        core::Vec3 velocity_derivative_command_n_mps2 = reference.additional_feedforward_n_mps2;
        for (std::unique_ptr<GuidanceAccelerationModel>& acceleration :
             active_state.translation.acceleration) {
            core::Vec3 contribution{};
            if (!acceleration->advance(
                    state, environment, reference, elapsed_in_state_s, dt_s, contribution)) {
                return false;
            }
            velocity_derivative_command_n_mps2 += contribution;
        }
        output.diagnostics.v_reference_i_mps =
            guidance::velocity_reference_i_mps(state, environment, velocity_reference_n_mps);
        if (!guidance::velocity_derivative_n_to_acceleration_i(velocity_derivative_command_n_mps2,
                                                               environment,
                                                               output.diagnostics.a_cmd_n_mps2,
                                                               output.diagnostics.a_cmd_i_mps2)) {
            return false;
        }
    }

    if (!active_state.bank->advance(
            reference, output.diagnostics.a_cmd_i_mps2, environment, output.bank_command_n_rad)) {
        return false;
    }
    output.diagnostics.reference_position_e_m = reference.reference_position_e_m;
    output.diagnostics.reference_index = reference.reference_index;
    output.diagnostics.reference_position_valid = reference.reference_position_valid;
    output.diagnostics.bank_to_turn_active = active_state.bank->bank_to_turn_active();
    return output.diagnostics.a_cmd_i_mps2.allFinite() &&
           output.diagnostics.a_cmd_n_mps2.allFinite() && std::isfinite(output.bank_command_n_rad);
}

} // namespace navkit::sim
