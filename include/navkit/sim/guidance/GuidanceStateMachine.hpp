// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/guidance/GuidanceBlocks.hpp"
#include "navkit/sim/guidance/GuidanceModel.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace navkit::sim
{

enum class GuidanceCyclePolicy
{
    Reject,
    Allow,
};

enum class GuidancePlantConstraint
{
    None,
    StaticLaunchPad,
};

struct GuidanceElapsedTransition
{
    std::string to{};
    core::Time_t elapsed_in_state_s{};
    std::size_t priority{};
};

struct GuidanceTranslationDefinition
{
    std::unique_ptr<GuidanceReferenceModel> reference{};
    std::vector<std::unique_ptr<GuidanceReferenceModifier>> reference_modifiers{};
    std::vector<std::unique_ptr<GuidanceAccelerationModel>> acceleration{};
};

/** One runtime-selected Guidance state and its explicitly typed command pipeline. */
struct GuidanceStateDefinition
{
    std::string id{};
    GuidancePlantConstraint plant_constraint{GuidancePlantConstraint::None};
    GuidanceTranslationDefinition translation{};
    std::unique_ptr<GuidanceBankPolicy> bank{};
    GuidanceCommandFilterConfig guidance_command_filter{};
    GuidanceCommandFilterStateEntryConfig on_entry_guidance_command_filter{};
    std::vector<GuidanceElapsedTransition> transitions{};
    bool terminal{false};
    bool guidance_enabled{true};
    bool autopilot_enabled{true};
    bool body_y_specific_force_enabled{true};
};

/** Complete runtime Guidance graph constructed from validated scenario configuration. */
struct GuidanceStateMachineDefinition
{
    std::string initial_state_id{};
    GuidanceCyclePolicy cycle_policy{GuidanceCyclePolicy::Reject};
    std::vector<GuidanceStateDefinition> states{};
};

/** Validate graph structure, state pipelines, and deterministic transition semantics. */
[[nodiscard]] bool
guidance_state_machine_definition_is_valid(const GuidanceStateMachineDefinition& definition,
                                           std::string& diagnostic);

/** Source-agnostic runtime Guidance state machine used by generated trajectories. */
class GuidanceStateMachine final : public GuidanceModel
{
public:
    explicit GuidanceStateMachine(GuidanceStateMachineDefinition definition);

    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;

    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               core::Time_t dt_s,
                               GuidanceOutput& output) override;

    [[nodiscard]] std::size_t active_state_index() const;
    [[nodiscard]] const std::string& active_state_id() const;
    [[nodiscard]] bool active_state_is_terminal() const;

private:
    [[nodiscard]] bool enter_state(std::size_t state_index,
                                   const TrajectoryControlState& state,
                                   const TrajectoryEnvironment& environment);
    [[nodiscard]] bool evaluate_transitions(const TrajectoryControlState& state,
                                            const TrajectoryEnvironment& environment,
                                            bool& state_entered);
    [[nodiscard]] bool evaluate_active_state(const TrajectoryControlState& state,
                                             const TrajectoryEnvironment& environment,
                                             core::Time_t elapsed_in_state_s,
                                             core::Time_t dt_s,
                                             bool state_entered,
                                             GuidanceOutput& output);

    GuidanceStateMachineDefinition m_definition{};
    core::Timestamp m_state_entry_time{};
    std::size_t m_active_state_index{};
    bool m_first_output_pending{false};
    bool m_initialized{false};
};

} // namespace navkit::sim
