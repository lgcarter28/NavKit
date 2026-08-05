// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <cstddef>

namespace navkit::sim
{

/** Optional temporary first-order shaping applied immediately after entering a Guidance state. */
struct GuidanceCommandFilterStateEntryConfig
{
    core::Vec3 specific_force_time_constant_b_s{core::Vec3::Zero()};
    core::Time_t bank_time_constant_s{};
    core::Time_t duration_s{};
    bool enabled{false};
};

/** Persistent first-order shaping applied at the Guidance output boundary. */
struct GuidanceCommandFilterConfig
{
    core::Vec3 specific_force_time_constant_b_s{core::Vec3::Zero()};
    core::Time_t bank_time_constant_s{};
};

/** Minimal source-agnostic Guidance command passed to the selected Autopilot. */
struct GuidanceCommand
{
    core::Vec3 specific_force_command_ib_b_mps2{core::Vec3::Zero()};
    core::Scalar_t bank_command_n_rad{};
};

/** State-machine and command-filter control data retained outside the command boundary. */
struct GuidanceExecutionState
{
    GuidanceCommandFilterConfig filter_config{};
    GuidanceCommandFilterStateEntryConfig filter_on_entry{};
    std::size_t state_index{};
    bool state_entered{false};
    bool guidance_active{false};
    bool autopilot_active{false};
    bool pad_constraint_active{false};
};

/** Secondary Guidance values retained for diagnostics and command realization. */
struct GuidanceDiagnostics
{
    core::Vec3 v_reference_i_mps{core::Vec3::Zero()};
    core::Vec3 a_cmd_n_mps2{core::Vec3::Zero()};
    core::Vec3 a_cmd_i_mps2{core::Vec3::Zero()};
    core::Vec3 reference_position_e_m{core::Vec3::Zero()};
    std::size_t reference_index{};
    bool reference_position_valid{false};
    bool bank_to_turn_active{false};
    bool body_y_specific_force_enabled{true};
};

/** Complete Guidance producer output before command filtering and Autopilot consumption. */
struct GuidanceOutput
{
    core::Scalar_t bank_command_n_rad{};
    GuidanceExecutionState execution{};
    GuidanceDiagnostics diagnostics{};
};

/** Raw and filtered Guidance outputs presented to downstream consumers. */
struct GuidanceCommandFilterOutput
{
    core::Vec3 specific_force_command_ib_b_mps2{core::Vec3::Zero()};
    core::Vec3 specific_force_filtered_ib_b_mps2{core::Vec3::Zero()};
    core::Scalar_t bank_command_n_rad{};
    core::Scalar_t bank_filtered_n_rad{};
};

} // namespace navkit::sim
