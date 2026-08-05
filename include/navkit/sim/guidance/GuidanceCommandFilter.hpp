// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/guidance/GuidanceCommand.hpp"

namespace navkit::sim
{

/** Return true when all Guidance command-filter time constants are finite and nonnegative. */
[[nodiscard]] bool
guidance_command_filter_config_is_valid(const GuidanceCommandFilterConfig& config);

/** Return true when an optional entry-window override is internally consistent. */
[[nodiscard]] bool guidance_command_filter_state_entry_config_is_valid(
    const GuidanceCommandFilterStateEntryConfig& config);

/**
 * Persistent first-order LPF between raw Guidance output and downstream consumers.
 *
 * A state's nominal configuration applies continuously. An explicit state-entry override takes
 * effect on the first sample of the entered state and automatically restores the nominal values
 * after its configured duration. Neither path resets filter state.
 */
class GuidanceCommandFilter
{
public:
    /** Initialize filter state and the initially active time constants. */
    [[nodiscard]] bool initialize(const core::Vec3& initial_specific_force_ib_b_mps2,
                                  core::Scalar_t initial_bank_n_rad,
                                  const GuidanceCommandFilterConfig& initial_config);

    /** Advance all four channels, including any mode-entry override window. */
    [[nodiscard]] bool advance(const core::Vec3& specific_force_command_ib_b_mps2,
                               core::Scalar_t bank_command_n_rad,
                               const GuidanceCommandFilterConfig& requested_config,
                               const GuidanceCommandFilterStateEntryConfig& requested_on_entry,
                               bool state_entered,
                               core::Time_t dt_s,
                               GuidanceCommandFilterOutput& output);

    [[nodiscard]] const GuidanceCommandFilterConfig& active_config() const;

private:
    GuidanceCommandFilterConfig m_nominal_config{};
    GuidanceCommandFilterConfig m_active_config{};
    core::Vec3 m_specific_force_filtered_ib_b_mps2{core::Vec3::Zero()};
    core::Scalar_t m_bank_filtered_n_rad{};
    core::Time_t m_state_entry_time_remaining_s{};
    bool m_initialized{false};
};

} // namespace navkit::sim
