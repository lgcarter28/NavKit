// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"

#include "navkit/sim/math/FirstOrderResponse.hpp"

#include <algorithm>
#include <cmath>

namespace navkit::sim
{

bool guidance_command_filter_config_is_valid(const GuidanceCommandFilterConfig& config)
{
    return config.specific_force_time_constant_b_s.allFinite() &&
           (config.specific_force_time_constant_b_s.array() >= 0.0).all() &&
           std::isfinite(config.bank_time_constant_s) && config.bank_time_constant_s >= 0.0;
}

bool guidance_command_filter_state_entry_config_is_valid(
    const GuidanceCommandFilterStateEntryConfig& config)
{
    const bool constants_valid = config.specific_force_time_constant_b_s.allFinite() &&
                                 (config.specific_force_time_constant_b_s.array() >= 0.0).all() &&
                                 std::isfinite(config.bank_time_constant_s) &&
                                 config.bank_time_constant_s >= 0.0;
    const bool duration_valid =
        std::isfinite(config.duration_s) && ((config.enabled && config.duration_s > 0.0) ||
                                             (!config.enabled && config.duration_s == 0.0));
    return constants_valid && duration_valid;
}

bool GuidanceCommandFilter::initialize(const core::Vec3& initial_specific_force_ib_b_mps2,
                                       const core::Scalar_t initial_bank_n_rad,
                                       const GuidanceCommandFilterConfig& initial_config)
{
    if (!initial_specific_force_ib_b_mps2.allFinite() || !std::isfinite(initial_bank_n_rad) ||
        !guidance_command_filter_config_is_valid(initial_config)) {
        return false;
    }
    m_specific_force_filtered_ib_b_mps2 = initial_specific_force_ib_b_mps2;
    m_bank_filtered_n_rad = initial_bank_n_rad;
    m_nominal_config = initial_config;
    m_active_config = initial_config;
    m_state_entry_time_remaining_s = 0.0;
    m_initialized = true;
    return true;
}

bool GuidanceCommandFilter::advance(const core::Vec3& specific_force_command_ib_b_mps2,
                                    const core::Scalar_t bank_command_n_rad,
                                    const GuidanceCommandFilterConfig& requested_config,
                                    const GuidanceCommandFilterStateEntryConfig& requested_on_entry,
                                    const bool state_entered,
                                    const core::Time_t dt_s,
                                    GuidanceCommandFilterOutput& output)
{
    output = {};
    if (!m_initialized || !specific_force_command_ib_b_mps2.allFinite() ||
        !std::isfinite(bank_command_n_rad) || !std::isfinite(dt_s) || dt_s <= 0.0 ||
        !guidance_command_filter_config_is_valid(requested_config) ||
        !guidance_command_filter_state_entry_config_is_valid(requested_on_entry)) {
        return false;
    }

    m_nominal_config = requested_config;
    if (state_entered && requested_on_entry.enabled) {
        m_active_config.specific_force_time_constant_b_s =
            requested_on_entry.specific_force_time_constant_b_s;
        m_active_config.bank_time_constant_s = requested_on_entry.bank_time_constant_s;
        m_state_entry_time_remaining_s = requested_on_entry.duration_s;
    }
    else if (state_entered) {
        m_active_config = m_nominal_config;
        m_state_entry_time_remaining_s = 0.0;
    }
    else if (m_state_entry_time_remaining_s <= 0.0) {
        m_active_config = m_nominal_config;
    }

    m_specific_force_filtered_ib_b_mps2 =
        exact_first_order_step(m_specific_force_filtered_ib_b_mps2,
                               specific_force_command_ib_b_mps2,
                               m_active_config.specific_force_time_constant_b_s,
                               dt_s);
    m_bank_filtered_n_rad = exact_first_order_step(
        m_bank_filtered_n_rad, bank_command_n_rad, m_active_config.bank_time_constant_s, dt_s);

    output.specific_force_command_ib_b_mps2 = specific_force_command_ib_b_mps2;
    output.specific_force_filtered_ib_b_mps2 = m_specific_force_filtered_ib_b_mps2;
    output.bank_command_n_rad = bank_command_n_rad;
    output.bank_filtered_n_rad = m_bank_filtered_n_rad;

    if (m_state_entry_time_remaining_s > 0.0) {
        m_state_entry_time_remaining_s = std::max(0.0, m_state_entry_time_remaining_s - dt_s);
        if (m_state_entry_time_remaining_s == 0.0) {
            m_active_config = m_nominal_config;
        }
    }
    return true;
}

const GuidanceCommandFilterConfig& GuidanceCommandFilter::active_config() const
{
    return m_active_config;
}

} // namespace navkit::sim
