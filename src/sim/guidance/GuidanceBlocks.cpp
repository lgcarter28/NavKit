// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/guidance/GuidanceBlocks.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace navkit::sim
{

namespace
{

[[nodiscard]] bool nonnegative_finite(const core::Vec3& value)
{
    return value.allFinite() && (value.array() >= 0.0).all();
}

[[nodiscard]] bool positive_finite(const core::Scalar_t value)
{
    return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool valid_state_and_environment(const TrajectoryControlState& state,
                                               const TrajectoryEnvironment& environment)
{
    return core::timestamp_is_valid(state.t) && state.p_i_m.allFinite() &&
           state.v_i_mps.allFinite() && state.q_b2i.coeffs().allFinite() &&
           environment.p_e_m.allFinite() && environment.v_eb_n_mps.allFinite();
}

} // namespace

GuidanceReferenceRole CurrentStateGuidanceReference::role() const
{
    return GuidanceReferenceRole::CurrentState;
}

bool CurrentStateGuidanceReference::config_is_valid() const
{
    return true;
}

bool CurrentStateGuidanceReference::initialize(const TrajectoryControlState& initial_state,
                                               const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(initial_state, environment);
}

bool CurrentStateGuidanceReference::enter(const TrajectoryControlState& state,
                                          const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool CurrentStateGuidanceReference::advance(const TrajectoryControlState& state,
                                            const TrajectoryEnvironment& environment,
                                            const core::Time_t elapsed_in_state_s,
                                            const core::Time_t dt_s,
                                            GuidanceReferenceOutput& output)
{
    output = {};
    core::Vec3 local_attitude_rpy_rad{};
    if (!valid_state_and_environment(state, environment) || !std::isfinite(elapsed_in_state_s) ||
        elapsed_in_state_s < 0.0 || !positive_finite(dt_s) ||
        !guidance::local_rpy_b2n_rad(state, environment, local_attitude_rpy_rad)) {
        return false;
    }
    output.kinematics.speed_mps = environment.v_eb_n_mps.norm();
    if (output.kinematics.speed_mps > 1.0e-9) {
        const core::Scalar_t horizontal_speed_mps = environment.v_eb_n_mps.head<2>().norm();
        output.kinematics.heading_rad =
            std::atan2(environment.v_eb_n_mps.y(), environment.v_eb_n_mps.x());
        output.kinematics.pitch_rad = std::atan2(-environment.v_eb_n_mps.z(), horizontal_speed_mps);
    }
    else {
        output.kinematics.heading_rad = local_attitude_rpy_rad.z();
        output.kinematics.pitch_rad = local_attitude_rpy_rad.y();
    }
    return true;
}

ConstantSpeedGuidanceReference::ConstantSpeedGuidanceReference(
    ConstantSpeedGuidanceReferenceConfig config)
    : m_config(config)
{}

GuidanceReferenceRole ConstantSpeedGuidanceReference::role() const
{
    return GuidanceReferenceRole::Kinematic;
}

bool ConstantSpeedGuidanceReference::config_is_valid() const
{
    return positive_finite(m_config.speed_mps) && std::isfinite(m_config.heading_rad) &&
           std::isfinite(m_config.pitch_rad);
}

bool ConstantSpeedGuidanceReference::initialize(const TrajectoryControlState& initial_state,
                                                const TrajectoryEnvironment& environment)
{
    core::Vec3 rpy_b2n_rad{};
    if (!config_is_valid() || !valid_state_and_environment(initial_state, environment) ||
        ((m_config.capture_initial_heading || m_config.capture_initial_pitch) &&
         !guidance::local_rpy_b2n_rad(initial_state, environment, rpy_b2n_rad))) {
        return false;
    }
    m_heading_rad = m_config.capture_initial_heading ? rpy_b2n_rad.z() : m_config.heading_rad;
    m_pitch_rad = m_config.capture_initial_pitch ? rpy_b2n_rad.y() : m_config.pitch_rad;
    m_initialized = true;
    return true;
}

bool ConstantSpeedGuidanceReference::enter(const TrajectoryControlState& state,
                                           const TrajectoryEnvironment& environment)
{
    return m_initialized && valid_state_and_environment(state, environment);
}

bool ConstantSpeedGuidanceReference::advance(const TrajectoryControlState& state,
                                             const TrajectoryEnvironment& environment,
                                             const core::Time_t elapsed_in_state_s,
                                             const core::Time_t dt_s,
                                             GuidanceReferenceOutput& output)
{
    output = {};
    if (!m_initialized || !valid_state_and_environment(state, environment) ||
        !std::isfinite(elapsed_in_state_s) || elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    output.kinematics.speed_mps = m_config.speed_mps;
    output.kinematics.heading_rad = m_heading_rad;
    output.kinematics.pitch_rad = m_pitch_rad;
    return true;
}

WaypointGuidanceReference::WaypointGuidanceReference(WaypointGuidanceReferenceConfig config)
    : m_config(std::move(config))
{}

GuidanceReferenceRole WaypointGuidanceReference::role() const
{
    return GuidanceReferenceRole::Kinematic;
}

bool WaypointGuidanceReference::config_is_valid() const
{
    if (m_config.waypoint_e_m.empty() || !positive_finite(m_config.speed_mps) ||
        !positive_finite(m_config.acceptance_radius_m) ||
        !std::isfinite(m_config.heading_error_gain_mps2_per_rad) ||
        m_config.heading_error_gain_mps2_per_rad < 0.0) {
        return false;
    }
    return std::ranges::all_of(m_config.waypoint_e_m, [](const core::Vec3& waypoint_e_m) {
        return waypoint_e_m.allFinite() && waypoint_e_m.norm() > 0.0;
    });
}

bool WaypointGuidanceReference::initialize(const TrajectoryControlState& initial_state,
                                           const TrajectoryEnvironment& environment)
{
    if (!config_is_valid() || !valid_state_and_environment(initial_state, environment)) {
        return false;
    }
    m_waypoint_index = 0U;
    m_route_complete = false;
    m_terminal_heading_rad = 0.0;
    m_initialized = true;
    return true;
}

bool WaypointGuidanceReference::enter(const TrajectoryControlState& state,
                                      const TrajectoryEnvironment& environment)
{
    return m_initialized && valid_state_and_environment(state, environment);
}

bool WaypointGuidanceReference::advance(const TrajectoryControlState& state,
                                        const TrajectoryEnvironment& environment,
                                        const core::Time_t elapsed_in_state_s,
                                        const core::Time_t dt_s,
                                        GuidanceReferenceOutput& output)
{
    output = {};
    if (!m_initialized || !valid_state_and_environment(state, environment) ||
        !std::isfinite(elapsed_in_state_s) || elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }

    core::Scalar_t current_heading_rad = m_terminal_heading_rad;
    if (environment.v_eb_n_mps.head<2>().norm() > 1.0e-9) {
        current_heading_rad = std::atan2(environment.v_eb_n_mps.y(), environment.v_eb_n_mps.x());
    }
    core::Vec3 target_delta_n_m{core::Vec3::Zero()};
    if (!m_route_complete) {
        while (m_waypoint_index < m_config.waypoint_e_m.size()) {
            target_delta_n_m = environment.C_e2n *
                               (m_config.waypoint_e_m.at(m_waypoint_index) - environment.p_e_m);
            if (target_delta_n_m.head<2>().norm() > m_config.acceptance_radius_m) {
                break;
            }
            if (m_waypoint_index + 1U < m_config.waypoint_e_m.size()) {
                ++m_waypoint_index;
                continue;
            }
            m_terminal_heading_rad = current_heading_rad;
            m_route_complete = true;
            break;
        }
    }

    const core::Scalar_t desired_heading_rad =
        m_route_complete ? m_terminal_heading_rad
                         : std::atan2(target_delta_n_m.y(), target_delta_n_m.x());
    const core::Scalar_t heading_error_rad =
        guidance::wrap_angle_rad(desired_heading_rad - current_heading_rad);
    const core::Scalar_t lateral_acceleration_mps2 =
        m_config.heading_error_gain_mps2_per_rad * heading_error_rad;

    output.kinematics.speed_mps = m_config.speed_mps;
    output.kinematics.heading_rad = current_heading_rad;
    output.additional_feedforward_n_mps2 =
        core::Vec3{-std::sin(current_heading_rad) * lateral_acceleration_mps2,
                   std::cos(current_heading_rad) * lateral_acceleration_mps2,
                   0.0};
    output.reference_position_e_m = m_config.waypoint_e_m.at(m_waypoint_index);
    output.reference_index = m_waypoint_index;
    output.reference_position_valid = !m_route_complete;
    return output.additional_feedforward_n_mps2.allFinite();
}

HorizontalSinusoidalGuidanceReferenceModifier::HorizontalSinusoidalGuidanceReferenceModifier(
    SinusoidalGuidanceReferenceModifierConfig config)
    : m_config(config)
{}

bool HorizontalSinusoidalGuidanceReferenceModifier::config_is_valid() const
{
    return std::isfinite(m_config.amplitude_rad) && positive_finite(m_config.frequency_hz) &&
           std::isfinite(m_config.phase_rad);
}

bool HorizontalSinusoidalGuidanceReferenceModifier::initialize(
    const TrajectoryControlState& initial_state, const TrajectoryEnvironment& environment)
{
    return config_is_valid() && valid_state_and_environment(initial_state, environment);
}

bool HorizontalSinusoidalGuidanceReferenceModifier::enter(const TrajectoryControlState& state,
                                                          const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool HorizontalSinusoidalGuidanceReferenceModifier::apply(const TrajectoryControlState& state,
                                                          const TrajectoryEnvironment& environment,
                                                          const core::Time_t elapsed_in_state_s,
                                                          const core::Time_t dt_s,
                                                          GuidanceReferenceOutput& reference)
{
    if (!valid_state_and_environment(state, environment) || !std::isfinite(elapsed_in_state_s) ||
        elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    const core::Scalar_t frequency_radps =
        2.0 * std::numbers::pi_v<core::Scalar_t> * m_config.frequency_hz;
    const core::Scalar_t phase_rad = (frequency_radps * elapsed_in_state_s) + m_config.phase_rad;
    reference.kinematics.heading_rad += m_config.amplitude_rad * std::sin(phase_rad);
    reference.kinematics.heading_rate_radps +=
        m_config.amplitude_rad * frequency_radps * std::cos(phase_rad);
    return true;
}

VerticalSinusoidalGuidanceReferenceModifier::VerticalSinusoidalGuidanceReferenceModifier(
    SinusoidalGuidanceReferenceModifierConfig config)
    : m_config(config)
{}

bool VerticalSinusoidalGuidanceReferenceModifier::config_is_valid() const
{
    return std::isfinite(m_config.amplitude_rad) && positive_finite(m_config.frequency_hz) &&
           std::isfinite(m_config.phase_rad);
}

bool VerticalSinusoidalGuidanceReferenceModifier::initialize(
    const TrajectoryControlState& initial_state, const TrajectoryEnvironment& environment)
{
    return config_is_valid() && valid_state_and_environment(initial_state, environment);
}

bool VerticalSinusoidalGuidanceReferenceModifier::enter(const TrajectoryControlState& state,
                                                        const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool VerticalSinusoidalGuidanceReferenceModifier::apply(const TrajectoryControlState& state,
                                                        const TrajectoryEnvironment& environment,
                                                        const core::Time_t elapsed_in_state_s,
                                                        const core::Time_t dt_s,
                                                        GuidanceReferenceOutput& reference)
{
    if (!valid_state_and_environment(state, environment) || !std::isfinite(elapsed_in_state_s) ||
        elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    const core::Scalar_t frequency_radps =
        2.0 * std::numbers::pi_v<core::Scalar_t> * m_config.frequency_hz;
    const core::Scalar_t phase_rad = (frequency_radps * elapsed_in_state_s) + m_config.phase_rad;
    reference.kinematics.pitch_rad += m_config.amplitude_rad * std::sin(phase_rad);
    reference.kinematics.pitch_rate_radps +=
        m_config.amplitude_rad * frequency_radps * std::cos(phase_rad);
    return true;
}

VelocityTrackingGuidanceAcceleration::VelocityTrackingGuidanceAcceleration(
    VelocityTrackingGuidanceAccelerationConfig config)
    : m_config(std::move(config))
{}

GuidanceAccelerationRole PathFeedforwardGuidanceAcceleration::role() const
{
    return GuidanceAccelerationRole::AdditiveVelocityDerivative;
}

bool PathFeedforwardGuidanceAcceleration::config_is_valid() const
{
    return true;
}

bool PathFeedforwardGuidanceAcceleration::initialize(const TrajectoryControlState& initial_state,
                                                     const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(initial_state, environment);
}

bool PathFeedforwardGuidanceAcceleration::enter(const TrajectoryControlState& state,
                                                const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool PathFeedforwardGuidanceAcceleration::advance(const TrajectoryControlState& state,
                                                  const TrajectoryEnvironment& environment,
                                                  const GuidanceReferenceOutput& reference,
                                                  const core::Time_t elapsed_in_state_s,
                                                  const core::Time_t dt_s,
                                                  core::Vec3& contribution)
{
    contribution = core::Vec3::Zero();
    if (!valid_state_and_environment(state, environment) || !std::isfinite(elapsed_in_state_s) ||
        elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    contribution = guidance::feedforward_acceleration_n_mps2(reference.kinematics);
    return contribution.allFinite();
}

GuidanceAccelerationRole VelocityTrackingGuidanceAcceleration::role() const
{
    return GuidanceAccelerationRole::AdditiveVelocityDerivative;
}

bool VelocityTrackingGuidanceAcceleration::config_is_valid() const
{
    return nonnegative_finite(m_config.gain_n_1ps);
}

bool VelocityTrackingGuidanceAcceleration::initialize(const TrajectoryControlState& initial_state,
                                                      const TrajectoryEnvironment& environment)
{
    return config_is_valid() && valid_state_and_environment(initial_state, environment);
}

bool VelocityTrackingGuidanceAcceleration::enter(const TrajectoryControlState& state,
                                                 const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool VelocityTrackingGuidanceAcceleration::advance(const TrajectoryControlState& state,
                                                   const TrajectoryEnvironment& environment,
                                                   const GuidanceReferenceOutput& reference,
                                                   const core::Time_t elapsed_in_state_s,
                                                   const core::Time_t dt_s,
                                                   core::Vec3& contribution)
{
    contribution = core::Vec3::Zero();
    if (!valid_state_and_environment(state, environment) || !std::isfinite(elapsed_in_state_s) ||
        elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    contribution = m_config.gain_n_1ps.cwiseProduct(guidance::velocity_n_mps(reference.kinematics) -
                                                    environment.v_eb_n_mps);
    return contribution.allFinite();
}

AltitudeHoldPdGuidanceAcceleration::AltitudeHoldPdGuidanceAcceleration(
    AltitudeHoldPdGuidanceAccelerationConfig config)
    : m_config(config)
{}

GuidanceAccelerationRole AltitudeHoldPdGuidanceAcceleration::role() const
{
    return GuidanceAccelerationRole::AdditiveVelocityDerivative;
}

bool AltitudeHoldPdGuidanceAcceleration::config_is_valid() const
{
    return std::isfinite(m_config.target_altitude_m) &&
           std::isfinite(m_config.proportional_gain_1ps2) &&
           m_config.proportional_gain_1ps2 >= 0.0 && std::isfinite(m_config.derivative_gain_1ps) &&
           m_config.derivative_gain_1ps >= 0.0;
}

bool AltitudeHoldPdGuidanceAcceleration::initialize(const TrajectoryControlState& initial_state,
                                                    const TrajectoryEnvironment& environment)
{
    if (!config_is_valid() || !valid_state_and_environment(initial_state, environment)) {
        return false;
    }
    if (m_config.capture_initial_altitude &&
        !guidance::altitude_m(environment.p_e_m, m_target_altitude_m)) {
        return false;
    }
    if (!m_config.capture_initial_altitude) {
        m_target_altitude_m = m_config.target_altitude_m;
    }
    m_initialized = true;
    return true;
}

bool AltitudeHoldPdGuidanceAcceleration::enter(const TrajectoryControlState& state,
                                               const TrajectoryEnvironment& environment)
{
    return m_initialized && valid_state_and_environment(state, environment);
}

bool AltitudeHoldPdGuidanceAcceleration::advance(const TrajectoryControlState& state,
                                                 const TrajectoryEnvironment& environment,
                                                 const GuidanceReferenceOutput& /*reference*/,
                                                 const core::Time_t elapsed_in_state_s,
                                                 const core::Time_t dt_s,
                                                 core::Vec3& contribution)
{
    contribution = core::Vec3::Zero();
    core::Scalar_t current_altitude_m{};
    if (!m_initialized || !valid_state_and_environment(state, environment) ||
        !std::isfinite(elapsed_in_state_s) || elapsed_in_state_s < 0.0 || !positive_finite(dt_s) ||
        !guidance::altitude_m(environment.p_e_m, current_altitude_m)) {
        return false;
    }
    contribution.z() =
        (m_config.proportional_gain_1ps2 * (current_altitude_m - m_target_altitude_m)) -
        (m_config.derivative_gain_1ps * environment.v_eb_n_mps.z());
    return std::isfinite(contribution.z());
}

BodySpecificForceGuidanceAcceleration::BodySpecificForceGuidanceAcceleration(
    BodySpecificForceGuidanceAccelerationConfig config)
    : m_config(std::move(config))
{}

GuidanceAccelerationRole BodySpecificForceGuidanceAcceleration::role() const
{
    return GuidanceAccelerationRole::DirectBodySpecificForce;
}

bool BodySpecificForceGuidanceAcceleration::config_is_valid() const
{
    return m_config.specific_force_ib_b_mps2.allFinite();
}

bool BodySpecificForceGuidanceAcceleration::initialize(const TrajectoryControlState& initial_state,
                                                       const TrajectoryEnvironment& environment)
{
    return config_is_valid() && valid_state_and_environment(initial_state, environment);
}

bool BodySpecificForceGuidanceAcceleration::enter(const TrajectoryControlState& state,
                                                  const TrajectoryEnvironment& environment)
{
    return valid_state_and_environment(state, environment);
}

bool BodySpecificForceGuidanceAcceleration::advance(const TrajectoryControlState& state,
                                                    const TrajectoryEnvironment& environment,
                                                    const GuidanceReferenceOutput& reference,
                                                    const core::Time_t elapsed_in_state_s,
                                                    const core::Time_t dt_s,
                                                    core::Vec3& contribution)
{
    contribution = core::Vec3::Zero();
    if (!valid_state_and_environment(state, environment) ||
        !reference.additional_feedforward_n_mps2.allFinite() ||
        !std::isfinite(elapsed_in_state_s) || elapsed_in_state_s < 0.0 || !positive_finite(dt_s)) {
        return false;
    }
    contribution = m_config.specific_force_ib_b_mps2;
    return true;
}

bool ZeroBankGuidancePolicy::config_is_valid() const
{
    return true;
}

bool ZeroBankGuidancePolicy::advance(const GuidanceReferenceOutput& reference,
                                     const core::Vec3& acceleration_command_i_mps2,
                                     const TrajectoryEnvironment& environment,
                                     core::Scalar_t& bank_command_n_rad) const
{
    if (!reference.additional_feedforward_n_mps2.allFinite() ||
        !acceleration_command_i_mps2.allFinite() || !environment.p_e_m.allFinite()) {
        return false;
    }
    bank_command_n_rad = 0.0;
    return true;
}

bool ZeroBankGuidancePolicy::bank_to_turn_active() const
{
    return false;
}

CoordinatedBankToTurnGuidancePolicy::CoordinatedBankToTurnGuidancePolicy(
    CoordinatedBankToTurnGuidancePolicyConfig config)
    : m_config(config)
{}

bool CoordinatedBankToTurnGuidancePolicy::config_is_valid() const
{
    return positive_finite(m_config.maximum_bank_angle_rad) &&
           m_config.maximum_bank_angle_rad <= std::numbers::pi_v<core::Scalar_t>;
}

bool CoordinatedBankToTurnGuidancePolicy::advance(const GuidanceReferenceOutput& reference,
                                                  const core::Vec3& acceleration_command_i_mps2,
                                                  const TrajectoryEnvironment& environment,
                                                  core::Scalar_t& bank_command_n_rad) const
{
    return config_is_valid() &&
           guidance::coordinated_bank_command_rad(acceleration_command_i_mps2,
                                                  reference.kinematics.heading_rad,
                                                  m_config.maximum_bank_angle_rad,
                                                  environment,
                                                  bank_command_n_rad);
}

bool CoordinatedBankToTurnGuidancePolicy::bank_to_turn_active() const
{
    return true;
}

} // namespace navkit::sim
