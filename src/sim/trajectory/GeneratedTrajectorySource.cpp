// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/GeneratedTrajectorySource.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/core/time/RationalSchedule.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"

#include <cmath>
#include <utility>

namespace navkit::sim
{

namespace
{

using Planet = core::environment::Wgs84;
using Gravity = core::environment::J2<Planet>;

[[nodiscard]] core::Time_t elapsed_seconds(const core::Timestamp& first,
                                           const core::Timestamp& second)
{
    core::Duration elapsed{};
    if (!core::elapsed_time(second, first, elapsed)) {
        return 0.0;
    }
    return core::duration_seconds(elapsed);
}

[[nodiscard]] TrajectoryControlState control_state_from(const TrajectoryDynamicState& state)
{
    return TrajectoryControlState{
        .t = state.t,
        .p_i_m = state.p_i_m,
        .v_i_mps = state.v_i_mps,
        .a_i_mps2 = state.a_i_mps2,
        .q_b2i = state.q_b2i,
        .w_ib_b_radps = state.w_ib_b_radps,
    };
}

[[nodiscard]] bool control_state_is_valid(const TrajectoryControlState& state)
{
    return core::timestamp_is_valid(state.t) && state.p_i_m.allFinite() &&
           state.v_i_mps.allFinite() && state.a_i_mps2.allFinite() &&
           state.q_b2i.coeffs().allFinite() && state.q_b2i.norm() > 0.0 &&
           state.w_ib_b_radps.allFinite();
}

[[nodiscard]] bool environment_at(const TrajectoryControlState& state,
                                  const core::Timestamp& t_epoch,
                                  TrajectoryEnvironment& environment)
{
    const core::Time_t elapsed_s = elapsed_seconds(t_epoch, state.t);
    environment.elapsed_s = elapsed_s;
    if (!core::frames::fixed_to_inertial_matrix<Planet>(elapsed_s, environment.C_e2i) ||
        !core::frames::inertial_to_fixed_position<Planet>(
            state.p_i_m, elapsed_s, environment.p_e_m) ||
        !core::frames::inertial_to_fixed_velocity<Planet>(
            state.p_i_m, state.v_i_mps, elapsed_s, environment.v_eb_e_mps) ||
        !core::frames::inertial_to_fixed_acceleration<Planet>(
            state.p_i_m, state.v_i_mps, state.a_i_mps2, elapsed_s, environment.a_eb_e_mps2) ||
        !core::frames::ecef_to_ned_matrix(environment.p_e_m, environment.C_e2n)) {
        return false;
    }
    environment.C_i2e = environment.C_e2i.transpose();
    environment.C_i2n = environment.C_e2n * environment.C_i2e;
    environment.C_n2i = environment.C_i2n.transpose();
    environment.v_eb_n_mps = environment.C_e2n * environment.v_eb_e_mps;
    const core::Vec3 gravity_e_mps2 = Gravity::acceleration(environment.p_e_m);
    environment.gravity_i_mps2 = environment.C_e2i * gravity_e_mps2;
    environment.gravity_n_mps2 = environment.C_e2n * gravity_e_mps2;
    return environment.v_eb_n_mps.allFinite() && environment.a_eb_e_mps2.allFinite() &&
           environment.gravity_i_mps2.allFinite() && environment.gravity_n_mps2.allFinite();
}

[[nodiscard]] bool constrained_state_at(const TrajectoryInitialCondition& initial,
                                        const core::Timestamp& t_epoch,
                                        const core::Timestamp& t,
                                        TrajectoryDynamicState& state)
{
    const core::Time_t elapsed_s = elapsed_seconds(t_epoch, t);
    core::Mat3 c_e2i{};
    if (!core::frames::fixed_to_inertial_matrix<Planet>(elapsed_s, c_e2i) ||
        !core::frames::fixed_to_inertial_position<Planet>(initial.p_e_m, elapsed_s, state.p_i_m) ||
        !core::frames::fixed_to_inertial_velocity<Planet>(
            initial.p_e_m, initial.v_e_mps, elapsed_s, state.v_i_mps) ||
        !core::frames::fixed_to_inertial_acceleration<Planet>(
            initial.p_e_m, initial.v_e_mps, core::Vec3::Zero(), elapsed_s, state.a_i_mps2)) {
        return false;
    }
    state.t = t;
    state.q_b2i = core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<core::Scalar_t>{c_e2i} * initial.q_b2e);
    const core::Vec3 planet_rate_e_radps = core::environment::planet_rate_fixed_radps<Planet>();
    state.w_ib_b_radps = initial.q_b2e.conjugate() * planet_rate_e_radps;
    TrajectoryEnvironment environment{};
    if (!environment_at(control_state_from(state), t_epoch, environment)) {
        return false;
    }
    state.specific_force_ib_b_mps2 =
        state.q_b2i.conjugate() * (state.a_i_mps2 - environment.gravity_i_mps2);
    return state.specific_force_ib_b_mps2.allFinite();
}

[[nodiscard]] bool truth_from_state(const TrajectoryDynamicState& state,
                                    const core::Timestamp& t_epoch,
                                    TruthSample& sample)
{
    const core::Time_t elapsed_s = elapsed_seconds(t_epoch, state.t);
    core::Mat3 c_e2i{};
    sample = {};
    sample.t = state.t;
    if (!core::frames::fixed_to_inertial_matrix<Planet>(elapsed_s, c_e2i) ||
        !core::frames::inertial_to_fixed_position<Planet>(state.p_i_m, elapsed_s, sample.p_e) ||
        !core::frames::inertial_to_fixed_velocity<Planet>(
            state.p_i_m, state.v_i_mps, elapsed_s, sample.v_e)) {
        return false;
    }
    sample.q_b2e = core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<core::Scalar_t>{c_e2i.transpose()} * state.q_b2i);
    sample.w_ib_b_radps = state.w_ib_b_radps;
    return sample.p_e.allFinite() && sample.v_e.allFinite();
}

[[nodiscard]] bool bank_angle_from(const TrajectoryDynamicState& state,
                                   const TrajectoryEnvironment& environment,
                                   core::Scalar_t& bank_rad)
{
    const Eigen::Quaternion<core::Scalar_t> q_b2n = core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<core::Scalar_t>{environment.C_i2n} * state.q_b2i);
    bank_rad = core::math::rpy_rad_from_quaternion(q_b2n).x();
    return std::isfinite(bank_rad);
}

[[nodiscard]] bool guidance_command_from(const GuidanceOutput& guidance,
                                         const TrajectoryDynamicState& state,
                                         const TrajectoryEnvironment& environment,
                                         GuidanceCommand& command)
{
    command = {};
    command.bank_command_n_rad = guidance.bank_command_n_rad;
    command.specific_force_command_ib_b_mps2 =
        state.q_b2i.conjugate() * (guidance.diagnostics.a_cmd_i_mps2 - environment.gravity_i_mps2);
    if (guidance.diagnostics.bank_to_turn_active &&
        !guidance.diagnostics.body_y_specific_force_enabled) {
        command.specific_force_command_ib_b_mps2.y() = 0.0;
    }
    return command.specific_force_command_ib_b_mps2.allFinite() &&
           std::isfinite(command.bank_command_n_rad);
}

[[nodiscard]] AutopilotState autopilot_state_from(const TrajectoryControlState& state,
                                                  const TrajectoryEnvironment& environment)
{
    return AutopilotState{
        .q_b2i = state.q_b2i,
        .w_ib_b_radps = state.w_ib_b_radps,
        .v_eb_n_mps = environment.v_eb_n_mps,
        .C_i2n = environment.C_i2n,
    };
}

[[nodiscard]] AutopilotExecutionState
autopilot_execution_from(const GuidanceExecutionState& execution)
{
    return AutopilotExecutionState{
        .active = execution.autopilot_active,
        .hold_initial_attitude = execution.pad_constraint_active,
    };
}

[[nodiscard]] TrajectoryDiagnostics
diagnostics_from(const TrajectoryDynamicState& state,
                 const TrajectoryEnvironment& environment,
                 const GuidanceOutput& guidance,
                 const GuidanceCommandFilterOutput& guidance_filter,
                 const AutopilotOutput& autopilot,
                 const VehicleCommand& vehicle_command,
                 const VehicleResponseOutput& response)
{
    TrajectoryDiagnostics diagnostics{};
    diagnostics.p_i_m = state.p_i_m;
    diagnostics.v_i_mps = state.v_i_mps;
    diagnostics.a_i_mps2 = state.a_i_mps2;
    diagnostics.q_b2i = state.q_b2i;
    diagnostics.w_ib_b_radps = state.w_ib_b_radps;
    diagnostics.specific_force_ib_b_mps2 = state.specific_force_ib_b_mps2;

    diagnostics.guidance_velocity_reference_i_mps = guidance.diagnostics.v_reference_i_mps;
    diagnostics.guidance_acceleration_command_i_mps2 = guidance.diagnostics.a_cmd_i_mps2;
    diagnostics.guidance_acceleration_command_n_mps2 = guidance.diagnostics.a_cmd_n_mps2;
    diagnostics.guidance_acceleration_command_b_mps2 =
        state.q_b2i.conjugate() * guidance.diagnostics.a_cmd_i_mps2;
    diagnostics.guidance_acceleration_response_i_mps2 = state.a_i_mps2;
    diagnostics.guidance_acceleration_response_n_mps2 = environment.C_i2n * state.a_i_mps2;
    diagnostics.guidance_acceleration_response_b_mps2 = state.q_b2i.conjugate() * state.a_i_mps2;
    diagnostics.guidance_specific_force_command_b_mps2 =
        guidance_filter.specific_force_command_ib_b_mps2;
    diagnostics.guidance_specific_force_filtered_b_mps2 =
        guidance_filter.specific_force_filtered_ib_b_mps2;
    diagnostics.guidance_reference_position_e_m = guidance.diagnostics.reference_position_e_m;
    diagnostics.guidance_bank_command_n_rad = guidance.bank_command_n_rad;
    diagnostics.guidance_bank_filtered_n_rad = guidance_filter.bank_filtered_n_rad;
    static_cast<void>(
        bank_angle_from(state, environment, diagnostics.guidance_bank_response_n_rad));
    diagnostics.guidance_state_index = guidance.execution.state_index;
    diagnostics.guidance_reference_index = guidance.diagnostics.reference_index;
    diagnostics.guidance_reference_position_valid = guidance.diagnostics.reference_position_valid;
    diagnostics.guidance_active = guidance.execution.guidance_active;
    diagnostics.pad_constraint_active = guidance.execution.pad_constraint_active;

    diagnostics.autopilot_q_command_b2i = autopilot.q_command_b2i;
    diagnostics.autopilot_q_response_b2i = state.q_b2i;
    diagnostics.autopilot_angular_rate_command_b_radps = autopilot.w_command_ib_b_radps;
    diagnostics.autopilot_angular_rate_feedforward_b_radps = autopilot.w_feedforward_ib_b_radps;
    diagnostics.autopilot_angular_rate_controller_response_b_radps =
        autopilot.w_controller_response_ib_b_radps;
    diagnostics.autopilot_gyro_observation_b_radps = autopilot.gyro_observation_ib_b_radps;
    diagnostics.autopilot_active = autopilot.active;

    diagnostics.vehicle_velocity_ib_b_mps = state.q_b2i.conjugate() * state.v_i_mps;
    diagnostics.vehicle_acceleration_ib_b_mps2 = state.q_b2i.conjugate() * state.a_i_mps2;
    const Eigen::Quaternion<core::Scalar_t> q_b2e = core::math::normalized_with_positive_scalar(
        Eigen::Quaternion<core::Scalar_t>{environment.C_i2e} * state.q_b2i);
    diagnostics.vehicle_velocity_eb_b_mps = q_b2e.conjugate() * environment.v_eb_e_mps;
    diagnostics.vehicle_acceleration_eb_b_mps2 = q_b2e.conjugate() * environment.a_eb_e_mps2;
    diagnostics.vehicle_specific_force_command_b_mps2 =
        vehicle_command.specific_force_command_ib_b_mps2;
    diagnostics.vehicle_specific_force_command_response_b_mps2 =
        response.specific_force_command_response_ib_b_mps2;
    diagnostics.vehicle_specific_force_response_b_mps2 = response.specific_force_ib_b_mps2;
    diagnostics.vehicle_angular_rate_command_b_radps = vehicle_command.w_command_ib_b_radps;
    diagnostics.vehicle_angular_rate_response_b_radps = response.w_ib_b_radps;

    diagnostics.velocity_tracking_error_b_mps =
        state.q_b2i.conjugate() * (guidance.diagnostics.v_reference_i_mps - state.v_i_mps);
    diagnostics.acceleration_tracking_error_b_mps2 =
        state.q_b2i.conjugate() * (guidance.diagnostics.a_cmd_i_mps2 - state.a_i_mps2);
    diagnostics.attitude_tracking_error_b_rad =
        core::math::rotvec_rad_from_quaternion(core::math::normalized_with_positive_scalar(
            state.q_b2i.conjugate() * autopilot.q_command_b2i));
    diagnostics.angular_rate_tracking_error_b_radps =
        autopilot.w_command_ib_b_radps - state.w_ib_b_radps;
    diagnostics.specific_force_tracking_error_b_mps2 =
        vehicle_command.specific_force_command_ib_b_mps2 - state.specific_force_ib_b_mps2;
    diagnostics.angular_rate_limited = response.angular_rate_limited;
    diagnostics.specific_force_limited = response.specific_force_limited;
    return diagnostics;
}

} // namespace

class GeneratedTrajectorySource::Implementation
{
public:
    Implementation(const core::RationalRate rate,
                   const core::RationalRate guidance_rate,
                   const core::RationalRate autopilot_rate,
                   const core::Timestamp t_start,
                   const core::Timestamp t_end,
                   const TranslationalIntegrationMethod integration_method,
                   TrajectoryInitialCondition initial_condition,
                   std::unique_ptr<GuidanceModel> guidance,
                   std::unique_ptr<AutopilotModel> autopilot,
                   std::unique_ptr<VehicleResponseModel> vehicle_response,
                   const TrajectoryTerminationMode termination_mode)
        : m_initial_condition(std::move(initial_condition))
        , m_guidance(std::move(guidance))
        , m_autopilot(std::move(autopilot))
        , m_vehicle_response(std::move(vehicle_response))
        , m_rate(rate)
        , m_guidance_rate(guidance_rate)
        , m_autopilot_rate(autopilot_rate)
        , m_t_start(t_start)
        , m_t_end(t_end)
        , m_integration_method(integration_method)
        , m_termination_mode(termination_mode)
    {}

    [[nodiscard]] bool initialize()
    {
        if (m_initialized || !core::rational_cadence_is_valid(m_t_start, m_rate) ||
            !core::rational_rate_is_integer_multiple(m_rate, m_autopilot_rate) ||
            !core::rational_rate_is_integer_multiple(m_autopilot_rate, m_guidance_rate) ||
            !core::timestamp_is_valid(m_t_end) || m_t_end.scale != m_t_start.scale ||
            !core::timestamp_less(m_t_start, m_t_end) || !m_guidance || !m_autopilot ||
            !m_vehicle_response || !m_guidance_schedule.initialize(m_t_start, m_guidance_rate) ||
            !m_autopilot_schedule.initialize(m_t_start, m_autopilot_rate) ||
            !constrained_state_at(m_initial_condition, m_t_start, m_t_start, m_state)) {
            return false;
        }
        core::Vec3 initial_lla_deg_m{};
        if (!core::frames::fixed_m_to_lla_deg_m<Planet>(m_initial_condition.p_e_m,
                                                        initial_lla_deg_m)) {
            return false;
        }
        m_ground_reference_height_m = initial_lla_deg_m.z();

        if (!m_vehicle_response->initialize(m_state)) {
            return false;
        }

        const core::Time_t physics_dt_s =
            static_cast<core::Time_t>(m_rate.s) / static_cast<core::Time_t>(m_rate.samples);
        if (!m_guidance_schedule.due(m_t_start) || !m_autopilot_schedule.due(m_t_start) ||
            !initialize_control_models(control_state_from(m_state), physics_dt_s)) {
            return false;
        }
        m_pad_constraint_was_active = m_guidance_output.execution.pad_constraint_active;

        m_vehicle_output.w_ib_b_radps = m_state.w_ib_b_radps;
        m_vehicle_output.specific_force_ib_b_mps2 = m_state.specific_force_ib_b_mps2;
        m_vehicle_output.specific_force_command_response_ib_b_mps2 =
            m_state.specific_force_ib_b_mps2;
        m_vehicle_output.a_i_mps2 = m_state.a_i_mps2;

        TrajectoryEnvironment plant_environment{};
        TruthSample initial_truth{};
        if (!environment_at(control_state_from(m_state), m_t_start, plant_environment) ||
            !truth_from_state(m_state, m_t_start, initial_truth) ||
            !m_truth.append(initial_truth,
                            diagnostics_from(m_state,
                                             plant_environment,
                                             m_guidance_output,
                                             m_guidance_filter_output,
                                             m_autopilot_output,
                                             m_vehicle_command,
                                             m_vehicle_output))) {
            return false;
        }

        m_last_guidance_t = m_t_start;
        m_last_autopilot_t = m_t_start;
        m_t_available = m_t_start;
        m_next_sample_index = 1U;
        m_initialized = true;
        return true;
    }

    [[nodiscard]] bool initialize_control_models(const TrajectoryControlState& control_state,
                                                 const core::Time_t physics_dt_s)
    {
        TrajectoryEnvironment control_environment{};
        TrajectoryEnvironment plant_environment{};
        if (!environment_at(control_state, m_t_start, control_environment) ||
            !environment_at(control_state_from(m_state), m_t_start, plant_environment) ||
            !m_guidance->initialize(control_state, control_environment) ||
            !m_autopilot->initialize(autopilot_state_from(control_state, control_environment)) ||
            !m_guidance->advance(
                control_state, control_environment, physics_dt_s, m_guidance_output)) {
            return false;
        }

        GuidanceCommand raw_guidance_command{};
        core::Scalar_t initial_bank_n_rad{};
        if (!guidance_command_from(
                m_guidance_output, m_state, plant_environment, raw_guidance_command) ||
            !bank_angle_from(m_state, plant_environment, initial_bank_n_rad) ||
            !m_guidance_filter.initialize(m_state.specific_force_ib_b_mps2,
                                          initial_bank_n_rad,
                                          m_guidance_output.execution.filter_config) ||
            !m_guidance_filter.advance(raw_guidance_command.specific_force_command_ib_b_mps2,
                                       raw_guidance_command.bank_command_n_rad,
                                       m_guidance_output.execution.filter_config,
                                       m_guidance_output.execution.filter_on_entry,
                                       m_guidance_output.execution.state_entered,
                                       physics_dt_s,
                                       m_guidance_filter_output)) {
            return false;
        }
        m_filtered_guidance_command.bank_command_n_rad =
            m_guidance_filter_output.bank_filtered_n_rad;
        m_filtered_guidance_command.specific_force_command_ib_b_mps2 =
            m_guidance_filter_output.specific_force_filtered_ib_b_mps2;

        if (!m_autopilot->advance(m_filtered_guidance_command,
                                  autopilot_state_from(control_state, control_environment),
                                  autopilot_execution_from(m_guidance_output.execution),
                                  physics_dt_s,
                                  m_vehicle_command,
                                  m_autopilot_output)) {
            return false;
        }
        return true;
    }

    [[nodiscard]] bool advance_guidance_filter(const TrajectoryEnvironment& plant_environment,
                                               const core::Time_t guidance_dt_s)
    {
        GuidanceCommand raw_guidance_command{};
        if (!guidance_command_from(
                m_guidance_output, m_state, plant_environment, raw_guidance_command) ||
            !m_guidance_filter.advance(raw_guidance_command.specific_force_command_ib_b_mps2,
                                       raw_guidance_command.bank_command_n_rad,
                                       m_guidance_output.execution.filter_config,
                                       m_guidance_output.execution.filter_on_entry,
                                       m_guidance_output.execution.state_entered,
                                       guidance_dt_s,
                                       m_guidance_filter_output)) {
            return false;
        }
        m_filtered_guidance_command.bank_command_n_rad =
            m_guidance_filter_output.bank_filtered_n_rad;
        m_filtered_guidance_command.specific_force_command_ib_b_mps2 =
            m_guidance_filter_output.specific_force_filtered_ib_b_mps2;
        return true;
    }

    [[nodiscard]] bool replace_initial_control_state(const TrajectoryControlState& control_state)
    {
        const core::Time_t physics_dt_s =
            static_cast<core::Time_t>(m_rate.s) / static_cast<core::Time_t>(m_rate.samples);
        TrajectoryEnvironment plant_environment{};
        return initialize_control_models(control_state, physics_dt_s) &&
               environment_at(control_state_from(m_state), m_t_start, plant_environment) &&
               m_truth.update_last_diagnostics(diagnostics_from(m_state,
                                                                plant_environment,
                                                                m_guidance_output,
                                                                m_guidance_filter_output,
                                                                m_autopilot_output,
                                                                m_vehicle_command,
                                                                m_vehicle_output));
    }

    [[nodiscard]] TrajectoryControlState selected_control_state() const
    {
        if (m_external_control_state_valid &&
            !core::timestamp_less(m_state.t, m_external_control_state.t)) {
            return m_external_control_state;
        }
        return control_state_from(m_state);
    }

    [[nodiscard]] bool advance_native()
    {
        if (!m_initialized || !core::timestamp_less(m_state.t, m_t_end)) {
            return false;
        }

        core::Timestamp t_next{};
        if (!core::timestamp_at_sample_index(m_t_start, m_rate, m_next_sample_index, t_next) ||
            core::timestamp_less(m_t_end, t_next)) {
            return false;
        }
        const core::Time_t physics_dt_s = elapsed_seconds(m_state.t, t_next);
        TrajectoryControlState control_state = selected_control_state();
        TrajectoryEnvironment control_environment{};
        if (physics_dt_s <= 0.0 || !environment_at(control_state, m_t_start, control_environment)) {
            return false;
        }

        if (m_guidance_schedule.due(m_state.t)) {
            const core::Time_t guidance_dt_s = elapsed_seconds(m_last_guidance_t, m_state.t);
            TrajectoryEnvironment guidance_plant_environment{};
            if (guidance_dt_s <= 0.0 ||
                !m_guidance->advance(
                    control_state, control_environment, guidance_dt_s, m_guidance_output) ||
                !environment_at(
                    control_state_from(m_state), m_t_start, guidance_plant_environment) ||
                !advance_guidance_filter(guidance_plant_environment, guidance_dt_s)) {
                return false;
            }
            m_last_guidance_t = m_state.t;
        }
        if (m_autopilot_schedule.due(m_state.t)) {
            const core::Time_t autopilot_dt_s = elapsed_seconds(m_last_autopilot_t, m_state.t);
            if (autopilot_dt_s <= 0.0 ||
                !m_autopilot->advance(m_filtered_guidance_command,
                                      autopilot_state_from(control_state, control_environment),
                                      autopilot_execution_from(m_guidance_output.execution),
                                      autopilot_dt_s,
                                      m_vehicle_command,
                                      m_autopilot_output)) {
                return false;
            }
            m_last_autopilot_t = m_state.t;
        }

        const bool release_overlap =
            !m_guidance_output.execution.pad_constraint_active && m_pad_constraint_was_active;
        if (!m_guidance_output.execution.pad_constraint_active) {
            TrajectoryEnvironment plant_environment{};
            if (!environment_at(control_state_from(m_state), m_t_start, plant_environment)) {
                return false;
            }
            if (!m_vehicle_response->advance(m_vehicle_command,
                                             m_state,
                                             plant_environment,
                                             physics_dt_s,
                                             m_vehicle_output)) {
                return false;
            }
        }

        if (m_guidance_output.execution.pad_constraint_active || release_overlap) {
            if (!constrained_state_at(m_initial_condition, m_t_start, t_next, m_state)) {
                return false;
            }
            if (!release_overlap) {
                m_vehicle_output.w_ib_b_radps = m_state.w_ib_b_radps;
                m_vehicle_output.specific_force_ib_b_mps2 = m_state.specific_force_ib_b_mps2;
                m_vehicle_output.specific_force_command_response_ib_b_mps2 =
                    m_state.specific_force_ib_b_mps2;
                m_vehicle_output.a_i_mps2 = m_state.a_i_mps2;
            }
        }
        else {
            const core::Vec3 previous_rate_radps = m_state.w_ib_b_radps;
            const core::Vec3 previous_acceleration_mps2 = m_state.a_i_mps2;
            if (!integrate_attitude_eci(previous_rate_radps,
                                        m_vehicle_output.w_ib_b_radps,
                                        physics_dt_s,
                                        m_state.q_b2i)) {
                return false;
            }

            TrajectoryDynamicState endpoint_state = m_state;
            endpoint_state.t = t_next;
            endpoint_state.p_i_m +=
                (endpoint_state.v_i_mps * physics_dt_s) +
                (0.5 * previous_acceleration_mps2 * physics_dt_s * physics_dt_s);
            TrajectoryEnvironment endpoint_environment{};
            if (!environment_at(
                    control_state_from(endpoint_state), m_t_start, endpoint_environment)) {
                return false;
            }
            m_vehicle_output.a_i_mps2 =
                (m_state.q_b2i * m_vehicle_output.specific_force_ib_b_mps2) +
                endpoint_environment.gravity_i_mps2;
            if (!integrate_translation_eci(previous_acceleration_mps2,
                                           m_vehicle_output.a_i_mps2,
                                           physics_dt_s,
                                           m_integration_method,
                                           m_state.p_i_m,
                                           m_state.v_i_mps)) {
                return false;
            }
            m_state.t = t_next;
            m_state.a_i_mps2 = m_vehicle_output.a_i_mps2;
            m_state.w_ib_b_radps = m_vehicle_output.w_ib_b_radps;
            m_state.specific_force_ib_b_mps2 = m_vehicle_output.specific_force_ib_b_mps2;
        }
        m_pad_constraint_was_active = m_guidance_output.execution.pad_constraint_active;

        TrajectoryEnvironment log_environment{};
        TruthSample sample{};
        core::Vec3 current_lla_deg_m{};
        if (!environment_at(control_state_from(m_state), m_t_start, log_environment) ||
            !core::frames::fixed_m_to_lla_deg_m<Planet>(log_environment.p_e_m, current_lla_deg_m) ||
            !truth_from_state(m_state, m_t_start, sample) ||
            !m_truth.append(sample,
                            diagnostics_from(m_state,
                                             log_environment,
                                             m_guidance_output,
                                             m_guidance_filter_output,
                                             m_autopilot_output,
                                             m_vehicle_command,
                                             m_vehicle_output))) {
            return false;
        }
        if (m_termination_mode == TrajectoryTerminationMode::GroundImpact) {
            constexpr core::Scalar_t departure_height_tolerance_m = 0.1;
            m_departed_ground = m_departed_ground ||
                                current_lla_deg_m.z() >
                                    (m_ground_reference_height_m + departure_height_tolerance_m);
            if (m_departed_ground && current_lla_deg_m.z() <= m_ground_reference_height_m) {
                m_t_end = t_next;
                m_complete = true;
            }
        }
        ++m_next_sample_index;
        return true;
    }

    TrajectoryInitialCondition m_initial_condition{};
    AutopilotOutput m_autopilot_output{};
    TrajectoryControlState m_external_control_state{};
    TrajectoryDynamicState m_state{};
    std::unique_ptr<GuidanceModel> m_guidance;
    std::unique_ptr<AutopilotModel> m_autopilot;
    std::unique_ptr<VehicleResponseModel> m_vehicle_response;
    GuidanceCommandFilter m_guidance_filter;
    core::SampleIndex m_next_sample_index{};
    core::RationalRate m_rate{};
    core::RationalRate m_guidance_rate{};
    core::RationalRate m_autopilot_rate{};
    core::Timestamp m_t_start{};
    core::Timestamp m_t_end{};
    core::Timestamp m_last_guidance_t{};
    core::Timestamp m_last_autopilot_t{};
    core::Timestamp m_t_available{};
    VehicleCommand m_vehicle_command{};
    core::RationalSchedule m_guidance_schedule;
    core::RationalSchedule m_autopilot_schedule;
    TruthTrajectory m_truth;
    VehicleResponseOutput m_vehicle_output{};
    GuidanceOutput m_guidance_output{};
    GuidanceCommand m_filtered_guidance_command{};
    GuidanceCommandFilterOutput m_guidance_filter_output{};
    TranslationalIntegrationMethod m_integration_method{};
    TrajectoryTerminationMode m_termination_mode{TrajectoryTerminationMode::ConfiguredDuration};
    core::Scalar_t m_ground_reference_height_m{};
    bool m_external_control_state_valid{false};
    bool m_pad_constraint_was_active{false};
    bool m_departed_ground{false};
    bool m_initialized{false};
    bool m_complete{false};
};

GeneratedTrajectorySource::GeneratedTrajectorySource(
    const core::RationalRate rate,
    const core::RationalRate guidance_rate,
    const core::RationalRate autopilot_rate,
    const core::Timestamp t_start,
    const core::Timestamp t_end,
    const TranslationalIntegrationMethod integration_method,
    TrajectoryInitialCondition initial_condition,
    std::unique_ptr<GuidanceModel> guidance,
    std::unique_ptr<AutopilotModel> autopilot,
    std::unique_ptr<VehicleResponseModel> vehicle_response,
    const TrajectoryTerminationMode termination_mode)
    : m_impl(std::make_unique<Implementation>(rate,
                                              guidance_rate,
                                              autopilot_rate,
                                              t_start,
                                              t_end,
                                              integration_method,
                                              std::move(initial_condition),
                                              std::move(guidance),
                                              std::move(autopilot),
                                              std::move(vehicle_response),
                                              termination_mode))
{}

GeneratedTrajectorySource::~GeneratedTrajectorySource() = default;
GeneratedTrajectorySource::GeneratedTrajectorySource(GeneratedTrajectorySource&&) noexcept =
    default;
GeneratedTrajectorySource&
GeneratedTrajectorySource::operator=(GeneratedTrajectorySource&&) noexcept = default;

bool GeneratedTrajectorySource::initialize()
{
    return m_impl && m_impl->initialize();
}

bool GeneratedTrajectorySource::advance_to(const core::Timestamp& t)
{
    if (!m_impl || !m_impl->m_initialized || !core::timestamp_is_valid(t) ||
        t.scale != m_impl->m_t_start.scale || core::timestamp_less(t, m_impl->m_t_start) ||
        core::timestamp_less(m_impl->m_t_end, t)) {
        if (m_impl && core::timestamp_less(m_impl->m_t_end, t)) {
            m_impl->m_complete = true;
        }
        return false;
    }
    while (core::timestamp_less(m_impl->m_truth.last().t, t)) {
        if (!m_impl->advance_native()) {
            return false;
        }
        if (m_impl->m_complete) {
            m_impl->m_t_available = m_impl->m_t_end;
            return true;
        }
    }
    m_impl->m_t_available = t;
    m_impl->m_complete = (t == m_impl->m_t_end);
    return true;
}

bool GeneratedTrajectorySource::query(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    return m_impl && m_impl->m_initialized && !core::timestamp_less(m_impl->m_t_available, t) &&
           m_impl->m_truth.sample_at(t, sample);
}

bool GeneratedTrajectorySource::query_diagnostics(const core::Timestamp& t,
                                                  TrajectoryDiagnostics& diagnostics) const
{
    diagnostics = {};
    return m_impl && m_impl->m_initialized && !core::timestamp_less(m_impl->m_t_available, t) &&
           m_impl->m_truth.diagnostics_at(t, diagnostics);
}

bool GeneratedTrajectorySource::set_control_state(const TrajectoryControlState& state)
{
    if (!m_impl || !m_impl->m_initialized || !control_state_is_valid(state) ||
        state.t.scale != m_impl->m_t_start.scale ||
        core::timestamp_less(state.t, m_impl->m_t_start) ||
        core::timestamp_less(m_impl->m_t_end, state.t)) {
        return false;
    }
    m_impl->m_external_control_state = state;
    m_impl->m_external_control_state.q_b2i =
        core::math::normalized_with_positive_scalar(state.q_b2i);
    m_impl->m_external_control_state_valid = true;
    if (state.t == m_impl->m_t_start && m_impl->m_state.t == m_impl->m_t_start &&
        m_impl->m_next_sample_index == 1U &&
        !m_impl->replace_initial_control_state(m_impl->m_external_control_state)) {
        m_impl->m_external_control_state_valid = false;
        return false;
    }
    return true;
}

bool GeneratedTrajectorySource::observe_imu_increment(
    const core::estimation::ImuIncrement& increment)
{
    return m_impl && m_impl->m_initialized && m_impl->m_autopilot->observe_imu_increment(increment);
}

core::Timestamp GeneratedTrajectorySource::t_start() const
{
    return m_impl ? m_impl->m_t_start : core::Timestamp{};
}

core::Timestamp GeneratedTrajectorySource::t_end() const
{
    return m_impl ? m_impl->m_t_end : core::Timestamp{};
}

bool GeneratedTrajectorySource::is_complete() const
{
    return m_impl && m_impl->m_complete;
}

} // namespace navkit::sim
