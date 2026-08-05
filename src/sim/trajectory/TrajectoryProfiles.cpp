// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/TrajectoryProfiles.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/sim/guidance/GuidanceAlgorithms.hpp"
#include "navkit/sim/guidance/GuidanceBlocks.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"
#include "navkit/sim/trajectory/GeneratedTrajectorySource.hpp"

#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace navkit::sim
{

namespace
{

using Planet = core::environment::Wgs84;

constexpr core::Scalar_t maximum_supported_bank_angle_rad =
    std::numbers::pi_v<core::Scalar_t> / 3.0;

[[nodiscard]] bool nonnegative_finite(const core::Vec3& value)
{
    return value.allFinite() && (value.array() >= 0.0).all();
}

[[nodiscard]] bool profile_is_valid(const TrajectoryProfileConfig& cfg)
{
    return cfg.duration_s > 0.0 && core::rational_cadence_is_valid(cfg.t_epoch, cfg.rate) &&
           core::rational_cadence_is_valid(cfg.t_epoch, cfg.guidance_rate) &&
           core::rational_cadence_is_valid(cfg.t_epoch, cfg.autopilot_rate) &&
           core::rational_rate_is_integer_multiple(cfg.rate, cfg.autopilot_rate) &&
           core::rational_rate_is_integer_multiple(cfg.autopilot_rate, cfg.guidance_rate) &&
           cfg.p_e_m.allFinite() && cfg.p_e_m.norm() > 0.0 && cfg.v_e_mps.allFinite() &&
           cfg.q_b2e.coeffs().allFinite() && cfg.q_b2e.norm() > 0.0 &&
           std::isfinite(cfg.maximum_bank_angle_rad) && cfg.maximum_bank_angle_rad > 0.0 &&
           cfg.maximum_bank_angle_rad <= maximum_supported_bank_angle_rad &&
           guidance_command_filter_config_is_valid(cfg.guidance_command_filter) &&
           first_order_autopilot_config_is_valid(cfg.autopilot) &&
           first_order_vehicle_response_config_is_valid(cfg.vehicle_response);
}

[[nodiscard]] bool timestamp_and_elapsed(const TrajectoryProfileConfig& cfg,
                                         const core::SampleIndex sample_index,
                                         core::Timestamp& t,
                                         core::Time_t& elapsed_s)
{
    if (!core::timestamp_at_sample_index(cfg.t_epoch, cfg.rate, sample_index, t)) {
        return false;
    }
    core::Duration elapsed{};
    if (!core::elapsed_time(t, cfg.t_epoch, elapsed)) {
        return false;
    }
    elapsed_s = core::duration_seconds(elapsed);
    return elapsed_s <= (cfg.duration_s + 1.0e-12);
}

[[nodiscard]] core::Time_t elapsed_seconds(const core::Timestamp& first,
                                           const core::Timestamp& second)
{
    core::Duration elapsed{};
    if (!core::elapsed_time(second, first, elapsed)) {
        return 0.0;
    }
    return core::duration_seconds(elapsed);
}

[[nodiscard]] bool local_rpy_from_attitude(const core::Vec3& p_e_m,
                                           const Eigen::Quaternion<core::Scalar_t>& q_b2e,
                                           core::Vec3& rpy_b2n_rad)
{
    core::Mat3 c_e2n{};
    if (!core::frames::ecef_to_ned_matrix(p_e_m, c_e2n)) {
        return false;
    }
    const Eigen::Quaternion<core::Scalar_t> q_e2n{c_e2n};
    rpy_b2n_rad = core::math::rpy_rad_from_quaternion(
        core::math::normalized_with_positive_scalar(q_e2n * q_b2e));
    return rpy_b2n_rad.allFinite();
}

[[nodiscard]] bool initial_condition_for_speed(const TrajectoryProfileConfig& profile,
                                               const core::Scalar_t speed_mps,
                                               TrajectoryInitialCondition& initial)
{
    core::Mat3 c_e2n{};
    core::Vec3 rpy_b2n_rad{};
    if (!core::frames::ecef_to_ned_matrix(profile.p_e_m, c_e2n) ||
        !local_rpy_from_attitude(profile.p_e_m, profile.q_b2e, rpy_b2n_rad)) {
        return false;
    }
    initial.p_e_m = profile.p_e_m;
    initial.v_e_mps =
        profile.initial_velocity_configured
            ? profile.v_e_mps
            : c_e2n.transpose() * guidance::velocity_n_mps(guidance::KinematicReference{
                                      .speed_mps = speed_mps,
                                      .pitch_rad = rpy_b2n_rad.y(),
                                      .heading_rad = rpy_b2n_rad.z(),
                                  });
    initial.q_b2e = core::math::normalized_with_positive_scalar(profile.q_b2e);
    return initial.v_e_mps.allFinite();
}

[[nodiscard]] GuidanceTranslationDefinition
direct_specific_force_translation(const core::Vec3& specific_force_ib_b_mps2)
{
    GuidanceTranslationDefinition translation{};
    translation.reference = std::make_unique<CurrentStateGuidanceReference>();
    translation.acceleration.push_back(std::make_unique<BodySpecificForceGuidanceAcceleration>(
        BodySpecificForceGuidanceAccelerationConfig{
            .specific_force_ib_b_mps2 = specific_force_ib_b_mps2,
        }));
    return translation;
}

void append_path_tracking_acceleration(const core::Vec3& velocity_error_gain_n_1ps,
                                       const core::Scalar_t altitude_error_p_gain_1ps2,
                                       const core::Scalar_t altitude_error_d_gain_1ps,
                                       GuidanceTranslationDefinition& translation)
{
    translation.acceleration.push_back(std::make_unique<PathFeedforwardGuidanceAcceleration>());
    translation.acceleration.push_back(std::make_unique<VelocityTrackingGuidanceAcceleration>(
        VelocityTrackingGuidanceAccelerationConfig{
            .gain_n_1ps = velocity_error_gain_n_1ps,
        }));
    translation.acceleration.push_back(std::make_unique<AltitudeHoldPdGuidanceAcceleration>(
        AltitudeHoldPdGuidanceAccelerationConfig{
            .target_altitude_m = 0.0,
            .proportional_gain_1ps2 = altitude_error_p_gain_1ps2,
            .derivative_gain_1ps = altitude_error_d_gain_1ps,
            .capture_initial_altitude = true,
        }));
}

[[nodiscard]] std::unique_ptr<GuidanceBankPolicy>
guidance_bank_policy(const bool bank_to_turn_enabled, const core::Scalar_t maximum_bank_angle_rad)
{
    if (bank_to_turn_enabled) {
        return std::make_unique<CoordinatedBankToTurnGuidancePolicy>(
            CoordinatedBankToTurnGuidancePolicyConfig{
                .maximum_bank_angle_rad = maximum_bank_angle_rad,
            });
    }
    return std::make_unique<ZeroBankGuidancePolicy>();
}

[[nodiscard]] GuidanceStateMachineDefinition
ballistic_guidance_definition(const BallisticTrajectoryConfig& cfg)
{
    constexpr const char* launch_pad_id = "launch_pad";
    constexpr const char* boost_id = "boost";
    constexpr const char* gravity_turn_id = "gravity_turn";
    constexpr const char* free_inertial_id = "free_inertial";

    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = cfg.launch_pad_duration_s > 0.0 ? launch_pad_id : boost_id;
    definition.cycle_policy = GuidanceCyclePolicy::Reject;
    definition.states.reserve(cfg.launch_pad_duration_s > 0.0 ? 3U : 2U);

    if (cfg.launch_pad_duration_s > 0.0) {
        GuidanceStateDefinition launch_pad{};
        launch_pad.id = launch_pad_id;
        launch_pad.plant_constraint = GuidancePlantConstraint::StaticLaunchPad;
        launch_pad.translation = direct_specific_force_translation(core::Vec3::Zero());
        launch_pad.bank = std::make_unique<ZeroBankGuidancePolicy>();
        launch_pad.guidance_command_filter = cfg.profile.guidance_command_filter;
        launch_pad.transitions.push_back(GuidanceElapsedTransition{
            .to = boost_id,
            .elapsed_in_state_s = cfg.launch_pad_duration_s,
            .priority = 0U,
        });
        launch_pad.guidance_enabled = false;
        launch_pad.autopilot_enabled = false;
        definition.states.push_back(std::move(launch_pad));
    }

    const char* coast_id =
        cfg.coast_mode == BallisticCoastMode::GravityTurn ? gravity_turn_id : free_inertial_id;
    GuidanceStateDefinition boost{};
    boost.id = boost_id;
    boost.translation =
        direct_specific_force_translation(core::Vec3{cfg.boost_acceleration_b_x_mps2, 0.0, 0.0});
    boost.bank = std::make_unique<ZeroBankGuidancePolicy>();
    boost.guidance_command_filter = cfg.profile.guidance_command_filter;
    boost.transitions.push_back(GuidanceElapsedTransition{
        .to = coast_id,
        .elapsed_in_state_s = cfg.boost_duration_s,
        .priority = 0U,
    });
    definition.states.push_back(std::move(boost));

    GuidanceStateDefinition coast{};
    coast.id = coast_id;
    coast.translation = direct_specific_force_translation(core::Vec3::Zero());
    coast.bank = std::make_unique<ZeroBankGuidancePolicy>();
    coast.guidance_command_filter = cfg.profile.guidance_command_filter;
    coast.terminal = true;
    coast.autopilot_enabled = cfg.coast_mode == BallisticCoastMode::GravityTurn;
    definition.states.push_back(std::move(coast));
    return definition;
}

[[nodiscard]] GuidanceStateMachineDefinition
constant_altitude_guidance_definition(const ConstantAltitudeTrajectoryConfig& cfg)
{
    GuidanceStateDefinition state{};
    state.id = "constant_altitude";
    state.translation.reference =
        std::make_unique<ConstantSpeedGuidanceReference>(ConstantSpeedGuidanceReferenceConfig{
            .speed_mps = cfg.speed_mps,
            .heading_rad = 0.0,
            .pitch_rad = 0.0,
            .capture_initial_heading = true,
            .capture_initial_pitch = false,
        });
    append_path_tracking_acceleration(cfg.velocity_error_gain_n_1ps,
                                      cfg.altitude_error_p_gain_1ps2,
                                      cfg.altitude_error_d_gain_1ps,
                                      state.translation);
    state.bank = std::make_unique<ZeroBankGuidancePolicy>();
    state.guidance_command_filter = cfg.profile.guidance_command_filter;
    state.terminal = true;

    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = state.id;
    definition.states.push_back(std::move(state));
    return definition;
}

[[nodiscard]] GuidanceStateMachineDefinition
calibration_guidance_definition(const CalibrationTrajectoryConfig& cfg)
{
    GuidanceStateDefinition state{};
    state.id = "calibration";
    state.translation.reference =
        std::make_unique<ConstantSpeedGuidanceReference>(ConstantSpeedGuidanceReferenceConfig{
            .speed_mps = cfg.speed_mps,
            .heading_rad = 0.0,
            .pitch_rad = 0.0,
            .capture_initial_heading = true,
            .capture_initial_pitch = false,
        });
    if (cfg.maneuver == CalibrationManeuver::HorizontalSTurn ||
        cfg.maneuver == CalibrationManeuver::DutchRoll) {
        state.translation.reference_modifiers.push_back(
            std::make_unique<HorizontalSinusoidalGuidanceReferenceModifier>(
                SinusoidalGuidanceReferenceModifierConfig{
                    .amplitude_rad = cfg.horizontal_amplitude_rad,
                    .frequency_hz = 1.0 / cfg.period_s,
                    .phase_rad = 0.0,
                }));
    }
    if (cfg.maneuver == CalibrationManeuver::VerticalSTurn ||
        cfg.maneuver == CalibrationManeuver::DutchRoll) {
        const bool dutch_roll = cfg.maneuver == CalibrationManeuver::DutchRoll;
        state.translation.reference_modifiers.push_back(
            std::make_unique<VerticalSinusoidalGuidanceReferenceModifier>(
                SinusoidalGuidanceReferenceModifierConfig{
                    .amplitude_rad =
                        dutch_roll ? -cfg.vertical_amplitude_rad : cfg.vertical_amplitude_rad,
                    .frequency_hz = dutch_roll ? 2.0 / cfg.period_s : 1.0 / cfg.period_s,
                    .phase_rad = 0.0,
                }));
    }
    append_path_tracking_acceleration(cfg.velocity_error_gain_n_1ps,
                                      cfg.altitude_error_p_gain_1ps2,
                                      cfg.altitude_error_d_gain_1ps,
                                      state.translation);
    state.bank = guidance_bank_policy(cfg.bank_to_turn_enabled, cfg.profile.maximum_bank_angle_rad);
    state.guidance_command_filter = cfg.profile.guidance_command_filter;
    state.terminal = true;
    state.body_y_specific_force_enabled = cfg.body_y_specific_force_enabled;

    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = state.id;
    definition.states.push_back(std::move(state));
    return definition;
}

[[nodiscard]] GuidanceStateMachineDefinition
waypoint_guidance_definition(const WaypointTrajectoryConfig& cfg)
{
    GuidanceStateDefinition state{};
    state.id = "waypoint";
    state.translation.reference =
        std::make_unique<WaypointGuidanceReference>(WaypointGuidanceReferenceConfig{
            .waypoint_e_m = cfg.waypoint_e_m,
            .speed_mps = cfg.speed_mps,
            .acceptance_radius_m = cfg.acceptance_radius_m,
            .heading_error_gain_mps2_per_rad = cfg.heading_error_gain_mps2_per_rad,
        });
    append_path_tracking_acceleration(cfg.velocity_error_gain_n_1ps,
                                      cfg.altitude_error_p_gain_1ps2,
                                      cfg.altitude_error_d_gain_1ps,
                                      state.translation);
    state.bank = guidance_bank_policy(true, cfg.profile.maximum_bank_angle_rad);
    state.guidance_command_filter = cfg.profile.guidance_command_filter;
    state.terminal = true;
    state.body_y_specific_force_enabled = cfg.body_y_specific_force_enabled;

    GuidanceStateMachineDefinition definition{};
    definition.initial_state_id = state.id;
    definition.states.push_back(std::move(state));
    return definition;
}

[[nodiscard]] bool end_timestamp(const TrajectoryProfileConfig& profile, core::Timestamp& t_end)
{
    t_end = {};
    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp candidate{};
        core::Time_t elapsed_s{};
        if (!timestamp_and_elapsed(profile, index, candidate, elapsed_s)) {
            break;
        }
        t_end = candidate;
    }
    return core::timestamp_is_valid(t_end) && core::timestamp_less(profile.t_epoch, t_end);
}

[[nodiscard]] std::unique_ptr<TrajectorySource>
generated_source(const TrajectoryProfileConfig& profile,
                 TrajectoryInitialCondition initial_condition,
                 std::unique_ptr<GuidanceModel> guidance,
                 const TrajectoryTerminationMode termination_mode =
                     TrajectoryTerminationMode::ConfiguredDuration)
{
    core::Timestamp t_end{};
    if (!guidance || !end_timestamp(profile, t_end)) {
        return {};
    }
    std::unique_ptr<AutopilotModel> autopilot =
        make_autopilot_model(profile.autopilot_model, profile.autopilot);
    std::unique_ptr<VehicleResponseModel> vehicle_response =
        make_vehicle_response_model(profile.vehicle_response_model, profile.vehicle_response);
    if (!autopilot || !vehicle_response) {
        return {};
    }
    std::unique_ptr<GeneratedTrajectorySource> source =
        std::make_unique<GeneratedTrajectorySource>(profile.rate,
                                                    profile.guidance_rate,
                                                    profile.autopilot_rate,
                                                    profile.t_epoch,
                                                    t_end,
                                                    profile.translational_integration,
                                                    std::move(initial_condition),
                                                    std::move(guidance),
                                                    std::move(autopilot),
                                                    std::move(vehicle_response),
                                                    termination_mode);
    if (!source->initialize()) {
        return {};
    }
    return source;
}

[[nodiscard]] TruthTrajectory materialize(std::unique_ptr<TrajectorySource> source,
                                          const TrajectoryProfileConfig& profile)
{
    if (!source) {
        return {};
    }
    std::vector<TruthSample> samples{};
    std::vector<TrajectoryDiagnostics> diagnostics{};
    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp t{};
        core::Time_t elapsed_s{};
        if (!timestamp_and_elapsed(profile, index, t, elapsed_s)) {
            break;
        }
        TruthSample sample{};
        TrajectoryDiagnostics value{};
        if (!source->advance_to(t) || !source->query(t, sample) ||
            !source->query_diagnostics(t, value)) {
            return {};
        }
        samples.push_back(sample);
        diagnostics.push_back(value);
        if (source->is_complete()) {
            break;
        }
    }
    return TruthTrajectory{std::move(samples), std::move(diagnostics)};
}

} // namespace

void populate_truth_angular_rates(std::vector<TruthSample>& samples)
{
    if (samples.empty()) {
        return;
    }
    const core::Vec3 planet_rate_e_radps = core::environment::planet_rate_fixed_radps<Planet>();
    if (samples.size() == 1U) {
        samples.front().w_ib_b_radps = samples.front().q_b2e.conjugate() * planet_rate_e_radps;
        return;
    }

    for (std::size_t index = 0U; index < samples.size(); ++index) {
        const std::size_t first_index = index + 1U < samples.size() ? index : index - 1U;
        const std::size_t second_index = index + 1U < samples.size() ? index + 1U : index;
        const TruthSample& first = samples.at(first_index);
        const TruthSample& second = samples.at(second_index);
        const core::Time_t dt_s = elapsed_seconds(first.t, second.t);
        if (dt_s <= 0.0) {
            samples.at(index).w_ib_b_radps = {};
            continue;
        }
        const Eigen::Quaternion<core::Scalar_t> q_mid =
            core::math::normalized_with_positive_scalar(first.q_b2e.slerp(0.5, second.q_b2e));
        const Eigen::Quaternion<core::Scalar_t> q_body_relative =
            first.q_b2e.conjugate() * second.q_b2e;
        samples.at(index).w_ib_b_radps =
            (core::math::rotvec_rad_from_quaternion(q_body_relative) / dt_s) +
            (q_mid.conjugate() * planet_rate_e_radps);
    }
}

std::unique_ptr<TrajectorySource> ballistic_trajectory_source(const BallisticTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.launch_pad_duration_s < 0.0 ||
        cfg.boost_duration_s <= 0.0 || cfg.boost_acceleration_b_x_mps2 <= 0.0) {
        return {};
    }
    StateMachineTrajectoryConfig state_machine_config{
        .profile = cfg.profile,
        .state_machine = ballistic_guidance_definition(cfg),
        .termination_mode = TrajectoryTerminationMode::GroundImpact,
    };
    state_machine_config.profile.initial_velocity_configured = true;
    return state_machine_trajectory_source(std::move(state_machine_config));
}

std::unique_ptr<TrajectorySource>
constant_altitude_trajectory_source(const ConstantAltitudeTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.speed_mps <= 0.0 ||
        !nonnegative_finite(cfg.velocity_error_gain_n_1ps) ||
        !std::isfinite(cfg.altitude_error_p_gain_1ps2) || cfg.altitude_error_p_gain_1ps2 < 0.0 ||
        !std::isfinite(cfg.altitude_error_d_gain_1ps) || cfg.altitude_error_d_gain_1ps < 0.0) {
        return {};
    }
    TrajectoryInitialCondition initial{};
    if (!initial_condition_for_speed(cfg.profile, cfg.speed_mps, initial)) {
        return {};
    }
    StateMachineTrajectoryConfig state_machine_config{
        .profile = cfg.profile,
        .state_machine = constant_altitude_guidance_definition(cfg),
    };
    state_machine_config.profile.v_e_mps = initial.v_e_mps;
    state_machine_config.profile.initial_velocity_configured = true;
    return state_machine_trajectory_source(std::move(state_machine_config));
}

std::unique_ptr<TrajectorySource>
calibration_trajectory_source(const CalibrationTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || !std::isfinite(cfg.speed_mps) || cfg.speed_mps <= 0.0 ||
        ((cfg.maneuver == CalibrationManeuver::HorizontalSTurn ||
          cfg.maneuver == CalibrationManeuver::DutchRoll) &&
         (!std::isfinite(cfg.horizontal_amplitude_rad) || cfg.horizontal_amplitude_rad <= 0.0)) ||
        ((cfg.maneuver == CalibrationManeuver::VerticalSTurn ||
          cfg.maneuver == CalibrationManeuver::DutchRoll) &&
         (!std::isfinite(cfg.vertical_amplitude_rad) || cfg.vertical_amplitude_rad <= 0.0)) ||
        !std::isfinite(cfg.period_s) || cfg.period_s <= 0.0 ||
        !nonnegative_finite(cfg.velocity_error_gain_n_1ps) ||
        !std::isfinite(cfg.altitude_error_p_gain_1ps2) || cfg.altitude_error_p_gain_1ps2 < 0.0 ||
        !std::isfinite(cfg.altitude_error_d_gain_1ps) || cfg.altitude_error_d_gain_1ps < 0.0) {
        return {};
    }
    TrajectoryInitialCondition initial{};
    if (!initial_condition_for_speed(cfg.profile, cfg.speed_mps, initial)) {
        return {};
    }
    StateMachineTrajectoryConfig state_machine_config{
        .profile = cfg.profile,
        .state_machine = calibration_guidance_definition(cfg),
    };
    state_machine_config.profile.v_e_mps = initial.v_e_mps;
    state_machine_config.profile.initial_velocity_configured = true;
    return state_machine_trajectory_source(std::move(state_machine_config));
}

std::unique_ptr<TrajectorySource> waypoint_trajectory_source(const WaypointTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.waypoint_e_m.empty() || cfg.speed_mps <= 0.0 ||
        cfg.acceptance_radius_m <= 0.0 || !nonnegative_finite(cfg.velocity_error_gain_n_1ps) ||
        !std::isfinite(cfg.altitude_error_p_gain_1ps2) || cfg.altitude_error_p_gain_1ps2 < 0.0 ||
        !std::isfinite(cfg.altitude_error_d_gain_1ps) || cfg.altitude_error_d_gain_1ps < 0.0 ||
        !std::isfinite(cfg.heading_error_gain_mps2_per_rad) ||
        cfg.heading_error_gain_mps2_per_rad < 0.0) {
        return {};
    }
    TrajectoryInitialCondition initial{};
    if (!initial_condition_for_speed(cfg.profile, cfg.speed_mps, initial)) {
        return {};
    }
    StateMachineTrajectoryConfig state_machine_config{
        .profile = cfg.profile,
        .state_machine = waypoint_guidance_definition(cfg),
    };
    state_machine_config.profile.v_e_mps = initial.v_e_mps;
    state_machine_config.profile.initial_velocity_configured = true;
    return state_machine_trajectory_source(std::move(state_machine_config));
}

std::unique_ptr<TrajectorySource> state_machine_trajectory_source(StateMachineTrajectoryConfig cfg)
{
    std::string validation_diagnostic{};
    if (!profile_is_valid(cfg.profile) || !cfg.profile.initial_velocity_configured ||
        !guidance_state_machine_definition_is_valid(cfg.state_machine, validation_diagnostic)) {
        return {};
    }

    TrajectoryInitialCondition initial{
        .p_e_m = cfg.profile.p_e_m,
        .v_e_mps = cfg.profile.v_e_mps,
        .q_b2e = core::math::normalized_with_positive_scalar(cfg.profile.q_b2e),
    };
    return generated_source(cfg.profile,
                            std::move(initial),
                            std::make_unique<GuidanceStateMachine>(std::move(cfg.state_machine)),
                            cfg.termination_mode);
}

TruthTrajectory ballistic_trajectory(const BallisticTrajectoryConfig& cfg)
{
    return materialize(ballistic_trajectory_source(cfg), cfg.profile);
}

TruthTrajectory constant_altitude_trajectory(const ConstantAltitudeTrajectoryConfig& cfg)
{
    return materialize(constant_altitude_trajectory_source(cfg), cfg.profile);
}

TruthTrajectory calibration_trajectory(const CalibrationTrajectoryConfig& cfg)
{
    return materialize(calibration_trajectory_source(cfg), cfg.profile);
}

TruthTrajectory waypoint_trajectory(const WaypointTrajectoryConfig& cfg)
{
    return materialize(waypoint_trajectory_source(cfg), cfg.profile);
}

} // namespace navkit::sim
