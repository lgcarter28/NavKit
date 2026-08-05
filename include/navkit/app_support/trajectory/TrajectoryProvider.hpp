// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/app_support/trajectory/GuidanceStateMachineJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryAttitudeJson.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/sim/guidance/GuidanceCommandFilter.hpp"
#include "navkit/sim/trajectory/StationaryTrajectorySource.hpp"
#include "navkit/sim/trajectory/TabulatedTrajectorySource.hpp"
#include "navkit/sim/trajectory/TrajectoryProfiles.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace navkit::app_support
{

struct TrajectoryRun
{
    std::unique_ptr<sim::TrajectorySource> source{};
    sim::TruthSample initial_truth{};
};

namespace detail
{

[[nodiscard]] inline int count_present(const nlohmann::json& cfg,
                                       const std::vector<std::string>& keys)
{
    int count = 0;
    for (const std::string& key : keys) {
        if (cfg.contains(key)) {
            ++count;
        }
    }
    return count;
}

inline void require_exactly_one_if_any(const nlohmann::json& cfg,
                                       const std::vector<std::string>& keys,
                                       const std::string& group_name)
{
    const int count = count_present(cfg, keys);
    if (count > 1) {
        throw_runtime_config_error("trajectory must specify at most one " + group_name +
                                   " convention");
    }
}

inline void require_numeric_array_value(const nlohmann::json& value, const std::size_t count)
{
    if (!value.is_array() || value.size() != count) {
        throw_runtime_config_error("trajectory numeric array has unexpected size");
    }
    for (const nlohmann::json& entry : value) {
        if (!entry.is_number()) {
            throw_runtime_config_error("trajectory numeric array entries must be numeric");
        }
    }
}

[[nodiscard]] inline core::Vec3 position_e_m_from_json(const nlohmann::json& trajectory)
{
    const int position_count = count_present(trajectory, {"p_e_m", "p_lla_deg_m"});
    if (position_count == 0) {
        throw_runtime_config_error("trajectory must specify one of 'p_e_m' or 'p_lla_deg_m'");
    }
    if (position_count > 1) {
        throw_runtime_config_error("trajectory must specify only one position convention");
    }

    if (trajectory.contains("p_e_m")) {
        return vec3_from_json<core::Vec3>(trajectory.at("p_e_m"));
    }
    const core::Vec3 p_lla_deg_m = vec3_from_json<core::Vec3>(trajectory.at("p_lla_deg_m"));
    core::Vec3 p_e_m{};
    if (!core::frames::lla_deg_m_to_ecef_m(p_lla_deg_m, p_e_m)) {
        throw_runtime_config_error(
            "trajectory 'p_lla_deg_m' must contain finite latitude, longitude, and height");
    }
    return p_e_m;
}

[[nodiscard]] inline core::Vec3 velocity_e_mps_from_json(const nlohmann::json& trajectory,
                                                         const core::Vec3& p_e_m)
{
    require_exactly_one_if_any(trajectory, {"v_e_mps", "v_n_mps"}, "velocity");
    if (trajectory.contains("v_e_mps")) {
        return vec3_from_json<core::Vec3>(trajectory.at("v_e_mps"));
    }
    if (trajectory.contains("v_n_mps")) {
        core::Mat3 C_n2e{};
        if (!core::frames::ned_to_ecef_matrix(p_e_m, C_n2e)) {
            throw_runtime_config_error(
                "trajectory NED velocity requires a valid noncentral ECEF position");
        }
        return C_n2e * vec3_from_json<core::Vec3>(trajectory.at("v_n_mps"));
    }
    return core::Vec3::Zero();
}

[[nodiscard]] inline core::Vec3
angular_rate_ib_b_from_json(const nlohmann::json& trajectory,
                            const core::Vec3& p_e_m,
                            const core::Vec3& v_e_mps,
                            const Eigen::Quaternion<core::Scalar_t>& q_b2e)
{
    require_exactly_one_if_any(
        trajectory, {"w_ib_b_degps", "w_eb_b_degps", "w_nb_b_degps"}, "angular-rate");
    if (trajectory.contains("w_ib_b_degps")) {
        return radians_from_degrees_json<core::Vec3>(trajectory.at("w_ib_b_degps"));
    }

    const core::Vec3 earth_rate_b =
        q_b2e.conjugate() * core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
    if (trajectory.contains("w_eb_b_degps")) {
        return radians_from_degrees_json<core::Vec3>(trajectory.at("w_eb_b_degps")) + earth_rate_b;
    }
    if (trajectory.contains("w_nb_b_degps")) {
        core::Mat3 C_e2n{};
        core::Vec3 w_en_n_radps{};
        if (!core::frames::ecef_to_ned_matrix(p_e_m, C_e2n) ||
            !core::frames::transport_rate_en_n_radps(p_e_m, v_e_mps, w_en_n_radps)) {
            throw_runtime_config_error(
                "trajectory NED angular rate requires valid ECEF position and velocity");
        }
        const Eigen::Quaternion<core::Scalar_t> q_e2n{C_e2n};
        const Eigen::Quaternion<core::Scalar_t> q_b2n =
            core::math::normalized_with_positive_scalar(q_e2n * q_b2e);
        const core::Vec3 transport_rate_b = q_b2n.conjugate() * w_en_n_radps;
        return earth_rate_b + transport_rate_b +
               radians_from_degrees_json<core::Vec3>(trajectory.at("w_nb_b_degps"));
    }
    return earth_rate_b;
}

[[nodiscard]] inline std::vector<std::string> csv_columns_from_line(const std::string& line)
{
    std::vector<std::string> columns{};
    std::stringstream stream(line);
    std::string column{};
    while (std::getline(stream, column, ',')) {
        columns.push_back(column);
    }
    return columns;
}

[[nodiscard]] inline core::Scalar_t
csv_scalar_at(const std::vector<std::string>& values,
              const std::unordered_map<std::string, std::size_t>& columns,
              const std::string& name)
{
    const std::unordered_map<std::string, std::size_t>::const_iterator iter = columns.find(name);
    if (iter == columns.end() || iter->second >= values.size()) {
        throw_runtime_config_error("trajectory CSV is missing required column '" + name + "'");
    }
    try {
        return std::stod(values.at(iter->second));
    }
    catch (const std::exception&) {
        throw_runtime_config_error("trajectory CSV column '" + name + "' must be numeric");
    }
}

[[nodiscard]] inline bool
csv_has_columns(const std::unordered_map<std::string, std::size_t>& columns,
                const std::initializer_list<std::string>& names)
{
    for (const std::string& name : names) {
        if (!columns.contains(name)) {
            return false;
        }
    }
    return true;
}

inline void populate_missing_angular_rates(std::vector<sim::TruthSample>& samples)
{
    sim::populate_truth_angular_rates(samples);
}

[[nodiscard]] inline sim::TruthTrajectory
truth_trajectory_from_csv(const std::filesystem::path& path)
{
    std::ifstream stream(path);
    if (!stream) {
        throw_runtime_config_error("failed to open trajectory CSV '" + path.string() + "'");
    }

    std::string header_line{};
    if (!std::getline(stream, header_line)) {
        throw_runtime_config_error("trajectory CSV must contain a header row");
    }
    const std::vector<std::string> header = csv_columns_from_line(header_line);
    std::unordered_map<std::string, std::size_t> columns{};
    for (std::size_t index = 0U; index < header.size(); ++index) {
        columns.emplace(header.at(index), index);
    }

    const std::initializer_list<std::string> angular_rate_columns{
        "w_ib_b_x_radps", "w_ib_b_y_radps", "w_ib_b_z_radps"};
    const bool angular_rates_present = csv_has_columns(columns, angular_rate_columns);
    int angular_rate_column_count = 0;
    for (const std::string& name : angular_rate_columns) {
        if (columns.contains(name)) {
            ++angular_rate_column_count;
        }
    }
    if (angular_rate_column_count != 0 && angular_rate_column_count != 3) {
        throw_runtime_config_error(
            "trajectory CSV must provide all or none of the w_ib_b_*_radps columns");
    }
    std::vector<sim::TruthSample> samples{};
    std::string line{};
    while (std::getline(stream, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> values = csv_columns_from_line(line);
        sim::TruthSample sample{};
        if (!core::timestamp_from_seconds(
                csv_scalar_at(values, columns, "time_s"), core::TimeScale::Monotonic, sample.t)) {
            throw_runtime_config_error("trajectory CSV time_s must be finite and nonnegative");
        }
        sample.p_e << csv_scalar_at(values, columns, "p_e_x_m"),
            csv_scalar_at(values, columns, "p_e_y_m"), csv_scalar_at(values, columns, "p_e_z_m");
        sample.v_e << csv_scalar_at(values, columns, "v_e_x_mps"),
            csv_scalar_at(values, columns, "v_e_y_mps"),
            csv_scalar_at(values, columns, "v_e_z_mps");
        sample.q_b2e = core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<core::Scalar_t>{csv_scalar_at(values, columns, "q_b2e_w"),
                                              csv_scalar_at(values, columns, "q_b2e_x"),
                                              csv_scalar_at(values, columns, "q_b2e_y"),
                                              csv_scalar_at(values, columns, "q_b2e_z")});
        if (angular_rates_present) {
            sample.w_ib_b_radps << csv_scalar_at(values, columns, "w_ib_b_x_radps"),
                csv_scalar_at(values, columns, "w_ib_b_y_radps"),
                csv_scalar_at(values, columns, "w_ib_b_z_radps");
        }
        if (!samples.empty() && !core::timestamp_less(samples.back().t, sample.t)) {
            throw_runtime_config_error("trajectory CSV timestamps must be strictly increasing");
        }
        samples.push_back(sample);
    }
    if (samples.empty()) {
        throw_runtime_config_error("trajectory CSV must contain at least one sample");
    }
    if (!angular_rates_present) {
        populate_missing_angular_rates(samples);
    }
    return sim::TruthTrajectory{std::move(samples)};
}

} // namespace detail

inline sim::StationaryTrajectoryConfig
stationary_trajectory_config_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    sim::StationaryTrajectoryConfig traj_cfg;
    traj_cfg.duration_s = trajectory_config.at("duration_s").get<core::Time_t>();
    traj_cfg.rate = rational_rate_from_required_named_runtime_rate(
        trajectory_config, "dynamics_rate_hz", "dynamics_dt_s", "trajectory.dynamics");
    traj_cfg.p_e = detail::position_e_m_from_json(trajectory_config);
    traj_cfg.v_e = detail::velocity_e_mps_from_json(trajectory_config, traj_cfg.p_e);
    traj_cfg.q_b2e = detail::trajectory_attitude_b2e_from_json(
        trajectory_config, traj_cfg.p_e, traj_cfg.t_epoch, traj_cfg.t_epoch);
    traj_cfg.w_ib_b_radps = detail::angular_rate_ib_b_from_json(
        trajectory_config, traj_cfg.p_e, traj_cfg.v_e, traj_cfg.q_b2e);
    return traj_cfg;
}

[[nodiscard]] inline sim::TranslationalIntegrationMethod
translational_integration_method_from_json(const nlohmann::json& trajectory_config)
{
    const std::string method = trajectory_config.at("translational_integration").get<std::string>();
    if (method == "semi_implicit_euler") {
        return sim::TranslationalIntegrationMethod::SemiImplicitEuler;
    }
    if (method == "trapezoidal_predictor_corrector") {
        return sim::TranslationalIntegrationMethod::TrapezoidalPredictorCorrector;
    }
    detail::throw_runtime_config_error(
        "trajectory.translational_integration must be 'semi_implicit_euler' or "
        "'trapezoidal_predictor_corrector'");
}

[[nodiscard]] inline core::RationalRate
trajectory_subsystem_rate_from_json(const nlohmann::json& trajectory_config,
                                    const std::string_view subsystem)
{
    const std::string rate_key = std::string{subsystem} + "_rate_hz";
    const std::string period_key = std::string{subsystem} + "_dt_s";
    return rational_rate_from_required_named_runtime_rate(trajectory_config,
                                                          rate_key,
                                                          period_key,
                                                          std::string{"trajectory."} +
                                                              std::string{subsystem});
}

[[nodiscard]] inline sim::FirstOrderAutopilotConfig
autopilot_config_from_json(const nlohmann::json& trajectory_config)
{
    const nlohmann::json& config = trajectory_config.at("autopilot");
    if (config.at("type").get<std::string>() != "first_order") {
        detail::throw_runtime_config_error("trajectory.autopilot.type must be 'first_order'");
    }
    sim::FirstOrderAutopilotConfig result{
        .attitude_command_time_constant_s =
            config.value("attitude_command_time_constant_s", core::Time_t{}),
        .controller_rate_time_constant_pqr_s =
            vec3_from_json<core::Vec3>(config.at("controller_rate_time_constant_pqr_s")),
        .attitude_error_gain_pqr_per_s =
            vec3_from_json<core::Vec3>(config.at("attitude_error_gain_pqr_per_s")),
        .angular_rate_feedback_gain_pqr =
            vec3_from_json<core::Vec3>(config.at("angular_rate_feedback_gain_pqr")),
        .velocity_alignment_speed_threshold_mps =
            config.at("velocity_alignment_speed_threshold_mps").get<core::Scalar_t>(),
        .initial_velocity_alignment_tolerance_rad = radians_from_degrees(
            config.at("initial_velocity_alignment_tolerance_deg").get<core::Scalar_t>()),
        .gyro_moving_average_window_samples =
            config.at("gyro_moving_average_window_samples").get<std::size_t>(),
    };
    if (!sim::first_order_autopilot_config_is_valid(result)) {
        detail::throw_runtime_config_error(
            "trajectory first-order Autopilot time constants, gains, alignment settings, and "
            "moving-average window are invalid");
    }
    return result;
}

[[nodiscard]] inline sim::GuidanceCommandFilterConfig
guidance_command_filter_config_from_json(const nlohmann::json& trajectory_config)
{
    const nlohmann::json& config = trajectory_config.at("guidance_command_filter");
    sim::GuidanceCommandFilterConfig result{
        .specific_force_time_constant_b_s =
            vec3_from_json<core::Vec3>(config.at("specific_force_time_constant_b_s")),
        .bank_time_constant_s = config.at("bank_time_constant_s").get<core::Time_t>(),
    };
    if (!sim::guidance_command_filter_config_is_valid(result)) {
        detail::throw_runtime_config_error(
            "trajectory Guidance command-filter time constants must be finite and nonnegative");
    }
    return result;
}

[[nodiscard]] inline sim::AutopilotModelType
autopilot_model_type_from_json(const nlohmann::json& trajectory_config)
{
    const std::string type = trajectory_config.at("autopilot").at("type").get<std::string>();
    if (type == "first_order") {
        return sim::AutopilotModelType::FirstOrder;
    }
    detail::throw_runtime_config_error("trajectory.autopilot.type must be 'first_order'");
}

[[nodiscard]] inline sim::FirstOrderVehicleResponseConfig
vehicle_response_config_from_json(const nlohmann::json& trajectory_config)
{
    const nlohmann::json& config = trajectory_config.at("vehicle_response");
    if (config.at("type").get<std::string>() != "first_order") {
        detail::throw_runtime_config_error(
            "trajectory.vehicle_response.type must be 'first_order'");
    }
    sim::FirstOrderVehicleResponseConfig result{
        .vehicle_rate_time_constant_pqr_s =
            vec3_from_json<core::Vec3>(config.at("vehicle_rate_time_constant_pqr_s")),
        .specific_force_command_time_constant_b_s =
            vec3_from_json<core::Vec3>(config.at("specific_force_command_time_constant_b_s")),
        .specific_force_response_time_constant_b_s =
            vec3_from_json<core::Vec3>(config.at("specific_force_response_time_constant_b_s")),
    };
    if (config.contains("angular_rate_limit_pqr_degps")) {
        result.angular_rate_limits_enabled = true;
        result.angular_rate_limit_pqr_radps =
            radians_from_degrees_json<core::Vec3>(config.at("angular_rate_limit_pqr_degps"));
    }
    if (config.contains("specific_force_limit_b_mps2")) {
        result.specific_force_limits_enabled = true;
        result.specific_force_limit_b_mps2 =
            vec3_from_json<core::Vec3>(config.at("specific_force_limit_b_mps2"));
    }
    if (!sim::first_order_vehicle_response_config_is_valid(result)) {
        detail::throw_runtime_config_error(
            "trajectory first-order vehicle-response time constants must be finite and "
            "nonnegative, and configured limits must be finite and positive");
    }
    return result;
}

[[nodiscard]] inline sim::VehicleResponseModelType
vehicle_response_model_type_from_json(const nlohmann::json& trajectory_config)
{
    const std::string type = trajectory_config.at("vehicle_response").at("type").get<std::string>();
    if (type == "first_order") {
        return sim::VehicleResponseModelType::FirstOrder;
    }
    detail::throw_runtime_config_error("trajectory.vehicle_response.type must be 'first_order'");
}

[[nodiscard]] inline sim::TrajectoryProfileConfig
trajectory_profile_config_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    sim::TrajectoryProfileConfig profile{};
    profile.duration_s = trajectory_config.at("duration_s").get<core::Time_t>();
    profile.rate = rational_rate_from_required_named_runtime_rate(
        trajectory_config, "dynamics_rate_hz", "dynamics_dt_s", "trajectory.dynamics");
    profile.guidance_rate = trajectory_subsystem_rate_from_json(trajectory_config, "guidance");
    profile.autopilot_rate = trajectory_subsystem_rate_from_json(trajectory_config, "autopilot");
    profile.p_e_m = detail::position_e_m_from_json(trajectory_config);
    profile.initial_velocity_configured =
        trajectory_config.contains("v_e_mps") || trajectory_config.contains("v_n_mps");
    profile.v_e_mps = detail::velocity_e_mps_from_json(trajectory_config, profile.p_e_m);
    profile.q_b2e = detail::trajectory_attitude_b2e_from_json(
        trajectory_config, profile.p_e_m, profile.t_epoch, profile.t_epoch);
    profile.maximum_bank_angle_rad =
        radians_from_degrees(trajectory_config.at("maximum_bank_angle_deg").get<core::Scalar_t>());
    profile.guidance_command_filter = guidance_command_filter_config_from_json(trajectory_config);
    profile.translational_integration =
        translational_integration_method_from_json(trajectory_config);
    profile.autopilot_model = autopilot_model_type_from_json(trajectory_config);
    profile.autopilot = autopilot_config_from_json(trajectory_config);
    profile.vehicle_response_model = vehicle_response_model_type_from_json(trajectory_config);
    profile.vehicle_response = vehicle_response_config_from_json(trajectory_config);
    return profile;
}

[[nodiscard]] inline sim::StateMachineTrajectoryConfig
state_machine_trajectory_config_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    sim::StateMachineTrajectoryConfig result{};
    result.profile = trajectory_profile_config_from_json(cfg);
    result.state_machine =
        detail::guidance_state_machine_from_json(trajectory_config,
                                                 result.profile.guidance_command_filter,
                                                 result.profile.maximum_bank_angle_rad);
    const std::string termination_type =
        trajectory_config.at("termination").at("type").get<std::string>();
    result.termination_mode = termination_type == "ground_impact"
                                  ? sim::TrajectoryTerminationMode::GroundImpact
                                  : sim::TrajectoryTerminationMode::ConfiguredDuration;
    return result;
}

inline TrajectoryRun trajectory_run_from_json(const nlohmann::json& cfg,
                                              const std::filesystem::path& source_base_dir = {})
{
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    const std::string type = trajectory_config.value("type", "stationary");
    if (type == "csv") {
        const std::filesystem::path csv_path =
            source_base_dir / trajectory_config.at("csv_path").get<std::string>();
        sim::TruthTrajectory truth = detail::truth_trajectory_from_csv(csv_path);
        const sim::TruthSample initial_truth = truth.first();
        return {.source = std::make_unique<sim::TabulatedTrajectorySource>(std::move(truth)),
                .initial_truth = initial_truth};
    }

    if (type == "stationary") {
        const sim::StationaryTrajectoryConfig trajectory =
            stationary_trajectory_config_from_json(cfg);
        const sim::TruthSample initial_truth{
            .t = trajectory.t_epoch,
            .p_e = trajectory.p_e,
            .v_e = trajectory.v_e,
            .q_b2e = trajectory.q_b2e,
            .w_ib_b_radps = trajectory.w_ib_b_radps,
        };
        return {.source = std::make_unique<sim::StationaryTrajectorySource>(trajectory),
                .initial_truth = initial_truth};
    }

    std::unique_ptr<sim::TrajectorySource> source{};
    if (type == "state_machine") {
        source =
            sim::state_machine_trajectory_source(state_machine_trajectory_config_from_json(cfg));
    }
    else {
        detail::throw_runtime_config_error(
            "trajectory.type must be 'stationary', 'csv', or 'state_machine'");
    }

    if (!source || !source->advance_to(source->t_start())) {
        detail::throw_runtime_config_error(
            "trajectory generation failed for the configured profile");
    }
    sim::TruthSample initial_truth{};
    if (!source->query(source->t_start(), initial_truth)) {
        detail::throw_runtime_config_error(
            "trajectory generation did not provide its initial truth sample");
    }
    return {.source = std::move(source), .initial_truth = initial_truth};
}

} // namespace navkit::app_support
