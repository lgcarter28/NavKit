// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace navkit::app_support
{

struct TrajectoryRun
{
    core::Vec3 initial_position_e_m{};
    core::Vec3 initial_velocity_e_mps{};
    Eigen::Quaternion<core::Scalar_t> initial_q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 initial_w_ib_b_radps{core::Vec3::Zero()};
    sim::TruthTrajectory truth{};
};

namespace detail
{

struct GeodeticPosition
{
    core::Scalar_t lat_rad{};
    core::Scalar_t lon_rad{};
    core::Scalar_t h_m{};
};

[[nodiscard]] inline core::Scalar_t deg_to_rad(const core::Scalar_t deg)
{
    return deg * (core::Scalar_t{3.141592653589793238462643383279502884} / core::Scalar_t{180.0});
}

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

[[nodiscard]] inline core::Vec3 lla_deg_m_to_ecef_m(const core::Vec3& p_lla_deg_m)
{
    using Planet = core::environment::Wgs84;

    const core::Scalar_t lat_rad = deg_to_rad(p_lla_deg_m.x());
    const core::Scalar_t lon_rad = deg_to_rad(p_lla_deg_m.y());
    const core::Scalar_t h_m = p_lla_deg_m.z();
    const core::Scalar_t sin_lat = std::sin(lat_rad);
    const core::Scalar_t cos_lat = std::cos(lat_rad);
    const core::Scalar_t sin_lon = std::sin(lon_rad);
    const core::Scalar_t cos_lon = std::cos(lon_rad);
    const core::Scalar_t prime_vertical_radius_m =
        Planet::a_m / std::sqrt(core::Scalar_t{1.0} - (Planet::e2 * sin_lat * sin_lat));

    return core::Vec3{(prime_vertical_radius_m + h_m) * cos_lat * cos_lon,
                      (prime_vertical_radius_m + h_m) * cos_lat * sin_lon,
                      ((core::Scalar_t{1.0} - Planet::e2) * prime_vertical_radius_m + h_m) *
                          sin_lat};
}

[[nodiscard]] inline GeodeticPosition geodetic_from_ecef_m(const core::Vec3& p_e_m)
{
    using Planet = core::environment::Wgs84;

    const core::Scalar_t radial_distance_m = std::hypot(p_e_m.x(), p_e_m.y());
    if (radial_distance_m <= 1.0e-9 && std::abs(p_e_m.z()) <= 1.0e-9) {
        throw_runtime_config_error("trajectory ECEF position must not be the Earth center");
    }

    GeodeticPosition geodetic{};
    geodetic.lon_rad = std::atan2(p_e_m.y(), p_e_m.x());
    geodetic.lat_rad = std::atan2(p_e_m.z(), radial_distance_m * (1.0 - Planet::e2));
    for (int iteration = 0; iteration < 8; ++iteration) {
        const core::Scalar_t sin_lat = std::sin(geodetic.lat_rad);
        const core::Scalar_t prime_vertical_radius_m =
            Planet::a_m / std::sqrt(1.0 - (Planet::e2 * sin_lat * sin_lat));
        geodetic.h_m = (radial_distance_m / std::cos(geodetic.lat_rad)) - prime_vertical_radius_m;
        geodetic.lat_rad =
            std::atan2(p_e_m.z(),
                       radial_distance_m * (1.0 - ((Planet::e2 * prime_vertical_radius_m) /
                                                   (prime_vertical_radius_m + geodetic.h_m))));
    }
    return geodetic;
}

[[nodiscard]] inline Eigen::Matrix<core::Scalar_t, 3, 3> ecef_to_ned_matrix(const core::Vec3& p_e_m)
{
    const GeodeticPosition geodetic = geodetic_from_ecef_m(p_e_m);
    const core::Scalar_t sin_lat = std::sin(geodetic.lat_rad);
    const core::Scalar_t cos_lat = std::cos(geodetic.lat_rad);
    const core::Scalar_t sin_lon = std::sin(geodetic.lon_rad);
    const core::Scalar_t cos_lon = std::cos(geodetic.lon_rad);

    Eigen::Matrix<core::Scalar_t, 3, 3> C_e2n;
    C_e2n << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon, 0.0,
        -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
    return C_e2n;
}

[[nodiscard]] inline core::Vec3 transport_rate_en_n_radps(const core::Vec3& p_e_m,
                                                          const core::Vec3& v_e_mps)
{
    using Planet = core::environment::Wgs84;

    const GeodeticPosition geodetic = geodetic_from_ecef_m(p_e_m);
    const core::Scalar_t sin_lat = std::sin(geodetic.lat_rad);
    const core::Scalar_t denominator = 1.0 - (Planet::e2 * sin_lat * sin_lat);
    const core::Scalar_t prime_vertical_radius_m = Planet::a_m / std::sqrt(denominator);
    const core::Scalar_t meridian_radius_m =
        Planet::a_m * (1.0 - Planet::e2) / (denominator * std::sqrt(denominator));
    const core::Vec3 v_n_mps = ecef_to_ned_matrix(p_e_m) * v_e_mps;

    return core::Vec3{v_n_mps.y() / (prime_vertical_radius_m + geodetic.h_m),
                      -v_n_mps.x() / (meridian_radius_m + geodetic.h_m),
                      -(v_n_mps.y() * std::tan(geodetic.lat_rad)) /
                          (prime_vertical_radius_m + geodetic.h_m)};
}

[[nodiscard]] inline Eigen::Quaternion<core::Scalar_t>
quaternion_from_json_wxyz(const nlohmann::json& value)
{
    require_numeric_array_value(value, 4U);
    Eigen::Quaternion<core::Scalar_t> q{value.at(0).get<core::Scalar_t>(),
                                        value.at(1).get<core::Scalar_t>(),
                                        value.at(2).get<core::Scalar_t>(),
                                        value.at(3).get<core::Scalar_t>()};
    return core::math::normalized_with_positive_scalar(q);
}

[[nodiscard]] inline Eigen::Matrix<core::Scalar_t, 3, 3>
dcm_from_json_row_major(const nlohmann::json& value)
{
    require_numeric_array_value(value, 9U);
    Eigen::Matrix<core::Scalar_t, 3, 3> C;
    C << value.at(0).get<core::Scalar_t>(), value.at(1).get<core::Scalar_t>(),
        value.at(2).get<core::Scalar_t>(), value.at(3).get<core::Scalar_t>(),
        value.at(4).get<core::Scalar_t>(), value.at(5).get<core::Scalar_t>(),
        value.at(6).get<core::Scalar_t>(), value.at(7).get<core::Scalar_t>(),
        value.at(8).get<core::Scalar_t>();
    return C;
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
    return lla_deg_m_to_ecef_m(vec3_from_json<core::Vec3>(trajectory.at("p_lla_deg_m")));
}

[[nodiscard]] inline core::Vec3 velocity_e_mps_from_json(const nlohmann::json& trajectory,
                                                         const core::Vec3& p_e_m)
{
    require_exactly_one_if_any(trajectory, {"v_e_mps", "v_n_mps"}, "velocity");
    if (trajectory.contains("v_e_mps")) {
        return vec3_from_json<core::Vec3>(trajectory.at("v_e_mps"));
    }
    if (trajectory.contains("v_n_mps")) {
        const Eigen::Matrix<core::Scalar_t, 3, 3> C_n2e = ecef_to_ned_matrix(p_e_m).transpose();
        return C_n2e * vec3_from_json<core::Vec3>(trajectory.at("v_n_mps"));
    }
    return core::Vec3::Zero();
}

[[nodiscard]] inline Eigen::Quaternion<core::Scalar_t>
attitude_b2e_from_json(const nlohmann::json& trajectory, const core::Vec3& p_e_m)
{
    require_exactly_one_if_any(
        trajectory,
        {"q_b2e", "rpy_b2e_rad", "dcm_b2e", "q_b2n", "rpy_b2n_rad", "dcm_b2n"},
        "attitude");

    if (trajectory.contains("q_b2e")) {
        return quaternion_from_json_wxyz(trajectory.at("q_b2e"));
    }
    if (trajectory.contains("rpy_b2e_rad")) {
        return core::math::quaternion_from_rpy_rad(
            vec3_from_json<core::Vec3>(trajectory.at("rpy_b2e_rad")));
    }
    if (trajectory.contains("dcm_b2e")) {
        return core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<core::Scalar_t>{dcm_from_json_row_major(trajectory.at("dcm_b2e"))});
    }

    Eigen::Quaternion<core::Scalar_t> q_b2n{Eigen::Quaternion<core::Scalar_t>::Identity()};
    if (trajectory.contains("q_b2n")) {
        q_b2n = quaternion_from_json_wxyz(trajectory.at("q_b2n"));
    }
    else if (trajectory.contains("rpy_b2n_rad")) {
        q_b2n = core::math::quaternion_from_rpy_rad(
            vec3_from_json<core::Vec3>(trajectory.at("rpy_b2n_rad")));
    }
    else if (trajectory.contains("dcm_b2n")) {
        q_b2n = core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<core::Scalar_t>{dcm_from_json_row_major(trajectory.at("dcm_b2n"))});
    }

    const Eigen::Quaternion<core::Scalar_t> q_n2e{
        ecef_to_ned_matrix(p_e_m)
            .transpose()}; // default body axes are NED: x north, y east, z down
    return core::math::normalized_with_positive_scalar(q_n2e * q_b2n);
}

[[nodiscard]] inline core::Vec3
angular_rate_ib_b_from_json(const nlohmann::json& trajectory,
                            const core::Vec3& p_e_m,
                            const core::Vec3& v_e_mps,
                            const Eigen::Quaternion<core::Scalar_t>& q_b2e)
{
    require_exactly_one_if_any(
        trajectory, {"w_ib_b_radps", "w_eb_b_radps", "w_nb_b_radps"}, "angular-rate");
    if (trajectory.contains("w_ib_b_radps")) {
        return vec3_from_json<core::Vec3>(trajectory.at("w_ib_b_radps"));
    }

    const core::Vec3 earth_rate_b =
        q_b2e.conjugate() * core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
    if (trajectory.contains("w_eb_b_radps")) {
        return vec3_from_json<core::Vec3>(trajectory.at("w_eb_b_radps")) + earth_rate_b;
    }
    if (trajectory.contains("w_nb_b_radps")) {
        const Eigen::Quaternion<core::Scalar_t> q_e2n{ecef_to_ned_matrix(p_e_m)};
        const Eigen::Quaternion<core::Scalar_t> q_b2n =
            core::math::normalized_with_positive_scalar(q_e2n * q_b2e);
        const core::Vec3 transport_rate_b =
            q_b2n.conjugate() * transport_rate_en_n_radps(p_e_m, v_e_mps);
        return earth_rate_b + transport_rate_b +
               vec3_from_json<core::Vec3>(trajectory.at("w_nb_b_radps"));
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
    if (samples.empty()) {
        return;
    }
    if (samples.size() == 1U) {
        samples.front().w_ib_b_radps =
            samples.front().q_b2e.conjugate() *
            core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
        return;
    }
    for (std::size_t index = 0U; index < samples.size(); ++index) {
        const std::size_t first_index = index + 1U < samples.size() ? index : index - 1U;
        const std::size_t second_index = index + 1U < samples.size() ? index + 1U : index;
        if (first_index == second_index) {
            samples.at(index).w_ib_b_radps =
                samples.at(index).q_b2e.conjugate() *
                core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
            continue;
        }

        const sim::TruthSample& first = samples.at(first_index);
        const sim::TruthSample& second = samples.at(second_index);
        core::Duration duration{};
        if (!core::elapsed_time(second.t, first.t, duration)) {
            throw_runtime_config_error("trajectory CSV timestamps must be strictly increasing");
        }
        const core::Time_t dt_s = core::duration_seconds(duration);
        const Eigen::Quaternion<core::Scalar_t> q_mid =
            core::math::normalized_with_positive_scalar(first.q_b2e.slerp(0.5, second.q_b2e));
        const Eigen::Quaternion<core::Scalar_t> q_body_relative =
            first.q_b2e.conjugate() * second.q_b2e;
        samples.at(index).w_ib_b_radps =
            (core::math::rotvec_rad_from_quaternion(q_body_relative) / dt_s) +
            (q_mid.conjugate() *
             core::environment::planet_rate_fixed_radps<core::environment::Wgs84>());
    }
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
    traj_cfg.rate = rational_rate_from_required_runtime_rate(trajectory_config, "trajectory");
    traj_cfg.p_e = detail::position_e_m_from_json(trajectory_config);
    traj_cfg.v_e = detail::velocity_e_mps_from_json(trajectory_config, traj_cfg.p_e);
    traj_cfg.q_b2e = detail::attitude_b2e_from_json(trajectory_config, traj_cfg.p_e);
    traj_cfg.w_ib_b_radps = detail::angular_rate_ib_b_from_json(
        trajectory_config, traj_cfg.p_e, traj_cfg.v_e, traj_cfg.q_b2e);
    return traj_cfg;
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
        return {.initial_position_e_m = truth.first().p_e,
                .initial_velocity_e_mps = truth.first().v_e,
                .initial_q_b2e = truth.first().q_b2e,
                .initial_w_ib_b_radps = truth.first().w_ib_b_radps,
                .truth = std::move(truth)};
    }

    const sim::StationaryTrajectoryConfig traj_cfg = stationary_trajectory_config_from_json(cfg);
    return {.initial_position_e_m = traj_cfg.p_e,
            .initial_velocity_e_mps = traj_cfg.v_e,
            .initial_q_b2e = traj_cfg.q_b2e,
            .initial_w_ib_b_radps = traj_cfg.w_ib_b_radps,
            .truth = sim::TrajectoryGenerator::stationary(traj_cfg)};
}

} // namespace navkit::app_support
