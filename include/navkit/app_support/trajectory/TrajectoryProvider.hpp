// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace navkit::app_support
{

struct TrajectoryRun
{
    core::Vec3 initial_position_e_m{};
    core::Vec3 initial_velocity_e_mps{};
    Eigen::Quaternion<core::Scalar_t> initial_q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 initial_w_ib_b_radps{core::Vec3::Zero()};
    std::vector<sim::TruthSample> truth_samples{};
};

namespace detail
{

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

[[nodiscard]] inline Eigen::Matrix<core::Scalar_t, 3, 3> ecef_to_ned_matrix(const core::Vec3& p_e_m)
{
    const core::Scalar_t lon_rad = std::atan2(p_e_m.y(), p_e_m.x());
    const core::Scalar_t hyp_m = std::hypot(p_e_m.x(), p_e_m.y());
    const core::Scalar_t lat_rad = std::atan2(p_e_m.z(), hyp_m);
    const core::Scalar_t sin_lat = std::sin(lat_rad);
    const core::Scalar_t cos_lat = std::cos(lat_rad);
    const core::Scalar_t sin_lon = std::sin(lon_rad);
    const core::Scalar_t cos_lon = std::cos(lon_rad);

    Eigen::Matrix<core::Scalar_t, 3, 3> C_e2n;
    C_e2n << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon, 0.0,
        -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
    return C_e2n;
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
        return vec3_from_json<core::Vec3>(trajectory.at("w_nb_b_radps")) + earth_rate_b;
    }
    return earth_rate_b;
}

} // namespace detail

inline sim::StationaryTrajectoryConfig
stationary_trajectory_config_from_json(const nlohmann::json& cfg)
{
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    sim::StationaryTrajectoryConfig traj_cfg;
    traj_cfg.duration_s = trajectory_config.at("duration_s").get<core::Time_t>();
    traj_cfg.dt_s = dt_s_from_required_runtime_rate(trajectory_config, "trajectory");
    traj_cfg.p_e = detail::position_e_m_from_json(trajectory_config);
    traj_cfg.v_e = detail::velocity_e_mps_from_json(trajectory_config, traj_cfg.p_e);
    traj_cfg.q_b2e = detail::attitude_b2e_from_json(trajectory_config, traj_cfg.p_e);
    return traj_cfg;
}

inline TrajectoryRun trajectory_run_from_json(const nlohmann::json& cfg)
{
    const sim::StationaryTrajectoryConfig traj_cfg = stationary_trajectory_config_from_json(cfg);
    const nlohmann::json& trajectory_config = cfg.at("trajectory");
    return {.initial_position_e_m = traj_cfg.p_e,
            .initial_velocity_e_mps = traj_cfg.v_e,
            .initial_q_b2e = traj_cfg.q_b2e,
            .initial_w_ib_b_radps =
                detail::angular_rate_ib_b_from_json(trajectory_config, traj_cfg.q_b2e),
            .truth_samples = sim::TrajectoryGenerator::stationary(traj_cfg)};
}

} // namespace navkit::app_support
