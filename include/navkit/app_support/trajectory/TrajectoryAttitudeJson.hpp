// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigError.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::app_support::detail
{

inline constexpr std::array<std::string_view, 18> trajectory_attitude_json_keys{
    "q_b2e",
    "dcm_b2e",
    "rpy_b2e_deg",
    "q_e2b",
    "dcm_e2b",
    "rpy_e2b_deg",
    "q_b2i",
    "dcm_b2i",
    "rpy_b2i_deg",
    "q_i2b",
    "dcm_i2b",
    "rpy_i2b_deg",
    "q_b2n",
    "dcm_b2n",
    "rpy_b2n_deg",
    "q_n2b",
    "dcm_n2b",
    "rpy_n2b_deg",
};

enum class TrajectoryAttitudeFramePair
{
    BodyToEcef,
    EcefToBody,
    BodyToInertial,
    InertialToBody,
    BodyToNed,
    NedToBody,
};

[[nodiscard]] inline int trajectory_attitude_input_count(const nlohmann::json& trajectory)
{
    int count = 0;
    for (const std::string_view key : trajectory_attitude_json_keys) {
        if (trajectory.contains(std::string{key})) {
            ++count;
        }
    }
    return count;
}

[[nodiscard]] inline std::string_view
trajectory_attitude_input_key(const nlohmann::json& trajectory)
{
    for (const std::string_view key : trajectory_attitude_json_keys) {
        if (trajectory.contains(std::string{key})) {
            return key;
        }
    }
    return {};
}

inline void require_finite_numeric_array(const nlohmann::json& value,
                                         const std::size_t expected_size,
                                         const std::string_view key)
{
    if (!value.is_array() || value.size() != expected_size) {
        throw_runtime_config_error("trajectory attitude '" + std::string{key} +
                                   "' must contain exactly " + std::to_string(expected_size) +
                                   " numeric entries");
    }
    for (const nlohmann::json& entry : value) {
        if (!entry.is_number() || !std::isfinite(entry.get<core::Scalar_t>())) {
            throw_runtime_config_error("trajectory attitude '" + std::string{key} +
                                       "' entries must be finite numbers");
        }
    }
}

[[nodiscard]] inline TrajectoryAttitudeFramePair
trajectory_attitude_frame_pair(const std::string_view key)
{
    if (key.find("b2e") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::BodyToEcef;
    }
    if (key.find("e2b") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::EcefToBody;
    }
    if (key.find("b2i") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::BodyToInertial;
    }
    if (key.find("i2b") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::InertialToBody;
    }
    if (key.find("b2n") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::BodyToNed;
    }
    if (key.find("n2b") != std::string_view::npos) {
        return TrajectoryAttitudeFramePair::NedToBody;
    }
    throw_runtime_config_error("unsupported trajectory attitude frame pair in '" +
                               std::string{key} + "'");
}

[[nodiscard]] inline Eigen::Quaternion<core::Scalar_t>
trajectory_attitude_quaternion_from_json(const nlohmann::json& trajectory,
                                         const std::string_view key)
{
    const nlohmann::json& value = trajectory.at(std::string{key});
    Eigen::Quaternion<core::Scalar_t> q_start2end{};
    if (key.starts_with("q_")) {
        require_finite_numeric_array(value, 4U, key);
        const Eigen::Quaternion<core::Scalar_t> q{
            value.at(0).get<core::Scalar_t>(),
            value.at(1).get<core::Scalar_t>(),
            value.at(2).get<core::Scalar_t>(),
            value.at(3).get<core::Scalar_t>(),
        };
        if (!core::math::normalize_quaternion(q, q_start2end)) {
            throw_runtime_config_error("trajectory attitude '" + std::string{key} +
                                       "' must be a finite nonzero quaternion");
        }
        return q_start2end;
    }

    if (key.starts_with("dcm_")) {
        require_finite_numeric_array(value, 9U, key);
        core::Mat3 C_start2end{};
        C_start2end << value.at(0).get<core::Scalar_t>(), value.at(1).get<core::Scalar_t>(),
            value.at(2).get<core::Scalar_t>(), value.at(3).get<core::Scalar_t>(),
            value.at(4).get<core::Scalar_t>(), value.at(5).get<core::Scalar_t>(),
            value.at(6).get<core::Scalar_t>(), value.at(7).get<core::Scalar_t>(),
            value.at(8).get<core::Scalar_t>();
        if (!core::math::quaternion_from_dcm(C_start2end, q_start2end)) {
            throw_runtime_config_error("trajectory attitude '" + std::string{key} +
                                       "' must be a proper orthonormal DCM");
        }
        return q_start2end;
    }

    require_finite_numeric_array(value, 3U, key);
    const core::Vec3 rpy_start2end_rad = radians_from_degrees_json<core::Vec3>(value);
    const Eigen::Quaternion<core::Scalar_t> q =
        core::math::quaternion_from_rpy_rad(rpy_start2end_rad);
    if (!core::math::normalize_quaternion(q, q_start2end)) {
        throw_runtime_config_error("trajectory attitude '" + std::string{key} +
                                   "' could not be converted to a finite quaternion");
    }
    return q_start2end;
}

inline void validate_trajectory_attitude_json(const nlohmann::json& trajectory)
{
    const int count = trajectory_attitude_input_count(trajectory);
    if (count != 1) {
        throw_runtime_config_error(
            "trajectory must specify exactly one supported attitude convention");
    }
    const std::string_view key = trajectory_attitude_input_key(trajectory);
    (void)trajectory_attitude_quaternion_from_json(trajectory, key);
}

/**
 * Converts one validated trajectory attitude payload to canonical passive
 * body-to-ECEF quaternion form.
 *
 * NED inputs require the initial ECEF position. Inertial inputs use a uniform
 * WGS-84 Earth-orientation model with ECI and ECEF aligned at `t_epoch`.
 */
[[nodiscard]] inline Eigen::Quaternion<core::Scalar_t>
trajectory_attitude_b2e_from_json(const nlohmann::json& trajectory,
                                  const core::Vec3& p_e_m,
                                  const core::Timestamp& t,
                                  const core::Timestamp& t_epoch)
{
    validate_trajectory_attitude_json(trajectory);
    const std::string_view key = trajectory_attitude_input_key(trajectory);
    const Eigen::Quaternion<core::Scalar_t> q_start2end =
        trajectory_attitude_quaternion_from_json(trajectory, key);
    const TrajectoryAttitudeFramePair frame_pair = trajectory_attitude_frame_pair(key);

    Eigen::Quaternion<core::Scalar_t> q_b2e{};
    if (frame_pair == TrajectoryAttitudeFramePair::BodyToEcef) {
        q_b2e = q_start2end;
    }
    else if (frame_pair == TrajectoryAttitudeFramePair::EcefToBody) {
        q_b2e = q_start2end.conjugate();
    }
    else if (frame_pair == TrajectoryAttitudeFramePair::BodyToNed ||
             frame_pair == TrajectoryAttitudeFramePair::NedToBody) {
        core::Mat3 C_n2e{};
        if (!core::frames::ned_to_ecef_matrix(p_e_m, C_n2e)) {
            throw_runtime_config_error(
                "trajectory NED attitude requires a valid noncentral ECEF position");
        }
        const Eigen::Quaternion<core::Scalar_t> q_n2e{C_n2e};
        const Eigen::Quaternion<core::Scalar_t> q_b2n =
            frame_pair == TrajectoryAttitudeFramePair::BodyToNed ? q_start2end
                                                                 : q_start2end.conjugate();
        q_b2e = q_n2e * q_b2n;
    }
    else {
        core::Mat3 C_i2e{};
        if (!core::frames::inertial_to_fixed_matrix<core::environment::Wgs84>(t, t_epoch, C_i2e)) {
            throw_runtime_config_error(
                "trajectory inertial attitude requires compatible valid timestamps");
        }
        const Eigen::Quaternion<core::Scalar_t> q_i2e{C_i2e};
        const Eigen::Quaternion<core::Scalar_t> q_b2i =
            frame_pair == TrajectoryAttitudeFramePair::BodyToInertial ? q_start2end
                                                                      : q_start2end.conjugate();
        q_b2e = q_i2e * q_b2i;
    }

    Eigen::Quaternion<core::Scalar_t> q_b2e_normalized{};
    if (!core::math::normalize_quaternion(q_b2e, q_b2e_normalized)) {
        throw_runtime_config_error(
            "trajectory attitude conversion produced an invalid body-to-ECEF quaternion");
    }
    return q_b2e_normalized;
}

} // namespace navkit::app_support::detail
