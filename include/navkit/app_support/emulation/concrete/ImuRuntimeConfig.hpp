// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "navkit/sim/RandomDraw.hpp"

#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <string_view>

namespace navkit::app_support
{

namespace detail
{

struct ImuTriadRuntimeKeys
{
    std::string_view bias_turnon;
    std::string_view bias_turnon_var;
    std::string_view bias_turnon_cov;
    std::string_view bias_inrun_psd;
    std::string_view output_random_walk_psd;
    std::string_view scale_factor_var;
    std::string_view scale_factor_cov;
    std::string_view misalignment_var;
    std::string_view misalignment_cov;
    std::string_view nonorthogonality_var;
    std::string_view nonorthogonality_cov;
    std::string_view quantization;
    std::string_view limit;
};

[[nodiscard]] inline bool json_contains(const nlohmann::json& cfg, const std::string_view key)
{
    return cfg.contains(std::string(key));
}

inline void require_optional_nonnegative_vec3(const nlohmann::json& cfg, std::string_view path)
{
    require_optional_vec3(cfg, path);
    const nlohmann::json::const_iterator iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }

    for (const nlohmann::json& value : *iter) {
        if (value.get<core::Scalar_t>() < 0.0) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be nonnegative");
        }
    }
}

inline void require_optional_cov3(const nlohmann::json& cfg, std::string_view path)
{
    require_optional_numeric_array(cfg, path, 9U);
    const nlohmann::json::const_iterator iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }

    core::Mat3 covariance = core::Mat3::Zero();
    for (Eigen::Index row = 0; row < 3; ++row) {
        for (Eigen::Index col = 0; col < 3; ++col) {
            const std::size_t index =
                (static_cast<std::size_t>(row) * 3U) + static_cast<std::size_t>(col);
            covariance(row, col) = iter->at(index).get<core::Scalar_t>();
        }
    }

    if (!covariance.isApprox(covariance.transpose(), 1.0e-12)) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be symmetric");
    }

    const Eigen::SelfAdjointEigenSolver<core::Mat3> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success || eigensolver.eigenvalues().minCoeff() < -1.0e-12) {
        throw_runtime_config_error("expected " + quoted_path(path) +
                                   " to be positive semidefinite");
    }
}

inline void validate_direct_or_random_vec3(const nlohmann::json& triad,
                                           const std::string_view direct_key,
                                           const std::string_view var_key,
                                           const std::string_view cov_key)
{
    require_optional_vec3(triad, direct_key);
    require_optional_nonnegative_vec3(triad, var_key);
    require_optional_cov3(triad, cov_key);

    const int form_count = (json_contains(triad, direct_key) ? 1 : 0) +
                           (json_contains(triad, var_key) ? 1 : 0) +
                           (json_contains(triad, cov_key) ? 1 : 0);
    if (form_count > 1) {
        throw_runtime_config_error("IMU triad term must specify only one of '" +
                                   std::string(direct_key) + "', '" + std::string(var_key) +
                                   "', or '" + std::string(cov_key) + "'");
    }
}

inline void validate_imu_triad_config(const nlohmann::json& triad,
                                      std::string_view name,
                                      const ImuTriadRuntimeKeys& keys)
{
    if (!triad.is_object()) {
        throw_runtime_config_error("expected " + quoted_path(name) + " to be an object");
    }

    reject_unknown_top_level_keys(triad,
                                  {keys.bias_turnon,
                                   keys.bias_turnon_var,
                                   keys.bias_turnon_cov,
                                   keys.bias_inrun_psd,
                                   keys.output_random_walk_psd,
                                   "scale_factor",
                                   keys.scale_factor_var,
                                   keys.scale_factor_cov,
                                   "misalignment_rad",
                                   keys.misalignment_var,
                                   keys.misalignment_cov,
                                   "nonorthogonality",
                                   keys.nonorthogonality_var,
                                   keys.nonorthogonality_cov,
                                   keys.quantization,
                                   keys.limit});

    validate_direct_or_random_vec3(
        triad, keys.bias_turnon, keys.bias_turnon_var, keys.bias_turnon_cov);
    require_optional_nonnegative_vec3(triad, keys.bias_inrun_psd);
    require_optional_nonnegative_vec3(triad, keys.output_random_walk_psd);
    validate_direct_or_random_vec3(
        triad, "scale_factor", keys.scale_factor_var, keys.scale_factor_cov);
    validate_direct_or_random_vec3(
        triad, "misalignment_rad", keys.misalignment_var, keys.misalignment_cov);
    validate_direct_or_random_vec3(
        triad, "nonorthogonality", keys.nonorthogonality_var, keys.nonorthogonality_cov);
    require_optional_nonnegative_vec3(triad, keys.quantization);
    require_optional_nonnegative_vec3(triad, keys.limit);
}

[[nodiscard]] inline core::Mat3 cov3_from_json(const nlohmann::json& value)
{
    core::Mat3 covariance = core::Mat3::Zero();
    for (Eigen::Index row = 0; row < 3; ++row) {
        for (Eigen::Index col = 0; col < 3; ++col) {
            const std::size_t index =
                (static_cast<std::size_t>(row) * 3U) + static_cast<std::size_t>(col);
            covariance(row, col) = value.at(index).get<core::Scalar_t>();
        }
    }
    return covariance;
}

[[nodiscard]] inline core::Vec3 vec3_direct_or_random_from_json(const nlohmann::json& triad,
                                                                const std::string_view direct_key,
                                                                const std::string_view var_key,
                                                                const std::string_view cov_key,
                                                                std::mt19937& rng)
{
    if (json_contains(triad, direct_key)) {
        return vec3_from_json<core::Vec3>(triad.at(std::string(direct_key)));
    }
    if (json_contains(triad, var_key)) {
        return sim::draw_normal_diag_cov<3>(
            vec3_from_json<core::Vec3>(triad.at(std::string(var_key))), rng);
    }
    if (json_contains(triad, cov_key)) {
        return sim::draw_normal_cov<3>(cov3_from_json(triad.at(std::string(cov_key))), rng);
    }
    return core::Vec3::Zero();
}

[[nodiscard]] inline core::Vec3 optional_vec3_from_json(const nlohmann::json& triad,
                                                        const std::string_view key)
{
    if (json_contains(triad, key)) {
        return vec3_from_json<core::Vec3>(triad.at(std::string(key)));
    }
    return core::Vec3::Zero();
}

inline sim::ImuTriadErrorConfig imu_triad_config_from_json(const nlohmann::json& triad,
                                                           const ImuTriadRuntimeKeys& keys,
                                                           std::mt19937& rng)
{
    sim::ImuTriadErrorConfig config;
    config.bias_turnon = vec3_direct_or_random_from_json(
        triad, keys.bias_turnon, keys.bias_turnon_var, keys.bias_turnon_cov, rng);
    config.bias_inrun_psd = optional_vec3_from_json(triad, keys.bias_inrun_psd);
    config.output_random_walk_psd = optional_vec3_from_json(triad, keys.output_random_walk_psd);
    config.scale_factor = vec3_direct_or_random_from_json(
        triad, "scale_factor", keys.scale_factor_var, keys.scale_factor_cov, rng);
    config.misalignment_rad = vec3_direct_or_random_from_json(
        triad, "misalignment_rad", keys.misalignment_var, keys.misalignment_cov, rng);
    config.nonorthogonality = vec3_direct_or_random_from_json(
        triad, "nonorthogonality", keys.nonorthogonality_var, keys.nonorthogonality_cov, rng);
    config.quantization = optional_vec3_from_json(triad, keys.quantization);
    config.limit = optional_vec3_from_json(triad, keys.limit);
    return config;
}

[[nodiscard]] inline ImuTriadRuntimeKeys gyro_runtime_keys()
{
    return {.bias_turnon = "bias_turnon_radps",
            .bias_turnon_var = "bias_turnon_var_rad2ps2",
            .bias_turnon_cov = "bias_turnon_cov_rad2ps2",
            .bias_inrun_psd = "bias_inrun_psd_rad2ps3",
            .output_random_walk_psd = "angle_random_walk_psd_rad2ps",
            .scale_factor_var = "scale_factor_var",
            .scale_factor_cov = "scale_factor_cov",
            .misalignment_var = "misalignment_var_rad2",
            .misalignment_cov = "misalignment_cov_rad2",
            .nonorthogonality_var = "nonorthogonality_var",
            .nonorthogonality_cov = "nonorthogonality_cov",
            .quantization = "quantization_rad",
            .limit = "angular_rate_limit_radps"};
}

[[nodiscard]] inline ImuTriadRuntimeKeys accel_runtime_keys()
{
    return {.bias_turnon = "bias_turnon_mps2",
            .bias_turnon_var = "bias_turnon_var_m2ps4",
            .bias_turnon_cov = "bias_turnon_cov_m2ps4",
            .bias_inrun_psd = "bias_inrun_psd_m2ps5",
            .output_random_walk_psd = "velocity_random_walk_psd_m2ps3",
            .scale_factor_var = "scale_factor_var",
            .scale_factor_cov = "scale_factor_cov",
            .misalignment_var = "misalignment_var_rad2",
            .misalignment_cov = "misalignment_cov_rad2",
            .nonorthogonality_var = "nonorthogonality_var",
            .nonorthogonality_cov = "nonorthogonality_cov",
            .quantization = "quantization_mps",
            .limit = "acceleration_limit_mps2"};
}

} // namespace detail

inline void validate_imu_runtime_config(const nlohmann::json& cfg)
{
    const nlohmann::json& imu = detail::require_object(cfg, "imu");
    detail::require_string(imu, "type");
    detail::require_unsigned_integer(imu, "seed");
    validate_runtime_rate(imu, "imu");
    if (!imu.contains("dt_s") && !imu.contains("rate_hz")) {
        detail::throw_runtime_config_error("imu must specify one of 'dt_s' or 'rate_hz'");
    }
    if (imu.contains("sample_rate_hz")) {
        detail::throw_runtime_config_error(
            "imu.sample_rate_hz is unsupported; use 'dt_s' or 'rate_hz'");
    }
    detail::reject_unknown_top_level_keys(imu,
                                          {"type", "seed", "dt_s", "rate_hz", "gyro", "accel"});

    const std::string type = imu.at("type").get<std::string>();
    if (type != "ideal" && type != "error_model") {
        detail::throw_runtime_config_error("imu.type must be 'ideal' or 'error_model'");
    }

    if (type == "ideal") {
        if (imu.contains("gyro") || imu.contains("accel")) {
            detail::throw_runtime_config_error(
                "imu.gyro and imu.accel are only valid when imu.type is 'error_model'");
        }
        return;
    }

    detail::validate_imu_triad_config(
        detail::require_object(imu, "gyro"), "gyro", detail::gyro_runtime_keys());
    detail::validate_imu_triad_config(
        detail::require_object(imu, "accel"), "accel", detail::accel_runtime_keys());
}

inline sim::ImuSimulatorConfig imu_simulator_config_from_json(const nlohmann::json& cfg)
{
    validate_imu_runtime_config(cfg);

    const nlohmann::json& imu = cfg.at("imu");
    sim::ImuSimulatorConfig config;
    config.seed = imu.at("seed").get<unsigned int>();

    const std::string type = imu.at("type").get<std::string>();
    if (type == "ideal") {
        return config;
    }

    std::mt19937 draw_rng(config.seed);
    config.gyro =
        detail::imu_triad_config_from_json(imu.at("gyro"), detail::gyro_runtime_keys(), draw_rng);
    config.accel =
        detail::imu_triad_config_from_json(imu.at("accel"), detail::accel_runtime_keys(), draw_rng);
    return config;
}

} // namespace navkit::app_support
