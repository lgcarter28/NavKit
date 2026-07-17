// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/sim/ImuSimulator.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::app_support
{

namespace detail
{

struct ImuTriadRuntimeKeys
{
    std::string_view bias;
    std::string_view bias_random_walk_psd;
    std::string_view white_noise_psd;
    std::string_view quantization;
};

inline void require_optional_nonnegative_vec3(const nlohmann::json& cfg, std::string_view path)
{
    require_optional_vec3(cfg, path);
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }

    for (const auto& value : *iter) {
        if (value.get<core::Scalar_t>() < 0.0) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be nonnegative");
        }
    }
}

inline void validate_imu_triad_config(const nlohmann::json& triad,
                                      std::string_view name,
                                      const ImuTriadRuntimeKeys& keys)
{
    if (!triad.is_object()) {
        throw_runtime_config_error("expected " + quoted_path(name) + " to be an object");
    }

    require_optional_vec3(triad, keys.bias);
    require_optional_nonnegative_vec3(triad, keys.bias_random_walk_psd);
    require_optional_nonnegative_vec3(triad, keys.white_noise_psd);
    require_optional_vec3(triad, "scale_factor");
    require_optional_vec3(triad, "misalignment_rad");
    require_optional_vec3(triad, "nonorthogonality");
    require_optional_nonnegative_vec3(triad, keys.quantization);
}

inline sim::ImuTriadErrorConfig imu_triad_config_from_json(const nlohmann::json& triad,
                                                           const ImuTriadRuntimeKeys& keys)
{
    sim::ImuTriadErrorConfig config;
    if (triad.contains(std::string(keys.bias))) {
        config.bias = vec3_from_json<core::Vec3>(triad.at(std::string(keys.bias)));
    }
    if (triad.contains(std::string(keys.bias_random_walk_psd))) {
        config.bias_random_walk_psd =
            vec3_from_json<core::Vec3>(triad.at(std::string(keys.bias_random_walk_psd)));
    }
    if (triad.contains(std::string(keys.white_noise_psd))) {
        config.white_noise_psd =
            vec3_from_json<core::Vec3>(triad.at(std::string(keys.white_noise_psd)));
    }
    if (triad.contains("scale_factor")) {
        config.scale_factor = vec3_from_json<core::Vec3>(triad.at("scale_factor"));
    }
    if (triad.contains("misalignment_rad")) {
        config.misalignment_rad = vec3_from_json<core::Vec3>(triad.at("misalignment_rad"));
    }
    if (triad.contains("nonorthogonality")) {
        config.nonorthogonality = vec3_from_json<core::Vec3>(triad.at("nonorthogonality"));
    }
    if (triad.contains(std::string(keys.quantization))) {
        config.quantization = vec3_from_json<core::Vec3>(triad.at(std::string(keys.quantization)));
    }
    return config;
}

} // namespace detail

inline void validate_imu_runtime_config(const nlohmann::json& cfg)
{
    const auto& imu = detail::require_object(cfg, "imu");
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

    const std::string type = imu.at("type").get<std::string>();
    if (type != "ideal" && type != "error_model") {
        detail::throw_runtime_config_error("imu.type must be 'ideal' or 'error_model'");
    }

    if (type == "ideal") {
        return;
    }

    detail::validate_imu_triad_config(detail::require_object(imu, "gyro"),
                                      "gyro",
                                      {.bias = "bias_radps",
                                       .bias_random_walk_psd = "bias_rw_psd_rad2ps3",
                                       .white_noise_psd = "white_noise_psd_rad2ps",
                                       .quantization = "quantization_rad"});
    detail::validate_imu_triad_config(detail::require_object(imu, "accel"),
                                      "accel",
                                      {.bias = "bias_mps2",
                                       .bias_random_walk_psd = "bias_rw_psd_m2ps5",
                                       .white_noise_psd = "white_noise_psd_m2ps3",
                                       .quantization = "quantization_mps"});
}

inline sim::ImuSimulatorConfig imu_simulator_config_from_json(const nlohmann::json& cfg)
{
    validate_imu_runtime_config(cfg);

    const auto& imu = cfg.at("imu");
    sim::ImuSimulatorConfig config;
    config.seed = imu.at("seed").get<unsigned int>();

    const std::string type = imu.at("type").get<std::string>();
    if (type == "ideal") {
        return config;
    }

    config.gyro = detail::imu_triad_config_from_json(imu.at("gyro"),
                                                     {.bias = "bias_radps",
                                                      .bias_random_walk_psd = "bias_rw_psd_rad2ps3",
                                                      .white_noise_psd = "white_noise_psd_rad2ps",
                                                      .quantization = "quantization_rad"});
    config.accel = detail::imu_triad_config_from_json(imu.at("accel"),
                                                      {.bias = "bias_mps2",
                                                       .bias_random_walk_psd = "bias_rw_psd_m2ps5",
                                                       .white_noise_psd = "white_noise_psd_m2ps3",
                                                       .quantization = "quantization_mps"});
    return config;
}

} // namespace navkit::app_support
