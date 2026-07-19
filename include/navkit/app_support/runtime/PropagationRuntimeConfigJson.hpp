// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support::detail
{

inline void reject_unknown_object_keys(const nlohmann::json& object,
                                       const std::string_view object_path,
                                       const std::vector<std::string_view>& allowed_keys)
{
    for (nlohmann::json::const_iterator iter = object.begin(); iter != object.end(); ++iter) {
        const std::string& key = iter.key();
        if (!contains_key(allowed_keys, key)) {
            throw_runtime_config_error("unknown key '" + key + "' in " + quoted_path(object_path));
        }
    }
}

inline void require_nonnegative_vec3(const nlohmann::json& cfg, const std::string_view path)
{
    require_vec3(cfg, path);
    const nlohmann::json& values = cfg.at(std::string(path));
    for (const nlohmann::json& value : values) {
        if (value.template get<navkit::core::Scalar_t>() < 0.0) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be nonnegative");
        }
    }
}

[[nodiscard]] inline navkit::core::Vec3 vec3_field_from_json(const nlohmann::json& cfg,
                                                             const std::string_view path)
{
    const nlohmann::json& values = cfg.at(std::string(path));
    return navkit::core::Vec3{values.at(0U).template get<navkit::core::Scalar_t>(),
                              values.at(1U).template get<navkit::core::Scalar_t>(),
                              values.at(2U).template get<navkit::core::Scalar_t>()};
}

inline void validate_runtime_process_noise_shape(const nlohmann::json& process_noise)
{
    reject_unknown_object_keys(process_noise,
                               "propagation.process_noise",
                               {"gyro_white_noise_psd_rad2ps",
                                "accel_white_noise_psd_m2ps3",
                                "gyro_bias_drive_psd_rad2ps3",
                                "accel_bias_drive_psd_m2ps5"});
    require_nonnegative_vec3(process_noise, "gyro_white_noise_psd_rad2ps");
    require_nonnegative_vec3(process_noise, "accel_white_noise_psd_m2ps3");
    require_nonnegative_vec3(process_noise, "gyro_bias_drive_psd_rad2ps3");
    require_nonnegative_vec3(process_noise, "accel_bias_drive_psd_m2ps5");
}

inline void validate_runtime_imu_bias_dynamics_shape(const nlohmann::json& imu_bias_dynamics)
{
    reject_unknown_object_keys(
        imu_bias_dynamics,
        "propagation.imu_bias_dynamics",
        {"gyro_bias_correlation_rate_1ps", "accel_bias_correlation_rate_1ps"});
    require_nonnegative_vec3(imu_bias_dynamics, "gyro_bias_correlation_rate_1ps");
    require_nonnegative_vec3(imu_bias_dynamics, "accel_bias_correlation_rate_1ps");
}

template<typename Propagation>
inline void validate_runtime_propagation_config_shape(const nlohmann::json& cfg)
{
    const nlohmann::json::const_iterator propagation_iter = cfg.find("propagation");
    if (propagation_iter == cfg.end()) {
        return;
    }
    if (!propagation_iter->is_object()) {
        throw_runtime_config_error("expected 'propagation' to be an object");
    }
    reject_unknown_object_keys(
        *propagation_iter, "propagation", {"process_noise", "imu_bias_dynamics"});

    const nlohmann::json::const_iterator process_noise_iter =
        propagation_iter->find("process_noise");
    if (process_noise_iter != propagation_iter->end()) {
        if (!process_noise_iter->is_object()) {
            throw_runtime_config_error("expected 'propagation.process_noise' to be an object");
        }
        validate_runtime_process_noise_shape(*process_noise_iter);
    }

    const nlohmann::json::const_iterator bias_dynamics_iter =
        propagation_iter->find("imu_bias_dynamics");
    if (bias_dynamics_iter != propagation_iter->end()) {
        if (!bias_dynamics_iter->is_object()) {
            throw_runtime_config_error("expected 'propagation.imu_bias_dynamics' to be an object");
        }
        validate_runtime_imu_bias_dynamics_shape(*bias_dynamics_iter);
    }
}

template<typename Propagation>
[[nodiscard]] inline typename Propagation::RuntimeConfig_t propagation_runtime_config_from_json(
    const nlohmann::json& cfg, const typename Propagation::RuntimeConfig_t& compile_time_config)
{
    typename Propagation::RuntimeConfig_t config = compile_time_config;
    const nlohmann::json::const_iterator propagation_iter = cfg.find("propagation");
    if (propagation_iter == cfg.end() || !propagation_iter->is_object()) {
        return config;
    }

    validate_runtime_propagation_config_shape<Propagation>(cfg);

    const nlohmann::json::const_iterator process_noise_iter =
        propagation_iter->find("process_noise");
    if (process_noise_iter != propagation_iter->end()) {
        config.process_noise = typename Propagation::ProcessNoise_t{
            .gyro_white_noise_psd_rad2ps =
                vec3_field_from_json(*process_noise_iter, "gyro_white_noise_psd_rad2ps"),
            .accel_white_noise_psd_m2ps3 =
                vec3_field_from_json(*process_noise_iter, "accel_white_noise_psd_m2ps3"),
            .gyro_bias_drive_psd_rad2ps3 =
                vec3_field_from_json(*process_noise_iter, "gyro_bias_drive_psd_rad2ps3"),
            .accel_bias_drive_psd_m2ps5 =
                vec3_field_from_json(*process_noise_iter, "accel_bias_drive_psd_m2ps5"),
        };
    }

    const nlohmann::json::const_iterator bias_dynamics_iter =
        propagation_iter->find("imu_bias_dynamics");
    if (bias_dynamics_iter != propagation_iter->end()) {
        config.imu_bias_dynamics = typename Propagation::ImuBiasDynamics_t{
            .gyro_bias_correlation_rate_1ps =
                vec3_field_from_json(*bias_dynamics_iter, "gyro_bias_correlation_rate_1ps"),
            .accel_bias_correlation_rate_1ps =
                vec3_field_from_json(*bias_dynamics_iter, "accel_bias_correlation_rate_1ps"),
        };
    }

    return config;
}

} // namespace navkit::app_support::detail
