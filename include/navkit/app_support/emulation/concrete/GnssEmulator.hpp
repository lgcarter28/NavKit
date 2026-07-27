// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/io/log_payloads/GnssMeasurementLogPayload.hpp"
#include "navkit/io/log_products/GnssPositionDebugLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityDebugLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityLogProduct.hpp"
#include "navkit/sim/GnssSimulator.hpp"

#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::app_support
{

using SensorId = navkit::core::estimation::SensorId;

namespace detail
{

inline void validate_gnss_covariance_config(const nlohmann::json& covariance,
                                            std::string_view path,
                                            std::string_view diag_key);

inline void validate_gnss_runtime_config(const nlohmann::json& cfg, std::string_view runtime_key)
{
    const nlohmann::json& gnss = require_object(cfg, runtime_key);
    validate_runtime_rate(gnss, runtime_key);
    if (!gnss.contains("dt_s") && !gnss.contains("rate_hz")) {
        throw_runtime_config_error("gnss must specify one of 'dt_s' or 'rate_hz'");
    }
    validate_gnss_covariance_config(
        require_object(gnss, "position_cov"), "gnss.position_cov", "pos_m2");
    validate_gnss_covariance_config(
        require_object(gnss, "velocity_cov"), "gnss.velocity_cov", "vel_m2ps2");
    require_optional_vec3(gnss, "p_b_ant_b_m");
    require_unsigned_integer(gnss, "seed");
    require_bool(gnss, "noise_enabled");
}

inline void validate_gnss_covariance_frame(const nlohmann::json& covariance, std::string_view path)
{
    require_string(covariance, "frame");
    const std::string frame = covariance.at("frame").get<std::string>();
    if (frame != "ecef" && frame != "ned") {
        throw_runtime_config_error(std::string(path) + ".frame must be 'ecef' or 'ned'");
    }
}

inline void validate_gnss_covariance_config(const nlohmann::json& covariance,
                                            std::string_view path,
                                            std::string_view diag_key)
{
    validate_gnss_covariance_frame(covariance, path);
    const bool has_diag = covariance.contains("diag");
    const bool has_full = covariance.contains("full");
    if ((has_diag ? 1 : 0) + (has_full ? 1 : 0) != 1) {
        throw_runtime_config_error(std::string(path) +
                                   " must contain exactly one of 'diag' or 'full'");
    }

    if (has_diag) {
        const nlohmann::json& diag = require_object(covariance, "diag");
        require_numeric_array(diag, diag_key, 3U);
        const nlohmann::json& values = diag.at(std::string(diag_key));
        for (const nlohmann::json& value : values) {
            if (value.get<core::Scalar_t>() < 0.0) {
                throw_runtime_config_error(std::string(path) +
                                           ".diag variances must be nonnegative");
            }
        }
        return;
    }

    require_numeric_array(covariance, "full", 9U);
    core::Mat3 R = core::Mat3::Zero();
    const nlohmann::json& values = covariance.at("full");
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            R(row, col) =
                values.at(static_cast<std::size_t>((row * 3) + col)).get<core::Scalar_t>();
        }
    }
    if (!R.isApprox(R.transpose(), 1.0e-12)) {
        throw_runtime_config_error(std::string(path) + ".full must be symmetric");
    }
    const Eigen::SelfAdjointEigenSolver<core::Mat3> eigensolver(R);
    if (eigensolver.info() != Eigen::Success || eigensolver.eigenvalues().minCoeff() < -1.0e-12) {
        throw_runtime_config_error(std::string(path) + ".full must be positive semidefinite");
    }
}

[[nodiscard]] inline sim::GnssCovarianceFrame
gnss_covariance_frame_from_json(const nlohmann::json& covariance)
{
    const std::string frame = covariance.at("frame").get<std::string>();
    if (frame == "ecef") {
        return sim::GnssCovarianceFrame::Ecef;
    }
    return sim::GnssCovarianceFrame::Ned;
}

[[nodiscard]] inline core::Mat3 gnss_covariance_from_json(const nlohmann::json& covariance,
                                                          std::string_view diag_key)
{
    core::Mat3 R = core::Mat3::Zero();
    if (covariance.contains("diag")) {
        const nlohmann::json& values = covariance.at("diag").at(std::string(diag_key));
        for (int i = 0; i < 3; ++i) {
            R(i, i) = values.at(static_cast<std::size_t>(i)).get<core::Scalar_t>();
        }
        return R;
    }

    const nlohmann::json& values = covariance.at("full");
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            R(row, col) =
                values.at(static_cast<std::size_t>((row * 3) + col)).get<core::Scalar_t>();
        }
    }
    return R;
}

[[nodiscard]] inline std::string gnss_covariance_frame_name(const sim::GnssCovarianceFrame frame)
{
    if (frame == sim::GnssCovarianceFrame::Ecef) {
        return "ecef";
    }
    return "ned";
}

[[nodiscard]] inline nlohmann::json covariance_matrix_metadata(const core::Mat3& covariance)
{
    nlohmann::json values = nlohmann::json::array();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            values.push_back(covariance(row, col));
        }
    }
    return values;
}

inline sim::GnssSimulatorConfig gnss_runtime_config_from_json(const nlohmann::json& cfg,
                                                              std::string_view runtime_key)
{
    const nlohmann::json& gnss_config = cfg.at(std::string(runtime_key));
    sim::GnssSimulatorConfig gnss_cfg;
    gnss_cfg.rate = rational_rate_from_required_runtime_rate(gnss_config, runtime_key);
    const nlohmann::json& position_cov = gnss_config.at("position_cov");
    const nlohmann::json& velocity_cov = gnss_config.at("velocity_cov");
    gnss_cfg.position_covariance_frame = gnss_covariance_frame_from_json(position_cov);
    gnss_cfg.position_cov_m2 = gnss_covariance_from_json(position_cov, "pos_m2");
    gnss_cfg.velocity_covariance_frame = gnss_covariance_frame_from_json(velocity_cov);
    gnss_cfg.velocity_cov_m2ps2 = gnss_covariance_from_json(velocity_cov, "vel_m2ps2");
    if (gnss_config.contains("p_b_ant_b_m")) {
        gnss_cfg.p_b_ant_b_m = vec3_from_json<core::Vec3>(gnss_config.at("p_b_ant_b_m"));
    }
    gnss_cfg.seed = gnss_config.at("seed").get<unsigned int>();
    gnss_cfg.noise_enabled = gnss_config.at("noise_enabled").get<bool>();
    return gnss_cfg;
}

inline core::Vec3 omega_eb_b_radps_from_truth(const sim::TruthSample& sample)
{
    const core::Vec3 omega_ie_b =
        sample.q_b2e.conjugate() *
        core::environment::planet_rate_fixed_radps<core::environment::Wgs84>();
    return sample.w_ib_b_radps - omega_ie_b;
}

} // namespace detail

template<SensorId IdValue>
struct GnssEmulator
{
    static constexpr SensorId Id = IdValue;
    static constexpr std::string_view RuntimeKey = "gnss";

    using Runtime = sim::GnssSimulator;
    using RuntimeConfig = sim::GnssSimulatorConfig;

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        detail::validate_gnss_runtime_config(cfg, RuntimeKey);
    }

    static RuntimeConfig runtime_config_from_json(const nlohmann::json& cfg)
    {
        return detail::gnss_runtime_config_from_json(cfg, RuntimeKey);
    }

    static Runtime make_runtime(const nlohmann::json& cfg)
    {
        return Runtime(runtime_config_from_json(cfg));
    }

    static core::RationalRate runtime_rate_from_json(const nlohmann::json& cfg)
    {
        return runtime_config_from_json(cfg).rate;
    }

    template<typename Sensor>
    static void configure_sensor(Sensor& sensor, const nlohmann::json& cfg)
    {
        const RuntimeConfig gnss_cfg = runtime_config_from_json(cfg);
        sensor.observation_context().p_b_ant_b_m = gnss_cfg.p_b_ant_b_m;
        if (gnss_cfg.position_covariance_frame == sim::GnssCovarianceFrame::Ecef) {
            sensor.observation_context().R_e_m2 = gnss_cfg.position_cov_m2;
        }
        // NED covariance requires the current truth position; update_sensor_context() transforms it
        // to ECEF immediately before the measurement update.
    }

    template<navkit::io::LoggerProductAccessPolicy<navkit::io::GnssPositionLogProduct> Logger>
    static void configure_logger(Logger& logger, const nlohmann::json& cfg)
    {
        const RuntimeConfig gnss_cfg = runtime_config_from_json(cfg);
        logger.template product<navkit::io::GnssPositionLogProduct>().set_metadata(
            detail::gnss_covariance_frame_name(gnss_cfg.position_covariance_frame),
            detail::covariance_matrix_metadata(gnss_cfg.position_cov_m2),
            gnss_cfg.seed);
    }

    static core::estimation::Measurement<3> generate(Runtime& runtime,
                                                     const sim::TruthSample& sample)
    {
        return runtime.generate_position(sample);
    }

    static bool should_generate(const Runtime& runtime, const sim::TruthSample& sample)
    {
        return runtime.should_generate(sample);
    }

    template<typename Logger, typename Measurement>
    static void log_measurement(Logger& logger, const Measurement& measurement)
    {
        logger.log(navkit::io::GnssPositionLogPayload{.measurement = measurement});
    }

    template<typename Sensor, typename Measurement>
    static void update_sensor_context(const Runtime& runtime,
                                      const sim::TruthSample& sample,
                                      const Measurement&,
                                      Sensor& sensor)
    {
        sensor.observation_context().R_e_m2 = runtime.position_cov_e_m2(sample);
    }

    template<typename Logger, typename Measurement>
    static void log_sample(const Runtime& runtime,
                           const sim::TruthSample& sample,
                           const Measurement& measurement,
                           Logger& logger)
    {
        log_measurement(logger, measurement);
        if constexpr (navkit::io::LoggerPayloadPolicy<Logger,
                                                      navkit::io::GnssPositionDebugLogPayload>) {
            const RuntimeConfig& gnss_cfg = runtime.config();
            const core::Vec3 truth_p_e_m = sample.p_e + (sample.q_b2e * gnss_cfg.p_b_ant_b_m);
            const core::Mat3 R_e_m2 = runtime.position_cov_e_m2(sample);
            logger.log(navkit::io::GnssPositionDebugLogPayload{
                .time_s = core::timestamp_seconds(measurement.t),
                .truth_p_e_m = truth_p_e_m,
                .measured_p_e_m = measurement.z,
                .sigma_p_e_m = R_e_m2.diagonal().cwiseSqrt(),
            });
        }
    }
};

template<SensorId IdValue>
struct GnssVelocityEmulator
{
    static constexpr SensorId Id = IdValue;
    static constexpr std::string_view RuntimeKey = "gnss";

    using Runtime = sim::GnssSimulator;
    using RuntimeConfig = sim::GnssSimulatorConfig;

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        detail::validate_gnss_runtime_config(cfg, RuntimeKey);
    }

    static RuntimeConfig runtime_config_from_json(const nlohmann::json& cfg)
    {
        return detail::gnss_runtime_config_from_json(cfg, RuntimeKey);
    }

    static Runtime make_runtime(const nlohmann::json& cfg)
    {
        return Runtime(runtime_config_from_json(cfg));
    }

    static core::RationalRate runtime_rate_from_json(const nlohmann::json& cfg)
    {
        return runtime_config_from_json(cfg).rate;
    }

    template<typename Sensor>
    static void configure_sensor(Sensor& sensor, const nlohmann::json& cfg)
    {
        const RuntimeConfig gnss_cfg = runtime_config_from_json(cfg);
        sensor.observation_context().p_b_ant_b_m = gnss_cfg.p_b_ant_b_m;
        if (gnss_cfg.velocity_covariance_frame == sim::GnssCovarianceFrame::Ecef) {
            sensor.observation_context().R_e_m2ps2 = gnss_cfg.velocity_cov_m2ps2;
        }
        // NED covariance requires the current truth position; update_sensor_context() transforms it
        // to ECEF immediately before the measurement update.
    }

    template<navkit::io::LoggerProductAccessPolicy<navkit::io::GnssVelocityLogProduct> Logger>
    static void configure_logger(Logger& logger, const nlohmann::json& cfg)
    {
        const RuntimeConfig gnss_cfg = runtime_config_from_json(cfg);
        logger.template product<navkit::io::GnssVelocityLogProduct>().set_metadata(
            detail::gnss_covariance_frame_name(gnss_cfg.velocity_covariance_frame),
            detail::covariance_matrix_metadata(gnss_cfg.velocity_cov_m2ps2),
            gnss_cfg.seed);
    }

    static core::estimation::Measurement<3> generate(Runtime& runtime,
                                                     const sim::TruthSample& sample)
    {
        return runtime.generate_velocity(sample);
    }

    template<typename Sensor, typename Measurement>
    static void update_sensor_context(const Runtime& runtime,
                                      const sim::TruthSample& sample,
                                      const Measurement&,
                                      Sensor& sensor)
    {
        sensor.observation_context().omega_eb_b_radps = detail::omega_eb_b_radps_from_truth(sample);
        sensor.observation_context().R_e_m2ps2 = runtime.velocity_cov_e_m2ps2(sample);
    }

    static bool should_generate(const Runtime& runtime, const sim::TruthSample& sample)
    {
        return runtime.should_generate(sample);
    }

    template<typename Logger, typename Measurement>
    static void log_measurement(Logger& logger, const Measurement& measurement)
    {
        logger.log(navkit::io::GnssVelocityLogPayload{.measurement = measurement});
    }

    template<typename Logger, typename Measurement>
    static void log_sample(const Runtime& runtime,
                           const sim::TruthSample& sample,
                           const Measurement& measurement,
                           Logger& logger)
    {
        log_measurement(logger, measurement);
        if constexpr (navkit::io::LoggerPayloadPolicy<Logger,
                                                      navkit::io::GnssVelocityDebugLogPayload>) {
            const RuntimeConfig& gnss_cfg = runtime.config();
            const core::Vec3 omega_eb_b_radps = detail::omega_eb_b_radps_from_truth(sample);
            const core::Vec3 truth_v_e_mps =
                sample.v_e + (sample.q_b2e * omega_eb_b_radps.cross(gnss_cfg.p_b_ant_b_m));
            const core::Mat3 R_e_m2ps2 = runtime.velocity_cov_e_m2ps2(sample);
            const core::Vec3 sigma_v_e_mps = R_e_m2ps2.diagonal().cwiseSqrt();
            logger.log(navkit::io::GnssVelocityDebugLogPayload{
                .time_s = core::timestamp_seconds(measurement.t),
                .truth_v_e_mps = truth_v_e_mps,
                .measured_v_e_mps = measurement.z,
                .sigma_v_e_mps = sigma_v_e_mps,
            });
        }
    }
};

} // namespace navkit::app_support
