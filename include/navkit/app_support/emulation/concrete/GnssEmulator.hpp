// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/runtime/RuntimeRate.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/sim/GnssSimulator.hpp"

#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::app_support
{

using SensorId = navkit::core::estimation::SensorId;

template<SensorId IdValue>
struct GnssEmulator
{
    static constexpr SensorId Id = IdValue;
    static constexpr std::string_view RuntimeKey = "gnss";

    using Runtime = sim::GnssSimulator;
    using RuntimeConfig = sim::GnssSimulatorConfig;

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const auto& gnss = detail::require_object(cfg, RuntimeKey);
        validate_runtime_rate(gnss, RuntimeKey);
        if (!gnss.contains("dt_s") && !gnss.contains("rate_hz")) {
            detail::throw_runtime_config_error("gnss must specify one of 'dt_s' or 'rate_hz'");
        }
        detail::require_nonnegative_number(gnss, "sigma_h_m");
        detail::require_nonnegative_number(gnss, "sigma_v_m");
        detail::require_unsigned_integer(gnss, "seed");
        detail::require_bool(gnss, "noise_enabled");
    }

    static RuntimeConfig runtime_config_from_json(const nlohmann::json& cfg)
    {
        const auto& gnss_config = cfg.at(RuntimeKey);
        RuntimeConfig gnss_cfg;
        gnss_cfg.dt_s = dt_s_from_required_runtime_rate(gnss_config, RuntimeKey);
        gnss_cfg.sigma_h_m = gnss_config.at("sigma_h_m").get<core::Scalar_t>();
        gnss_cfg.sigma_v_m = gnss_config.at("sigma_v_m").get<core::Scalar_t>();
        gnss_cfg.seed = gnss_config.at("seed").get<unsigned int>();
        gnss_cfg.noise_enabled = gnss_config.at("noise_enabled").get<bool>();
        return gnss_cfg;
    }

    static Runtime make_runtime(const nlohmann::json& cfg)
    {
        return Runtime(runtime_config_from_json(cfg));
    }

    template<typename Sensor>
    static void configure_sensor(Sensor& sensor, const nlohmann::json& cfg)
    {
        const auto gnss_cfg = runtime_config_from_json(cfg);
        sensor.noise_context().sigma_h = gnss_cfg.sigma_h_m;
        sensor.noise_context().sigma_v = gnss_cfg.sigma_v_m;
    }

    template<navkit::io::LoggerProductAccessPolicy<navkit::io::GnssPositionLogProduct> Logger>
    static void configure_logger(Logger& logger, const nlohmann::json& cfg)
    {
        const auto gnss_cfg = runtime_config_from_json(cfg);
        logger.template product<navkit::io::GnssPositionLogProduct>().set_metadata(
            gnss_cfg.sigma_h_m, gnss_cfg.sigma_v_m, gnss_cfg.seed);
    }

    static auto generate(Runtime& runtime, const sim::TruthSample& sample)
    {
        return runtime.generate(sample);
    }

    static bool should_generate(const Runtime& runtime, const sim::TruthSample& sample)
    {
        return runtime.should_generate(sample);
    }

    template<typename Logger, typename Measurement>
        requires navkit::io::LoggerPayloadPolicy<Logger, Measurement>
    static void log_measurement(Logger& logger, const Measurement& measurement)
    {
        logger.log(measurement);
    }
};

} // namespace navkit::app_support
