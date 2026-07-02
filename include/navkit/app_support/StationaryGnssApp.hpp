// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/ConfigTraits.hpp"
#include "navkit/app_support/EstimationAliases.hpp"
#include "navkit/app_support/JsonInput.hpp"
#include "navkit/app_support/ProfileExport.hpp"
#include "navkit/app_support/RuntimeConfigValidation.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicies.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/GnssSimulator.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"

#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>

namespace navkit::app_support
{

template<typename AppConfig>
class StationaryGnssApp
{
public:
    using NavKit = NavKitConfig_t<AppConfig>;
    using StateDef = core::estimation::InsStateDef;
    using GnssModel = core::models::GnssPosModel<StateDef>;
    using Profiler = ConfigProfiler_t<NavKit>;
    using GnssSensor =
        Sensor_t<GnssModel, NavKit::GnssBuffer::BufferSize, core::estimation::GnssFixedNoisePolicy>;
    using Sensors = std::tuple<GnssSensor>;
    using MeasurementModels = std::tuple<GnssModel>;
    using Filter = DefaultKalmanFilter_t<StateDef, MeasurementModels, Profiler>;
    using Navigator = UpdatePostFilterNavigator_t<Filter, Sensors, Profiler>;

    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        validate_stationary_gnss_runtime_config<AppConfig>(cfg);

        const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
        const std::filesystem::path output_dir =
            cfg.value("output_dir", std::string("data/logs/") + run_name);

        const auto traj_cfg = stationary_trajectory_config_from_json(cfg);
        const auto gnss_cfg = gnss_simulator_config_from_json(cfg);

        const auto truth = sim::TrajectoryGenerator::stationary(traj_cfg);
        sim::GnssSimulator gnss_sim(gnss_cfg);

        reset_profile_sink_if_configured<NavKit>();

        Navigator navigator;
        auto& filter = navigator.filter();

        configure_initial_filter_state(filter, cfg, traj_cfg.p_e);

        auto& gnss_sensor = navigator.template sensor<0>();
        gnss_sensor.noise_context().sigma_h = gnss_cfg.sigma_h_m;
        gnss_sensor.noise_context().sigma_v = gnss_cfg.sigma_v_m;

        io::RunLogger logger(output_dir, run_name, cfg);
        logger.set_gnss_metadata(gnss_cfg.sigma_h_m, gnss_cfg.sigma_v_m, gnss_cfg.seed);

        for (const auto& sample : truth) {
            logger.log_truth(sample);

            const auto meas = gnss_sim.generate(sample);
            logger.log_gnss(meas);

            if (!gnss_sensor.push(meas)) {
                throw std::runtime_error("GNSS sensor buffer overflow");
            }

            navigator.process_measurements();

            if (filter.template has_measurement_statistics<GnssModel>()) {
                logger.log_gnss_pos_statistics(filter.template measurement_statistics<GnssModel>());
            }

            logger.log_nav<StateDef>(sample.time, filter, sample);
        }

        logger.close();
        export_profile_if_configured<NavKit>(output_dir, run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n", output_dir.string().c_str());
        return 0;
    }

private:
    static sim::StationaryTrajectoryConfig
    stationary_trajectory_config_from_json(const nlohmann::json& cfg)
    {
        const auto& trajectory_config = cfg.at("trajectory");
        sim::StationaryTrajectoryConfig traj_cfg;
        traj_cfg.duration_s = trajectory_config.value("duration_s", 60.0);
        traj_cfg.dt_s = trajectory_config.value("dt_s", 1.0);
        traj_cfg.p_e =
            vec3_from_json<Eigen::Matrix<core::Scalar_t, 3, 1>>(trajectory_config.at("p_e_m"));
        return traj_cfg;
    }

    static sim::GnssSimulatorConfig gnss_simulator_config_from_json(const nlohmann::json& cfg)
    {
        const auto& gnss_config = cfg.at("gnss");
        sim::GnssSimulatorConfig gnss_cfg;
        gnss_cfg.dt_s = gnss_config.value("dt_s", 1.0);
        gnss_cfg.sigma_h_m = gnss_config.value("sigma_h_m", 3.0);
        gnss_cfg.sigma_v_m = gnss_config.value("sigma_v_m", 5.0);
        gnss_cfg.seed = gnss_config.value("seed", 42U);
        return gnss_cfg;
    }

    static void configure_initial_filter_state(Filter& filter,
                                               const nlohmann::json& cfg,
                                               const Eigen::Matrix<core::Scalar_t, 3, 1>& p_e)
    {
        typename Filter::State_t initial_state = Filter::State_t::Zero();
        initial_state.template segment<3>(StateDef::Pos::i) = p_e;

        const auto filter_config = cfg.find("filter");
        if (filter_config != cfg.end() && filter_config->contains("initial_position_offset_m")) {
            initial_state.template segment<3>(StateDef::Pos::i) +=
                vec3_from_json<Eigen::Matrix<core::Scalar_t, 3, 1>>(
                    filter_config->at("initial_position_offset_m"));
        }

        filter.set_state(initial_state);

        typename Filter::P_t initial_covariance = Filter::P_t::Identity();
        const core::Scalar_t sigma_p0 =
            cfg.value("filter", nlohmann::json::object()).value("initial_position_sigma_m", 100.0);
        initial_covariance *= 1.0e-6;
        initial_covariance.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) =
            (sigma_p0 * sigma_p0) * Eigen::Matrix<core::Scalar_t, 3, 3>::Identity();
        filter.set_covariance(initial_covariance);
    }
};

} // namespace navkit::app_support
