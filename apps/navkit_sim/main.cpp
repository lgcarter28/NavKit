// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/SelectedConfig.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicies.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/GnssSimulator.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{

template<typename Vec3>
Vec3 vec3_from_json(const json& value)
{
    Vec3 v;
    v << value.at(0).get<navkit::core::Scalar_t>(), value.at(1).get<navkit::core::Scalar_t>(),
        value.at(2).get<navkit::core::Scalar_t>();
    return v;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const fs::path config_path =
            (argc > 1) ? fs::path(argv[1])
                       : fs::path("config/runtime/navkit_sim/stationary_gnss.json");
        std::ifstream cfg_stream(config_path);
        if (!cfg_stream) {
            std::fprintf(stderr, "Failed to open config: %s\n", config_path.string().c_str());
            return 1;
        }

        const json cfg = json::parse(cfg_stream);

        const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
        const fs::path output_dir = cfg.value("output_dir", std::string("data/logs/") + run_name);

        const auto& trajectory_config = cfg.at("trajectory");
        navkit::sim::StationaryTrajectoryConfig traj_cfg;
        traj_cfg.duration_s = trajectory_config.value("duration_s", 60.0);
        traj_cfg.dt_s = trajectory_config.value("dt_s", 1.0);
        traj_cfg.p_e = vec3_from_json<Eigen::Matrix<navkit::core::Scalar_t, 3, 1>>(
            trajectory_config.at("p_e_m"));

        const auto& gnss_config = cfg.at("gnss");
        navkit::sim::GnssSimulatorConfig gnss_cfg;
        gnss_cfg.dt_s = gnss_config.value("dt_s", 1.0);
        gnss_cfg.sigma_h_m = gnss_config.value("sigma_h_m", 3.0);
        gnss_cfg.sigma_v_m = gnss_config.value("sigma_v_m", 5.0);
        gnss_cfg.seed = gnss_config.value("seed", 42U);

        const auto truth = navkit::sim::TrajectoryGenerator::stationary(traj_cfg);
        navkit::sim::GnssSimulator gnss_sim(gnss_cfg);

        using StateDef = navkit::core::estimation::InsStateDef;
        using GnssModel = navkit::core::models::GnssPosModel<StateDef>;
        using AppConfig = navkit::selected_config::Config;
        using GnssSensor =
            navkit::core::estimation::Sensor<GnssModel,
                                             AppConfig::GnssBuffer::BufferSize,
                                             navkit::core::estimation::GnssFixedNoisePolicy>;
        using Sensors = std::tuple<GnssSensor>;
        using MeasurementModels = std::tuple<GnssModel>;
        using Filter = navkit::core::estimation::KalmanFilter<
            StateDef,
            navkit::core::estimation::DefaultInjectionPolicy<StateDef>,
            navkit::core::estimation::DefaultResetPolicy<StateDef>,
            MeasurementModels>;
        using Navigator = navkit::core::estimation::
            Navigator<Filter, Sensors, navkit::core::estimation::UpdatePostFilter>;

        Navigator navigator;
        auto& filter = navigator.filter();

        Filter::State_t initial_state = Filter::State_t::Zero();
        initial_state.template segment<3>(StateDef::Pos::i) = traj_cfg.p_e;

        const auto filter_config = cfg.find("filter");
        if (filter_config != cfg.end() && filter_config->contains("initial_position_offset_m")) {
            initial_state.template segment<3>(StateDef::Pos::i) +=
                vec3_from_json<Eigen::Matrix<navkit::core::Scalar_t, 3, 1>>(
                    filter_config->at("initial_position_offset_m"));
        }

        filter.set_state(initial_state);

        Filter::P_t initial_covariance = Filter::P_t::Identity();
        const navkit::core::Scalar_t sigma_p0 =
            cfg.value("filter", json::object()).value("initial_position_sigma_m", 100.0);
        initial_covariance *= 1.0e-6;
        initial_covariance.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) =
            (sigma_p0 * sigma_p0) * Eigen::Matrix<navkit::core::Scalar_t, 3, 3>::Identity();
        filter.set_covariance(initial_covariance);

        auto& gnss_sensor = navigator.template sensor<0>();
        gnss_sensor.noise_context().sigma_h = gnss_cfg.sigma_h_m;
        gnss_sensor.noise_context().sigma_v = gnss_cfg.sigma_v_m;

        navkit::io::RunLogger logger(output_dir, run_name, cfg);
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

        std::printf("Wrote NavKit simulation logs to: %s\n", output_dir.string().c_str());
        return 0;
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "navkit_sim error: %s\n", e.what());
        return 1;
    }
}
