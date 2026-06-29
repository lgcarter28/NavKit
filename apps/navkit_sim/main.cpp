#include "navkit/core/Config.hpp"
#include "navkit/core/KalmanFilter.hpp"
#include "navkit/core/Navigator.hpp"
#include "navkit/core/Sensor.hpp"
#include "navkit/core/StateDefs.hpp"
#include "navkit/core/policies/NoisePolicies.hpp"
#include "navkit/core/policies/UpdatePolicies.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "navkit/sim/GnssSimulator.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{

template<typename Vec3>
Vec3 vec3_from_json(const json& j)
{
    Vec3 v;
    v << j.at(0).get<navkit::Scalar_t>(), j.at(1).get<navkit::Scalar_t>(),
        j.at(2).get<navkit::Scalar_t>();
    return v;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const fs::path config_path = (argc > 1)
                                         ? fs::path(argv[1])
                                         : fs::path("apps/navkit_sim/configs/stationary_gnss.json");
        std::ifstream cfg_stream(config_path);
        if (!cfg_stream) {
            std::cerr << "Failed to open config: " << config_path << '\n';
            return 1;
        }

        const json cfg = json::parse(cfg_stream);

        const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
        const fs::path output_dir = cfg.value("output_dir", std::string("data/logs/") + run_name);

        navkit::sim::StationaryTrajectoryConfig traj_cfg;
        traj_cfg.duration_s = cfg["trajectory"].value("duration_s", 60.0);
        traj_cfg.dt_s = cfg["trajectory"].value("dt_s", 1.0);
        traj_cfg.p_e =
            vec3_from_json<Eigen::Matrix<navkit::Scalar_t, 3, 1>>(cfg["trajectory"]["p_e_m"]);

        navkit::sim::GnssSimulatorConfig gnss_cfg;
        gnss_cfg.dt_s = cfg["gnss"].value("dt_s", 1.0);
        gnss_cfg.sigma_h_m = cfg["gnss"].value("sigma_h_m", 3.0);
        gnss_cfg.sigma_v_m = cfg["gnss"].value("sigma_v_m", 5.0);
        gnss_cfg.seed = cfg["gnss"].value("seed", 42U);

        const auto truth = navkit::sim::TrajectoryGenerator::stationary(traj_cfg);
        navkit::sim::GnssSimulator gnss_sim(gnss_cfg);

        using StateDef = navkit::InsStateDef;
        using GnssModel = navkit::GnssPosModel<StateDef>;
        using GnssSensor =
            navkit::Sensor<GnssModel, navkit::Config::GNSS_BUFF_SIZE, navkit::GnssFixedNoisePolicy>;
        using Sensors = std::tuple<GnssSensor>;
        using MeasurementModels = std::tuple<GnssModel>;
        using Filter = navkit::KalmanFilter<StateDef,
                                            navkit::DefaultInjectionPolicy<StateDef>,
                                            navkit::DefaultResetPolicy<StateDef>,
                                            MeasurementModels>;
        using Navigator = navkit::Navigator<Filter, Sensors, navkit::UpdatePostFilter>;

        Navigator navigator;
        auto& filter = navigator.filter();

        Filter::State_t x0 = Filter::State_t::Zero();
        x0.template segment<3>(StateDef::Pos::i) = traj_cfg.p_e;

        if (cfg.contains("filter") && cfg["filter"].contains("initial_position_offset_m")) {
            x0.template segment<3>(StateDef::Pos::i) +=
                vec3_from_json<Eigen::Matrix<navkit::Scalar_t, 3, 1>>(
                    cfg["filter"]["initial_position_offset_m"]);
        }

        filter.set_state(x0);

        Filter::P_t P0 = Filter::P_t::Identity();
        const navkit::Scalar_t sigma_p0 =
            cfg.value("filter", json::object()).value("initial_position_sigma_m", 100.0);
        P0 *= 1.0e-6;
        P0.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) =
            (sigma_p0 * sigma_p0) * Eigen::Matrix<navkit::Scalar_t, 3, 3>::Identity();
        filter.set_covariance(P0);

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

        std::cout << "Wrote NavKit simulation logs to: " << output_dir << '\n';
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "navkit_sim error: " << e.what() << '\n';
        return 1;
    }
}
