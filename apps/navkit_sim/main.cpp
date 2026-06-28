#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

#include <nlohmann/json.hpp>

#include "navkit/core/Config.hpp"
#include "navkit/core/KalmanFilter.hpp"
#include "navkit/core/Navigator.hpp"
#include "navkit/core/Sensor.hpp"
#include "navkit/core/StateDefs.hpp"
#include "navkit/core/policies/NoisePolicies.hpp"
#include "navkit/core/policies/UpdatePolicies.hpp"
#include "navkit/models/GnssPosModel.hpp"
#include "navkit/sim/GnssSimulator.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace
{

    template <typename Vec3>
    Vec3 vec3_from_json(const json &j)
    {
        Vec3 v;
        v << j.at(0).get<navkit::Scalar_t>(), j.at(1).get<navkit::Scalar_t>(), j.at(2).get<navkit::Scalar_t>();
        return v;
    }

    void write_json_file(const fs::path &path, const json &j)
    {
        std::ofstream f(path);
        f << std::setw(2) << j << '\n';
    }

} // namespace

int main(int argc, char **argv)
{
    try
    {
        const fs::path config_path = (argc > 1) ? fs::path(argv[1]) : fs::path("apps/navkit_sim/configs/stationary_gnss.json");
        std::ifstream cfg_stream(config_path);
        if (!cfg_stream)
        {
            std::cerr << "Failed to open config: " << config_path << '\n';
            return 1;
        }
        const json cfg = json::parse(cfg_stream);

        const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
        const fs::path output_dir = cfg.value("output_dir", std::string("data/logs/") + run_name);
        fs::create_directories(output_dir);

        navkit::sim::StationaryTrajectoryConfig traj_cfg;
        traj_cfg.duration_s = cfg["trajectory"].value("duration_s", 60.0);
        traj_cfg.dt_s = cfg["trajectory"].value("dt_s", 1.0);
        traj_cfg.p_e = vec3_from_json<Eigen::Matrix<navkit::Scalar_t, 3, 1>>(cfg["trajectory"]["p_e_m"]);

        navkit::sim::GnssSimulatorConfig gnss_cfg;
        gnss_cfg.dt_s = cfg["gnss"].value("dt_s", 1.0);
        gnss_cfg.sigma_h_m = cfg["gnss"].value("sigma_h_m", 3.0);
        gnss_cfg.sigma_v_m = cfg["gnss"].value("sigma_v_m", 5.0);
        gnss_cfg.seed = cfg["gnss"].value("seed", 42U);

        const auto truth = navkit::sim::TrajectoryGenerator::stationary(traj_cfg);
        navkit::sim::GnssSimulator gnss_sim(gnss_cfg);

        using StateDef = navkit::InsStateDef;
        using GnssModel = navkit::GnssPosModel<StateDef>;
        using GnssSensor = navkit::Sensor<GnssModel, navkit::Config::GNSS_BUFF_SIZE, navkit::GnssFixedNoisePolicy>;
        using Sensors = std::tuple<GnssSensor>;
        using Filter = navkit::KalmanFilter<StateDef>;
        using Navigator = navkit::Navigator<Filter, Sensors, navkit::UpdatePostFilter>;

        Navigator navigator;
        auto &filter = navigator.filter();
        Filter::State_t x0 = Filter::State_t::Zero();
        x0.template segment<3>(StateDef::Pos::i) = traj_cfg.p_e;
        if (cfg.contains("filter") && cfg["filter"].contains("initial_position_offset_m"))
        {
            x0.template segment<3>(StateDef::Pos::i) += vec3_from_json<Eigen::Matrix<navkit::Scalar_t, 3, 1>>(cfg["filter"]["initial_position_offset_m"]);
        }
        filter.set_state(x0);
        Filter::P_t P0 = Filter::P_t::Identity();
        const navkit::Scalar_t sigma_p0 = cfg.value("filter", json::object()).value("initial_position_sigma_m", 100.0);
        P0 *= 1.0e-6;
        P0.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) = (sigma_p0 * sigma_p0) * Eigen::Matrix<navkit::Scalar_t, 3, 3>::Identity();
        filter.set_covariance(P0);

        const fs::path truth_csv_path = output_dir / "truth.csv";
        const fs::path gnss_csv_path = output_dir / "gnss.csv";
        const fs::path nav_csv_path = output_dir / "nav.csv";

        std::ofstream truth_csv(truth_csv_path);
        std::ofstream gnss_csv(gnss_csv_path);
        std::ofstream nav_csv(nav_csv_path);

        truth_csv << "time_s,p_e_x_m,p_e_y_m,p_e_z_m,v_e_x_mps,v_e_y_mps,v_e_z_mps,a_e_x_mps2,a_e_y_mps2,a_e_z_mps2,q_eb_w,q_eb_x,q_eb_y,q_eb_z,w_ib_b_x_radps,w_ib_b_y_radps,w_ib_b_z_radps\n";
        gnss_csv << "time_s,p_e_x_m,p_e_y_m,p_e_z_m\n";
        nav_csv << "time_s,p_e_x_m,p_e_y_m,p_e_z_m,err_p_e_x_m,err_p_e_y_m,err_p_e_z_m,sigma_p_e_x_m,sigma_p_e_y_m,sigma_p_e_z_m\n";

        truth_csv << std::setprecision(17);
        gnss_csv << std::setprecision(17);
        nav_csv << std::setprecision(17);

        auto &gnss_sensor = navigator.template sensor<0>();
        gnss_sensor.noise_context().sigma_h = gnss_cfg.sigma_h_m;
        gnss_sensor.noise_context().sigma_v = gnss_cfg.sigma_v_m;

        for (const auto &sample : truth)
        {
            truth_csv << sample.time << ','
                      << sample.p_e.x() << ',' << sample.p_e.y() << ',' << sample.p_e.z() << ','
                      << sample.v_e.x() << ',' << sample.v_e.y() << ',' << sample.v_e.z() << ','
                      << sample.a_e.x() << ',' << sample.a_e.y() << ',' << sample.a_e.z() << ','
                      << sample.q_eb.w() << ',' << sample.q_eb.x() << ',' << sample.q_eb.y() << ',' << sample.q_eb.z() << ','
                      << sample.w_ib_b.x() << ',' << sample.w_ib_b.y() << ',' << sample.w_ib_b.z() << '\n';

            const auto meas = gnss_sim.generate(sample);
            gnss_csv << meas.time << ',' << meas.z.x() << ',' << meas.z.y() << ',' << meas.z.z() << '\n';
            (void)gnss_sensor.push(meas);
            navigator.process_measurements();

            const auto p_est = filter.state().template segment<3>(StateDef::Pos::i);
            const auto err = p_est - sample.p_e;
            const auto &P = filter.covariance();
            nav_csv << sample.time << ','
                    << p_est.x() << ',' << p_est.y() << ',' << p_est.z() << ','
                    << err.x() << ',' << err.y() << ',' << err.z() << ','
                    << std::sqrt(P(StateDef::Pos::i + 0, StateDef::Pos::i + 0)) << ','
                    << std::sqrt(P(StateDef::Pos::i + 1, StateDef::Pos::i + 1)) << ','
                    << std::sqrt(P(StateDef::Pos::i + 2, StateDef::Pos::i + 2)) << '\n';
        }

        const json truth_meta = {
            {"schema", "truth_v1"},
            {"file", "truth.csv"},
            {"units", {{"time", "s"}, {"p_e", "m"}, {"v_e", "m/s"}, {"a_e", "m/s^2"}, {"q_eb", "unit quaternion"}, {"w_ib_b", "rad/s"}}},
            {"frame_convention", "Groves-style; q_eb transforms b-frame components to e-frame components"}};
        const json gnss_meta = {
            {"schema", "gnss_pos_v1"},
            {"file", "gnss.csv"},
            {"units", {{"time", "s"}, {"p_e", "m"}}},
            {"noise", {{"sigma_h_m", gnss_cfg.sigma_h_m}, {"sigma_v_m", gnss_cfg.sigma_v_m}, {"seed", gnss_cfg.seed}}}};
        const json nav_meta = {
            {"schema", "nav_estimate_v1"},
            {"file", "nav.csv"},
            {"units", {{"time", "s"}, {"p_e", "m"}, {"err_p_e", "m"}, {"sigma_p_e", "m"}}}};
        write_json_file(output_dir / "truth.meta.json", truth_meta);
        write_json_file(output_dir / "gnss.meta.json", gnss_meta);
        write_json_file(output_dir / "nav.meta.json", nav_meta);
        write_json_file(output_dir / "run_manifest.json", {{"run_name", run_name},
                                                           {"config", cfg},
                                                           {"logs", {{"truth", {{"csv", "truth.csv"}, {"manifest", "truth.meta.json"}}}, {"gnss", {{"csv", "gnss.csv"}, {"manifest", "gnss.meta.json"}}}, {"nav", {{"csv", "nav.csv"}, {"manifest", "nav.meta.json"}}}}}});

        std::cout << "Wrote NavKit simulation logs to: " << output_dir << '\n';
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "navkit_sim error: " << e.what() << '\n';
        return 1;
    }
}
