#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace navkit::io
{

class RunLogger
{
public:
    RunLogger(std::filesystem::path output_dir, std::string run_name, nlohmann::json config)
        : m_output_dir(std::move(output_dir))
        , m_run_name(std::move(run_name))
        , m_config(std::move(config))
    {
        std::filesystem::create_directories(m_output_dir);

        m_truth_csv.open(m_output_dir / "truth.csv",
                         {"time_s",
                          "p_e_x_m",
                          "p_e_y_m",
                          "p_e_z_m",
                          "v_e_x_mps",
                          "v_e_y_mps",
                          "v_e_z_mps",
                          "a_e_x_mps2",
                          "a_e_y_mps2",
                          "a_e_z_mps2",
                          "q_eb_w",
                          "q_eb_x",
                          "q_eb_y",
                          "q_eb_z",
                          "w_ib_b_x_radps",
                          "w_ib_b_y_radps",
                          "w_ib_b_z_radps"});

        m_gnss_csv.open(m_output_dir / "gnss.csv", {"time_s", "p_e_x_m", "p_e_y_m", "p_e_z_m"});

        m_nav_csv.open(m_output_dir / "nav.csv",
                       {"time_s",
                        "p_e_x_m",
                        "p_e_y_m",
                        "p_e_z_m",
                        "err_p_e_x_m",
                        "err_p_e_y_m",
                        "err_p_e_z_m",
                        "sigma_p_e_x_m",
                        "sigma_p_e_y_m",
                        "sigma_p_e_z_m"});

        m_gnss_pos_update_csv.open(m_output_dir / "gnss_pos_update.csv",
                                   gnss_pos_update_header());
    }

    void set_gnss_metadata(double sigma_h_m, double sigma_v_m, unsigned int seed)
    {
        m_gnss_noise = {{"sigma_h_m", sigma_h_m}, {"sigma_v_m", sigma_v_m}, {"seed", seed}};
    }

    void log_truth(const sim::TruthSample& sample)
    {
        m_truth_csv.write_row(sample.time,
                              sample.p_e.x(),
                              sample.p_e.y(),
                              sample.p_e.z(),
                              sample.v_e.x(),
                              sample.v_e.y(),
                              sample.v_e.z(),
                              sample.a_e.x(),
                              sample.a_e.y(),
                              sample.a_e.z(),
                              sample.q_eb.w(),
                              sample.q_eb.x(),
                              sample.q_eb.y(),
                              sample.q_eb.z(),
                              sample.w_ib_b.x(),
                              sample.w_ib_b.y(),
                              sample.w_ib_b.z());
    }

    template<typename Measurement>
    void log_gnss(const Measurement& meas)
    {
        m_gnss_csv.write_row(meas.time, meas.z.x(), meas.z.y(), meas.z.z());
    }

    template<typename StateDef, typename Filter>
    void log_nav(double time_s, const Filter& filter, const sim::TruthSample& truth)
    {
        const auto p_est = filter.state().template segment<3>(StateDef::Pos::i);
        const auto err = p_est - truth.p_e;
        const auto& P = filter.covariance();

        m_nav_csv.write_row(time_s,
                            p_est.x(),
                            p_est.y(),
                            p_est.z(),
                            err.x(),
                            err.y(),
                            err.z(),
                            std::sqrt(P(StateDef::Pos::i + 0, StateDef::Pos::i + 0)),
                            std::sqrt(P(StateDef::Pos::i + 1, StateDef::Pos::i + 1)),
                            std::sqrt(P(StateDef::Pos::i + 2, StateDef::Pos::i + 2)));
    }

    template<typename Statistics>
    void log_gnss_pos_statistics(const Statistics& stats)
    {
        static_assert(Statistics::O_t::RowsAtCompileTime == 3,
                      "GNSS position update statistics must have measurement dimension 3.");

        m_gnss_pos_update_csv.write_row(stats.time,
                                        stats.accepted ? 1 : 0,
                                        stats.nis,
                                        stats.innovation(0),
                                        stats.innovation(1),
                                        stats.innovation(2),
                                        std::sqrt(stats.innovation_covariance(0, 0)),
                                        std::sqrt(stats.innovation_covariance(1, 1)),
                                        std::sqrt(stats.innovation_covariance(2, 2)),
                                        stats.innovation_covariance(0, 0),
                                        stats.innovation_covariance(0, 1),
                                        stats.innovation_covariance(0, 2),
                                        stats.innovation_covariance(1, 0),
                                        stats.innovation_covariance(1, 1),
                                        stats.innovation_covariance(1, 2),
                                        stats.innovation_covariance(2, 0),
                                        stats.innovation_covariance(2, 1),
                                        stats.innovation_covariance(2, 2),
                                        stats.measurement_covariance(0, 0),
                                        stats.measurement_covariance(0, 1),
                                        stats.measurement_covariance(0, 2),
                                        stats.measurement_covariance(1, 0),
                                        stats.measurement_covariance(1, 1),
                                        stats.measurement_covariance(1, 2),
                                        stats.measurement_covariance(2, 0),
                                        stats.measurement_covariance(2, 1),
                                        stats.measurement_covariance(2, 2),
                                        stats.jacobian_h(0, 0),
                                        stats.jacobian_h(0, 1),
                                        stats.jacobian_h(0, 2),
                                        stats.jacobian_h(0, 3),
                                        stats.jacobian_h(0, 4),
                                        stats.jacobian_h(0, 5),
                                        stats.jacobian_h(0, 6),
                                        stats.jacobian_h(0, 7),
                                        stats.jacobian_h(0, 8),
                                        stats.jacobian_h(0, 9),
                                        stats.jacobian_h(0, 10),
                                        stats.jacobian_h(0, 11),
                                        stats.jacobian_h(0, 12),
                                        stats.jacobian_h(0, 13),
                                        stats.jacobian_h(0, 14),
                                        stats.jacobian_h(0, 15),
                                        stats.jacobian_h(0, 16),
                                        stats.jacobian_h(0, 17),
                                        stats.jacobian_h(0, 18),
                                        stats.jacobian_h(0, 19),
                                        stats.jacobian_h(0, 20),
                                        stats.jacobian_h(1, 0),
                                        stats.jacobian_h(1, 1),
                                        stats.jacobian_h(1, 2),
                                        stats.jacobian_h(1, 3),
                                        stats.jacobian_h(1, 4),
                                        stats.jacobian_h(1, 5),
                                        stats.jacobian_h(1, 6),
                                        stats.jacobian_h(1, 7),
                                        stats.jacobian_h(1, 8),
                                        stats.jacobian_h(1, 9),
                                        stats.jacobian_h(1, 10),
                                        stats.jacobian_h(1, 11),
                                        stats.jacobian_h(1, 12),
                                        stats.jacobian_h(1, 13),
                                        stats.jacobian_h(1, 14),
                                        stats.jacobian_h(1, 15),
                                        stats.jacobian_h(1, 16),
                                        stats.jacobian_h(1, 17),
                                        stats.jacobian_h(1, 18),
                                        stats.jacobian_h(1, 19),
                                        stats.jacobian_h(1, 20),
                                        stats.jacobian_h(2, 0),
                                        stats.jacobian_h(2, 1),
                                        stats.jacobian_h(2, 2),
                                        stats.jacobian_h(2, 3),
                                        stats.jacobian_h(2, 4),
                                        stats.jacobian_h(2, 5),
                                        stats.jacobian_h(2, 6),
                                        stats.jacobian_h(2, 7),
                                        stats.jacobian_h(2, 8),
                                        stats.jacobian_h(2, 9),
                                        stats.jacobian_h(2, 10),
                                        stats.jacobian_h(2, 11),
                                        stats.jacobian_h(2, 12),
                                        stats.jacobian_h(2, 13),
                                        stats.jacobian_h(2, 14),
                                        stats.jacobian_h(2, 15),
                                        stats.jacobian_h(2, 16),
                                        stats.jacobian_h(2, 17),
                                        stats.jacobian_h(2, 18),
                                        stats.jacobian_h(2, 19),
                                        stats.jacobian_h(2, 20),
                                        stats.kalman_gain(0, 0),
                                        stats.kalman_gain(0, 1),
                                        stats.kalman_gain(0, 2),
                                        stats.kalman_gain(1, 0),
                                        stats.kalman_gain(1, 1),
                                        stats.kalman_gain(1, 2),
                                        stats.kalman_gain(2, 0),
                                        stats.kalman_gain(2, 1),
                                        stats.kalman_gain(2, 2),
                                        stats.kalman_gain(3, 0),
                                        stats.kalman_gain(3, 1),
                                        stats.kalman_gain(3, 2),
                                        stats.kalman_gain(4, 0),
                                        stats.kalman_gain(4, 1),
                                        stats.kalman_gain(4, 2),
                                        stats.kalman_gain(5, 0),
                                        stats.kalman_gain(5, 1),
                                        stats.kalman_gain(5, 2),
                                        stats.kalman_gain(6, 0),
                                        stats.kalman_gain(6, 1),
                                        stats.kalman_gain(6, 2),
                                        stats.kalman_gain(7, 0),
                                        stats.kalman_gain(7, 1),
                                        stats.kalman_gain(7, 2),
                                        stats.kalman_gain(8, 0),
                                        stats.kalman_gain(8, 1),
                                        stats.kalman_gain(8, 2),
                                        stats.kalman_gain(9, 0),
                                        stats.kalman_gain(9, 1),
                                        stats.kalman_gain(9, 2),
                                        stats.kalman_gain(10, 0),
                                        stats.kalman_gain(10, 1),
                                        stats.kalman_gain(10, 2),
                                        stats.kalman_gain(11, 0),
                                        stats.kalman_gain(11, 1),
                                        stats.kalman_gain(11, 2),
                                        stats.kalman_gain(12, 0),
                                        stats.kalman_gain(12, 1),
                                        stats.kalman_gain(12, 2),
                                        stats.kalman_gain(13, 0),
                                        stats.kalman_gain(13, 1),
                                        stats.kalman_gain(13, 2),
                                        stats.kalman_gain(14, 0),
                                        stats.kalman_gain(14, 1),
                                        stats.kalman_gain(14, 2),
                                        stats.kalman_gain(15, 0),
                                        stats.kalman_gain(15, 1),
                                        stats.kalman_gain(15, 2),
                                        stats.kalman_gain(16, 0),
                                        stats.kalman_gain(16, 1),
                                        stats.kalman_gain(16, 2),
                                        stats.kalman_gain(17, 0),
                                        stats.kalman_gain(17, 1),
                                        stats.kalman_gain(17, 2),
                                        stats.kalman_gain(18, 0),
                                        stats.kalman_gain(18, 1),
                                        stats.kalman_gain(18, 2),
                                        stats.kalman_gain(19, 0),
                                        stats.kalman_gain(19, 1),
                                        stats.kalman_gain(19, 2),
                                        stats.kalman_gain(20, 0),
                                        stats.kalman_gain(20, 1),
                                        stats.kalman_gain(20, 2));
    }

    void close()
    {
        if (m_closed) {
            return;
        }

        m_truth_csv.flush();
        m_gnss_csv.flush();
        m_nav_csv.flush();
        m_gnss_pos_update_csv.flush();

        write_json_file(m_output_dir / "truth.meta.json", truth_metadata());
        write_json_file(m_output_dir / "gnss.meta.json", gnss_metadata());
        write_json_file(m_output_dir / "nav.meta.json", nav_metadata());
        write_json_file(m_output_dir / "gnss_pos_update.meta.json", gnss_pos_update_metadata());
        write_json_file(m_output_dir / "run_manifest.json", run_manifest());

        m_closed = true;
    }

    const std::filesystem::path& output_dir() const
    {
        return m_output_dir;
    }

private:
    static std::vector<std::string> gnss_pos_update_header()
    {
        std::vector<std::string> header = {"time_s",
                                           "accepted",
                                           "nis",
                                           "nu_p_e_x_m",
                                           "nu_p_e_y_m",
                                           "nu_p_e_z_m",
                                           "sigma_nu_p_e_x_m",
                                           "sigma_nu_p_e_y_m",
                                           "sigma_nu_p_e_z_m"};

        append_matrix_header(header, "S", 3, 3);
        append_matrix_header(header, "R", 3, 3);
        append_matrix_header(header, "H", 3, 21);
        append_matrix_header(header, "K", 21, 3);

        return header;
    }

    static void append_matrix_header(std::vector<std::string>& header,
                                     const std::string& name,
                                     int rows,
                                     int cols)
    {
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                header.push_back(name + "_" + std::to_string(r) + "_" + std::to_string(c));
            }
        }
    }

    static void write_json_file(const std::filesystem::path& path, const nlohmann::json& j)
    {
        std::ofstream f(path);
        if (!f) {
            throw std::runtime_error("Failed to open JSON file: " + path.string());
        }
        f << std::setw(2) << j << '\n';
    }

    static nlohmann::json truth_metadata()
    {
        return {{"schema", "truth_v1"},
                {"file", "truth.csv"},
                {"units",
                 {{"time", "s"},
                  {"p_e", "m"},
                  {"v_e", "m/s"},
                  {"a_e", "m/s^2"},
                  {"q_eb", "unit quaternion"},
                  {"w_ib_b", "rad/s"}}},
                {"frame_convention",
                 "Groves-style; q_eb transforms b-frame components to e-frame components"}};
    }

    nlohmann::json gnss_metadata() const
    {
        return {{"schema", "gnss_pos_v1"},
                {"file", "gnss.csv"},
                {"units", {{"time", "s"}, {"p_e", "m"}}},
                {"noise", m_gnss_noise}};
    }

    static nlohmann::json nav_metadata()
    {
        return {{"schema", "nav_estimate_v1"},
                {"file", "nav.csv"},
                {"units", {{"time", "s"}, {"p_e", "m"}, {"err_p_e", "m"}, {"sigma_p_e", "m"}}}};
    }

    static nlohmann::json gnss_pos_update_metadata()
    {
        return {
            {"schema", "gnss_pos_update_v1"},
            {"file", "gnss_pos_update.csv"},
            {"model", "gnss_pos"},
            {"units",
             {{"time", "s"},
              {"innovation", "m"},
              {"innovation_covariance", "m^2"},
              {"measurement_covariance", "m^2"},
              {"jacobian_h", "mixed"},
              {"kalman_gain", "mixed"},
              {"nis", "dimensionless"}}},
            {"description",
             "GNSS position measurement update statistics. The CSV includes innovation, S, R, H, K, "
             "NIS, timestamp, and accepted flag."}};
    }

    nlohmann::json run_manifest() const
    {
        return {{"run_name", m_run_name},
                {"config", m_config},
                {"logs",
                 {{"truth", {{"csv", "truth.csv"}, {"manifest", "truth.meta.json"}}},
                  {"gnss", {{"csv", "gnss.csv"}, {"manifest", "gnss.meta.json"}}},
                  {"nav", {{"csv", "nav.csv"}, {"manifest", "nav.meta.json"}}},
                  {"gnss_pos_update",
                   {{"csv", "gnss_pos_update.csv"},
                    {"manifest", "gnss_pos_update.meta.json"}}}}}};
    }

    std::filesystem::path m_output_dir;
    std::string m_run_name;
    nlohmann::json m_config;
    nlohmann::json m_gnss_noise = nlohmann::json::object();

    CsvWriter m_truth_csv;
    CsvWriter m_gnss_csv;
    CsvWriter m_nav_csv;
    CsvWriter m_gnss_pos_update_csv;

    bool m_closed = false;
};

} // namespace navkit::io
