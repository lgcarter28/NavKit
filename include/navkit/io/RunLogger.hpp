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

    void close()
    {
        if (m_closed) {
            return;
        }

        m_truth_csv.flush();
        m_gnss_csv.flush();
        m_nav_csv.flush();

        write_json_file(m_output_dir / "truth.meta.json", truth_metadata());
        write_json_file(m_output_dir / "gnss.meta.json", gnss_metadata());
        write_json_file(m_output_dir / "nav.meta.json", nav_metadata());
        write_json_file(m_output_dir / "run_manifest.json", run_manifest());

        m_closed = true;
    }

    const std::filesystem::path& output_dir() const
    {
        return m_output_dir;
    }

private:
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

    nlohmann::json run_manifest() const
    {
        return {{"run_name", m_run_name},
                {"config", m_config},
                {"logs",
                 {{"truth", {{"csv", "truth.csv"}, {"manifest", "truth.meta.json"}}},
                  {"gnss", {{"csv", "gnss.csv"}, {"manifest", "gnss.meta.json"}}},
                  {"nav", {{"csv", "nav.csv"}, {"manifest", "nav.meta.json"}}}}}};
    }

    std::filesystem::path m_output_dir;
    std::string m_run_name;
    nlohmann::json m_config;
    nlohmann::json m_gnss_noise = nlohmann::json::object();

    CsvWriter m_truth_csv;
    CsvWriter m_gnss_csv;
    CsvWriter m_nav_csv;

    bool m_closed = false;
};

} // namespace navkit::io
