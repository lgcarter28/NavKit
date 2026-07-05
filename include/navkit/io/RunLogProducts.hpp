// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvSchemaUtils.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <cmath>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <vector>

namespace navkit::io
{

template<typename StateDef, typename Filter>
struct NavEstimateLogPayload
{
    double time_s{};
    const Filter& filter;
    const sim::TruthSample& truth;
};

template<typename Statistics>
struct MeasurementStatisticsLogPayload
{
    const Statistics& statistics;
};

class TruthLogProduct
{
public:
    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "truth.csv",
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
    }

    void log(const sim::TruthSample& sample)
    {
        m_csv.write_row(sample.time,
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

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
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

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "truth.csv"}, {"manifest", "truth.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

class GnssPositionLogProduct
{
public:
    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss.csv", {"time_s", "p_e_x_m", "p_e_y_m", "p_e_z_m"});
    }

    void set_metadata(double sigma_h_m, double sigma_v_m, unsigned int seed)
    {
        m_noise = {{"sigma_h_m", sigma_h_m}, {"sigma_v_m", sigma_v_m}, {"seed", seed}};
    }

    template<typename Measurement>
    void log(const Measurement& meas)
    {
        m_csv.write_row(meas.time, meas.z.x(), meas.z.y(), meas.z.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    nlohmann::json metadata() const
    {
        return {{"schema", "gnss_pos_v1"},
                {"file", "gnss.csv"},
                {"units", {{"time", "s"}, {"p_e", "m"}}},
                {"noise", m_noise}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "gnss.csv"}, {"manifest", "gnss.meta.json"}};
    }

private:
    CsvWriter m_csv;
    nlohmann::json m_noise = nlohmann::json::object();
};

class NavEstimateLogProduct
{
public:
    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "nav.csv",
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

    template<typename StateDef, typename Filter>
    void log(const NavEstimateLogPayload<StateDef, Filter>& payload)
    {
        const auto p_est = payload.filter.state().template segment<3>(StateDef::Pos::i);
        const auto err = p_est - payload.truth.p_e;
        const auto& P = payload.filter.covariance();

        m_csv.write_row(payload.time_s,
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

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "nav_estimate_v1"},
                {"file", "nav.csv"},
                {"units", {{"time", "s"}, {"p_e", "m"}, {"err_p_e", "m"}, {"sigma_p_e", "m"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "nav.csv"}, {"manifest", "nav.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

class GnssPositionUpdateLogProduct
{
public:
    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss_pos_update.csv", header());
    }

    template<typename Statistics>
    void log(const MeasurementStatisticsLogPayload<Statistics>& payload)
    {
        const auto& stats = payload.statistics;

        static_assert(Statistics::O_t::RowsAtCompileTime == MeasurementDimension,
                      "GNSS position update statistics must have measurement dimension 3.");
        static_assert(
            Statistics::H_t::ColsAtCompileTime == StateDimension,
            "GNSS position update statistics schema currently expects state dimension 21.");
        static_assert(
            Statistics::K_t::RowsAtCompileTime == StateDimension,
            "GNSS position update statistics schema currently expects state dimension 21.");

        std::vector<double> row = {stats.time,
                                   stats.accepted ? 1.0 : 0.0,
                                   stats.nis,
                                   stats.innovation(0),
                                   stats.innovation(1),
                                   stats.innovation(2),
                                   std::sqrt(stats.innovation_covariance(0, 0)),
                                   std::sqrt(stats.innovation_covariance(1, 1)),
                                   std::sqrt(stats.innovation_covariance(2, 2))};

        row.reserve(header().size());
        detail::append_matrix_values(row, stats.innovation_covariance);
        detail::append_matrix_values(row, stats.measurement_covariance);
        detail::append_matrix_values(row, stats.jacobian_h);
        detail::append_matrix_values(row, stats.kalman_gain);

        m_csv.write_row_values(row);
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "gnss_pos_update_v1"},
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
                 "GNSS position measurement update statistics. The CSV includes innovation, S, R, "
                 "H, K, "
                 "NIS, timestamp, and accepted flag."}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "gnss_pos_update.csv"}, {"manifest", "gnss_pos_update.meta.json"}};
    }

private:
    static constexpr int MeasurementDimension = 3;
    static constexpr int StateDimension = 21;

    static std::vector<std::string> header()
    {
        std::vector<std::string> result = {"time_s",
                                           "accepted",
                                           "nis",
                                           "nu_p_e_x_m",
                                           "nu_p_e_y_m",
                                           "nu_p_e_z_m",
                                           "sigma_nu_p_e_x_m",
                                           "sigma_nu_p_e_y_m",
                                           "sigma_nu_p_e_z_m"};

        detail::append_matrix_header(result, "S", MeasurementDimension, MeasurementDimension);
        detail::append_matrix_header(result, "R", MeasurementDimension, MeasurementDimension);
        detail::append_matrix_header(result, "H", MeasurementDimension, StateDimension);
        detail::append_matrix_header(result, "K", StateDimension, MeasurementDimension);

        return result;
    }

    CsvWriter m_csv;
};

} // namespace navkit::io
