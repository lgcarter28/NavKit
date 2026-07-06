// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvSchemaUtils.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/MeasurementStatisticsLogPayload.hpp"

#include <cmath>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace navkit::io
{

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
