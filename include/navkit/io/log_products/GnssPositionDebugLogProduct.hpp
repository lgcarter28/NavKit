// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/GnssMeasurementLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class GnssPositionDebugLogProduct
{
public:
    static constexpr std::string_view LogKey = "gnss_position_debug_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss_position_debug_ecef.csv",
                   {"time_s",
                    "truth_p_e_x_m",
                    "truth_p_e_y_m",
                    "truth_p_e_z_m",
                    "measured_p_e_x_m",
                    "measured_p_e_y_m",
                    "measured_p_e_z_m",
                    "sigma_p_e_x_m",
                    "sigma_p_e_y_m",
                    "sigma_p_e_z_m"});
    }

    void log(const GnssPositionDebugLogPayload& payload)
    {
        m_csv.write_row(payload.time_s,
                        payload.truth_p_e_m.x(),
                        payload.truth_p_e_m.y(),
                        payload.truth_p_e_m.z(),
                        payload.measured_p_e_m.x(),
                        payload.measured_p_e_m.y(),
                        payload.measured_p_e_m.z(),
                        payload.sigma_p_e_m.x(),
                        payload.sigma_p_e_m.y(),
                        payload.sigma_p_e_m.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "gnss_position_debug_ecef_v1"},
                {"file", "gnss_position_debug_ecef.csv"},
                {"description", "Truth and measured GNSS antenna position in ECEF coordinates."},
                {"units", {{"time", "s"}, {"p_e", "m"}, {"sigma_p_e", "m"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "gnss_position_debug_ecef.csv"},
                {"manifest", "gnss_position_debug_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
