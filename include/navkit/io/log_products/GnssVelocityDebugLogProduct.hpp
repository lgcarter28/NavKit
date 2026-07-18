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

class GnssVelocityDebugLogProduct
{
public:
    static constexpr std::string_view LogKey = "gnss_velocity_debug_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss_velocity_debug_ecef.csv",
                   {"time_s",
                    "truth_v_e_x_mps",
                    "truth_v_e_y_mps",
                    "truth_v_e_z_mps",
                    "measured_v_e_x_mps",
                    "measured_v_e_y_mps",
                    "measured_v_e_z_mps",
                    "sigma_v_e_x_mps",
                    "sigma_v_e_y_mps",
                    "sigma_v_e_z_mps"});
    }

    void log(const GnssVelocityDebugLogPayload& payload)
    {
        m_csv.write_row(payload.time_s,
                        payload.truth_v_e_mps.x(),
                        payload.truth_v_e_mps.y(),
                        payload.truth_v_e_mps.z(),
                        payload.measured_v_e_mps.x(),
                        payload.measured_v_e_mps.y(),
                        payload.measured_v_e_mps.z(),
                        payload.sigma_v_e_mps.x(),
                        payload.sigma_v_e_mps.y(),
                        payload.sigma_v_e_mps.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "gnss_velocity_debug_ecef_v1"},
                {"file", "gnss_velocity_debug_ecef.csv"},
                {"description", "Truth and measured GNSS antenna velocity in ECEF coordinates."},
                {"units", {{"time", "s"}, {"v_e", "m/s"}, {"sigma_v_e", "m/s"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "gnss_velocity_debug_ecef.csv"},
                {"manifest", "gnss_velocity_debug_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
