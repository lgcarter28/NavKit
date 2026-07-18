// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/GnssMeasurementLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::io
{

class GnssVelocityLogProduct
{
public:
    static constexpr std::string_view LogKey = "gnss_velocity_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss_velocity_ecef.csv",
                   {"time_s", "v_e_x_mps", "v_e_y_mps", "v_e_z_mps"});
    }

    void set_metadata(const std::string& covariance_frame,
                      const nlohmann::json& covariance,
                      unsigned int seed)
    {
        m_noise = {
            {"covariance_frame", covariance_frame}, {"covariance", covariance}, {"seed", seed}};
    }

    void log(const GnssVelocityLogPayload& payload)
    {
        const core::estimation::Measurement<3>& meas = payload.measurement;
        m_csv.write_row(meas.time, meas.z.x(), meas.z.y(), meas.z.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    nlohmann::json metadata() const
    {
        return {{"schema", "gnss_velocity_v1"},
                {"file", "gnss_velocity_ecef.csv"},
                {"units", {{"time", "s"}, {"v_e", "m/s"}}},
                {"noise", m_noise}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "gnss_velocity_ecef.csv"}, {"manifest", "gnss_velocity_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
    nlohmann::json m_noise = nlohmann::json::object();
};

} // namespace navkit::io
