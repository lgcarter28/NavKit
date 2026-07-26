// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/GnssMeasurementLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::io
{

class GnssPositionLogProduct
{
public:
    static constexpr std::string_view LogKey = "gnss";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "gnss.csv", {"time_s", "p_e_x_m", "p_e_y_m", "p_e_z_m"});
    }

    void set_metadata(const std::string& covariance_frame,
                      const nlohmann::json& covariance,
                      unsigned int seed)
    {
        m_noise = {
            {"covariance_frame", covariance_frame}, {"covariance", covariance}, {"seed", seed}};
    }

    void log(const GnssPositionLogPayload& payload)
    {
        const core::estimation::Measurement<3>& meas = payload.measurement;
        m_csv.write_row(core::timestamp_seconds(meas.t), meas.z.x(), meas.z.y(), meas.z.z());
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

} // namespace navkit::io
