// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class TruthLogProduct
{
public:
    static constexpr std::string_view LogKey = "truth_trajectory_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "truth_trajectory_ecef.csv",
                   {"time_s",
                    "p_e_x_m",
                    "p_e_y_m",
                    "p_e_z_m",
                    "v_e_x_mps",
                    "v_e_y_mps",
                    "v_e_z_mps",
                    "q_b2e_w",
                    "q_b2e_x",
                    "q_b2e_y",
                    "q_b2e_z"});
    }

    void log(const sim::TruthSample& sample)
    {
        m_csv.write_row(core::timestamp_seconds(sample.t),
                        sample.p_e.x(),
                        sample.p_e.y(),
                        sample.p_e.z(),
                        sample.v_e.x(),
                        sample.v_e.y(),
                        sample.v_e.z(),
                        sample.q_b2e.w(),
                        sample.q_b2e.x(),
                        sample.q_b2e.y(),
                        sample.q_b2e.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {
            {"schema", "truth_trajectory_ecef_v1"},
            {"file", "truth_trajectory_ecef.csv"},
            {"units", {{"time", "s"}, {"p_e", "m"}, {"v_e", "m/s"}, {"q_b2e", "unit quaternion"}}},
            {"frame_convention",
             "Groves-style; q_b2e transforms body-frame components to ECEF-frame components"}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "truth_trajectory_ecef.csv"},
                {"manifest", "truth_trajectory_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
