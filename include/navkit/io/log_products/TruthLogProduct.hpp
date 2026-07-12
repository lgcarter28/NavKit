// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class TruthLogProduct
{
public:
    static constexpr std::string_view LogKey = "truth";

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
                    "q_eb_w",
                    "q_eb_x",
                    "q_eb_y",
                    "q_eb_z"});
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
                        sample.q_eb.w(),
                        sample.q_eb.x(),
                        sample.q_eb.y(),
                        sample.q_eb.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {
            {"schema", "truth_v1"},
            {"file", "truth.csv"},
            {"units", {{"time", "s"}, {"p_e", "m"}, {"v_e", "m/s"}, {"q_eb", "unit quaternion"}}},
            {"frame_convention",
             "Groves-style; q_eb transforms e-frame components to b-frame components"}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "truth.csv"}, {"manifest", "truth.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
