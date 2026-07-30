// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/TrajectoryLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class TrajectoryEcefLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_kinematics_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_kinematics_ecef.csv",
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
                    "q_b2e_w",
                    "q_b2e_x",
                    "q_b2e_y",
                    "q_b2e_z",
                    "w_eb_b_x_radps",
                    "w_eb_b_y_radps",
                    "w_eb_b_z_radps"});
    }

    void log(const TrajectoryEcefLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.p_e_m.x(),
                        data.p_e_m.y(),
                        data.p_e_m.z(),
                        data.v_e_mps.x(),
                        data.v_e_mps.y(),
                        data.v_e_mps.z(),
                        data.a_e_mps2.x(),
                        data.a_e_mps2.y(),
                        data.a_e_mps2.z(),
                        data.q_b2e.w(),
                        data.q_b2e.x(),
                        data.q_b2e.y(),
                        data.q_b2e.z(),
                        data.w_eb_b_radps.x(),
                        data.w_eb_b_radps.y(),
                        data.w_eb_b_radps.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_kinematics_ecef.v1"},
                {"file", "trajectory_kinematics_ecef.csv"},
                {"units",
                 {{"time", "s"},
                  {"p_e", "m"},
                  {"v_e", "m/s"},
                  {"a_e", "m/s^2"},
                  {"q_b2e", "unit quaternion"},
                  {"w_eb_b", "rad/s"}}},
                {"frame_convention",
                 "Groves-style; q_b2e maps body components to ECEF components and w_eb_b is "
                 "body with respect to ECEF resolved in body"},
                {"acceleration_definition",
                 "a_e is the rotating-frame derivative of ECEF-relative velocity: "
                 "(d/dt)_e v_eb^e, equivalently the second derivative of ECEF position "
                 "coordinates"},
                {"earth_orientation",
                 "Selected constant-rate rotating planet; ECI and ECEF are coincident at the "
                 "trajectory source epoch"}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_kinematics_ecef.csv"},
                {"manifest", "trajectory_kinematics_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
