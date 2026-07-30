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

class TrajectoryEciLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_kinematics_eci";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_kinematics_eci.csv",
                   {"time_s",
                    "p_i_x_m",
                    "p_i_y_m",
                    "p_i_z_m",
                    "v_i_x_mps",
                    "v_i_y_mps",
                    "v_i_z_mps",
                    "a_i_x_mps2",
                    "a_i_y_mps2",
                    "a_i_z_mps2",
                    "q_b2i_w",
                    "q_b2i_x",
                    "q_b2i_y",
                    "q_b2i_z",
                    "w_ib_b_x_radps",
                    "w_ib_b_y_radps",
                    "w_ib_b_z_radps"});
    }

    void log(const TrajectoryEciLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.p_i_m.x(),
                        data.p_i_m.y(),
                        data.p_i_m.z(),
                        data.v_i_mps.x(),
                        data.v_i_mps.y(),
                        data.v_i_mps.z(),
                        data.a_i_mps2.x(),
                        data.a_i_mps2.y(),
                        data.a_i_mps2.z(),
                        data.q_b2i.w(),
                        data.q_b2i.x(),
                        data.q_b2i.y(),
                        data.q_b2i.z(),
                        data.w_ib_b_radps.x(),
                        data.w_ib_b_radps.y(),
                        data.w_ib_b_radps.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_kinematics_eci.v1"},
                {"file", "trajectory_kinematics_eci.csv"},
                {"units",
                 {{"time", "s"},
                  {"p_i", "m"},
                  {"v_i", "m/s"},
                  {"a_i", "m/s^2"},
                  {"q_b2i", "unit quaternion"},
                  {"w_ib_b", "rad/s"}}},
                {"frame_convention",
                 "Groves-style; q_b2i maps body components to ECI components and w_ib_b is "
                 "body with respect to ECI resolved in body"},
                {"acceleration_definition", "Inertial translational acceleration resolved in ECI"},
                {"earth_orientation",
                 "Selected constant-rate rotating planet; ECI and ECEF are coincident at the "
                 "trajectory source epoch"}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_kinematics_eci.csv"},
                {"manifest", "trajectory_kinematics_eci.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
