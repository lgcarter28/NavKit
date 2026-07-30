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

class TrajectoryNedLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_kinematics_ned";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_kinematics_ned.csv",
                   {"time_s",
                    "p_lla_lat_deg",
                    "p_lla_lon_deg",
                    "p_lla_h_m",
                    "v_n_n_mps",
                    "v_n_e_mps",
                    "v_n_d_mps",
                    "a_n_n_mps2",
                    "a_n_e_mps2",
                    "a_n_d_mps2",
                    "q_b2n_w",
                    "q_b2n_x",
                    "q_b2n_y",
                    "q_b2n_z",
                    "w_nb_b_x_radps",
                    "w_nb_b_y_radps",
                    "w_nb_b_z_radps"});
    }

    void log(const TrajectoryNedLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.p_lla_deg_m.x(),
                        data.p_lla_deg_m.y(),
                        data.p_lla_deg_m.z(),
                        data.v_n_mps.x(),
                        data.v_n_mps.y(),
                        data.v_n_mps.z(),
                        data.a_n_mps2.x(),
                        data.a_n_mps2.y(),
                        data.a_n_mps2.z(),
                        data.q_b2n.w(),
                        data.q_b2n.x(),
                        data.q_b2n.y(),
                        data.q_b2n.z(),
                        data.w_nb_b_radps.x(),
                        data.w_nb_b_radps.y(),
                        data.w_nb_b_radps.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_kinematics_ned.v1"},
                {"file", "trajectory_kinematics_ned.csv"},
                {"units",
                 {{"time", "s"},
                  {"p_lla", "deg, deg, m"},
                  {"v_n", "m/s"},
                  {"a_n", "m/s^2"},
                  {"q_b2n", "unit quaternion"},
                  {"w_nb_b", "rad/s"}}},
                {"frame_convention",
                 "Local geodetic north-east-down; q_b2n maps body components to NED components "
                 "and w_nb_b is body with respect to NED resolved in body"},
                {"acceleration_definition",
                 "a_n = C_e2n a_e resolves ECEF coordinate acceleration in instantaneous local "
                 "NED; because the local frame varies, this is not (d/dt)_n v_eb^n"}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_kinematics_ned.csv"},
                {"manifest", "trajectory_kinematics_ned.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
