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

class TrajectoryBodyLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_kinematics_body";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_kinematics_body.csv",
                   {"time_s",
                    "v_ib_b_x_mps",
                    "v_ib_b_y_mps",
                    "v_ib_b_z_mps",
                    "a_ib_b_x_mps2",
                    "a_ib_b_y_mps2",
                    "a_ib_b_z_mps2",
                    "v_eb_b_x_mps",
                    "v_eb_b_y_mps",
                    "v_eb_b_z_mps",
                    "a_eb_b_x_mps2",
                    "a_eb_b_y_mps2",
                    "a_eb_b_z_mps2",
                    "w_ib_b_x_radps",
                    "w_ib_b_y_radps",
                    "w_ib_b_z_radps",
                    "w_eb_b_x_radps",
                    "w_eb_b_y_radps",
                    "w_eb_b_z_radps",
                    "specific_force_ib_b_x_mps2",
                    "specific_force_ib_b_y_mps2",
                    "specific_force_ib_b_z_mps2"});
    }

    void log(const TrajectoryBodyLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.v_ib_b_mps.x(),
                        data.v_ib_b_mps.y(),
                        data.v_ib_b_mps.z(),
                        data.a_ib_b_mps2.x(),
                        data.a_ib_b_mps2.y(),
                        data.a_ib_b_mps2.z(),
                        data.v_eb_b_mps.x(),
                        data.v_eb_b_mps.y(),
                        data.v_eb_b_mps.z(),
                        data.a_eb_b_mps2.x(),
                        data.a_eb_b_mps2.y(),
                        data.a_eb_b_mps2.z(),
                        data.w_ib_b_radps.x(),
                        data.w_ib_b_radps.y(),
                        data.w_ib_b_radps.z(),
                        data.w_eb_b_radps.x(),
                        data.w_eb_b_radps.y(),
                        data.w_eb_b_radps.z(),
                        data.specific_force_ib_b_mps2.x(),
                        data.specific_force_ib_b_mps2.y(),
                        data.specific_force_ib_b_mps2.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_kinematics_body.v3"},
                {"file", "trajectory_kinematics_body.csv"},
                {"units",
                 {{"time", "s"},
                  {"v_ib_b", "m/s"},
                  {"a_ib_b", "m/s^2"},
                  {"v_eb_b", "m/s"},
                  {"a_eb_b", "m/s^2"},
                  {"w_ib_b", "rad/s"},
                  {"w_eb_b", "rad/s"},
                  {"specific_force_ib_b", "m/s^2"}}},
                {"frame_convention",
                 "ECI-relative and ECEF-relative body kinematics are logged explicitly; "
                 "specific force is body with respect to ECI resolved in body"},
                {"acceleration_definition",
                 "a_ib_b = C_i2b a_i resolves ECI coordinate acceleration in body; because body "
                 "rotates, this is not (d/dt)_b v_ib^b"}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_kinematics_body.csv"},
                {"manifest", "trajectory_kinematics_body.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
