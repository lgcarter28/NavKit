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

/** Log source-agnostic Guidance acceleration, bank, mode, and activity state. */
class TrajectoryGuidanceLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_guidance";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_guidance.csv",
                   {"time_s",
                    "guidance_velocity_reference_i_x_mps",
                    "guidance_velocity_reference_i_y_mps",
                    "guidance_velocity_reference_i_z_mps",
                    "guidance_acceleration_command_i_x_mps2",
                    "guidance_acceleration_command_i_y_mps2",
                    "guidance_acceleration_command_i_z_mps2",
                    "guidance_acceleration_command_n_n_mps2",
                    "guidance_acceleration_command_n_e_mps2",
                    "guidance_acceleration_command_n_d_mps2",
                    "guidance_acceleration_command_b_x_mps2",
                    "guidance_acceleration_command_b_y_mps2",
                    "guidance_acceleration_command_b_z_mps2",
                    "guidance_acceleration_response_i_x_mps2",
                    "guidance_acceleration_response_i_y_mps2",
                    "guidance_acceleration_response_i_z_mps2",
                    "guidance_acceleration_response_n_n_mps2",
                    "guidance_acceleration_response_n_e_mps2",
                    "guidance_acceleration_response_n_d_mps2",
                    "guidance_acceleration_response_b_x_mps2",
                    "guidance_acceleration_response_b_y_mps2",
                    "guidance_acceleration_response_b_z_mps2",
                    "guidance_bank_command_n_rad",
                    "guidance_bank_response_n_rad",
                    "guidance_reference_position_e_x_m",
                    "guidance_reference_position_e_y_m",
                    "guidance_reference_position_e_z_m",
                    "guidance_reference_index",
                    "guidance_reference_position_valid",
                    "guidance_mode",
                    "guidance_active",
                    "pad_constraint_active"});
    }

    void log(const TrajectoryGuidanceLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.guidance_velocity_reference_i_mps.x(),
                        data.guidance_velocity_reference_i_mps.y(),
                        data.guidance_velocity_reference_i_mps.z(),
                        data.guidance_acceleration_command_i_mps2.x(),
                        data.guidance_acceleration_command_i_mps2.y(),
                        data.guidance_acceleration_command_i_mps2.z(),
                        data.guidance_acceleration_command_n_mps2.x(),
                        data.guidance_acceleration_command_n_mps2.y(),
                        data.guidance_acceleration_command_n_mps2.z(),
                        data.guidance_acceleration_command_b_mps2.x(),
                        data.guidance_acceleration_command_b_mps2.y(),
                        data.guidance_acceleration_command_b_mps2.z(),
                        data.guidance_acceleration_response_i_mps2.x(),
                        data.guidance_acceleration_response_i_mps2.y(),
                        data.guidance_acceleration_response_i_mps2.z(),
                        data.guidance_acceleration_response_n_mps2.x(),
                        data.guidance_acceleration_response_n_mps2.y(),
                        data.guidance_acceleration_response_n_mps2.z(),
                        data.guidance_acceleration_response_b_mps2.x(),
                        data.guidance_acceleration_response_b_mps2.y(),
                        data.guidance_acceleration_response_b_mps2.z(),
                        data.guidance_bank_command_n_rad,
                        data.guidance_bank_response_n_rad,
                        data.guidance_reference_position_e_m.x(),
                        data.guidance_reference_position_e_m.y(),
                        data.guidance_reference_position_e_m.z(),
                        data.guidance_reference_index,
                        data.guidance_reference_position_valid,
                        static_cast<unsigned int>(data.guidance_mode),
                        data.guidance_active,
                        data.pad_constraint_active);
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_guidance.v1"},
                {"file", "trajectory_guidance.csv"},
                {"signal_flow",
                 "Guidance velocity reference -> inertial acceleration and NED bank command -> "
                 "vehicle translational response"},
                {"frame_convention",
                 "Acceleration command and response are logged in ECI, local NED, and body. "
                 "Bank is roll with respect to local NED. A valid Guidance reference position is "
                 "an ECEF waypoint or equivalent profile reference."}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_guidance.csv"}, {"manifest", "trajectory_guidance.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
