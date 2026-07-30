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

/** Log Autopilot commands and final Vehicle responses at one trajectory timestamp. */
class TrajectoryAutopilotVehicleLogProduct
{
public:
    static constexpr std::string_view LogKey = "trajectory_autopilot_vehicle";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "trajectory_autopilot_vehicle.csv",
                   {"time_s",
                    "autopilot_q_command_b2i_w",
                    "autopilot_q_command_b2i_x",
                    "autopilot_q_command_b2i_y",
                    "autopilot_q_command_b2i_z",
                    "autopilot_q_response_b2i_w",
                    "autopilot_q_response_b2i_x",
                    "autopilot_q_response_b2i_y",
                    "autopilot_q_response_b2i_z",
                    "autopilot_q_command_b2n_w",
                    "autopilot_q_command_b2n_x",
                    "autopilot_q_command_b2n_y",
                    "autopilot_q_command_b2n_z",
                    "autopilot_q_response_b2n_w",
                    "autopilot_q_response_b2n_x",
                    "autopilot_q_response_b2n_y",
                    "autopilot_q_response_b2n_z",
                    "autopilot_angular_rate_command_b_x_radps",
                    "autopilot_angular_rate_command_b_y_radps",
                    "autopilot_angular_rate_command_b_z_radps",
                    "autopilot_angular_rate_feedforward_b_x_radps",
                    "autopilot_angular_rate_feedforward_b_y_radps",
                    "autopilot_angular_rate_feedforward_b_z_radps",
                    "autopilot_angular_rate_response_b_x_radps",
                    "autopilot_angular_rate_response_b_y_radps",
                    "autopilot_angular_rate_response_b_z_radps",
                    "autopilot_gyro_observation_b_x_radps",
                    "autopilot_gyro_observation_b_y_radps",
                    "autopilot_gyro_observation_b_z_radps",
                    "autopilot_active",
                    "vehicle_velocity_ib_b_x_mps",
                    "vehicle_velocity_ib_b_y_mps",
                    "vehicle_velocity_ib_b_z_mps",
                    "vehicle_acceleration_ib_b_x_mps2",
                    "vehicle_acceleration_ib_b_y_mps2",
                    "vehicle_acceleration_ib_b_z_mps2",
                    "vehicle_specific_force_command_b_x_mps2",
                    "vehicle_specific_force_command_b_y_mps2",
                    "vehicle_specific_force_command_b_z_mps2",
                    "vehicle_specific_force_command_response_b_x_mps2",
                    "vehicle_specific_force_command_response_b_y_mps2",
                    "vehicle_specific_force_command_response_b_z_mps2",
                    "vehicle_specific_force_response_b_x_mps2",
                    "vehicle_specific_force_response_b_y_mps2",
                    "vehicle_specific_force_response_b_z_mps2",
                    "vehicle_angular_rate_command_b_x_radps",
                    "vehicle_angular_rate_command_b_y_radps",
                    "vehicle_angular_rate_command_b_z_radps",
                    "vehicle_angular_rate_response_b_x_radps",
                    "vehicle_angular_rate_response_b_y_radps",
                    "vehicle_angular_rate_response_b_z_radps",
                    "velocity_tracking_error_b_x_mps",
                    "velocity_tracking_error_b_y_mps",
                    "velocity_tracking_error_b_z_mps",
                    "acceleration_tracking_error_b_x_mps2",
                    "acceleration_tracking_error_b_y_mps2",
                    "acceleration_tracking_error_b_z_mps2",
                    "attitude_tracking_error_b_x_rad",
                    "attitude_tracking_error_b_y_rad",
                    "attitude_tracking_error_b_z_rad",
                    "angular_rate_tracking_error_b_x_radps",
                    "angular_rate_tracking_error_b_y_radps",
                    "angular_rate_tracking_error_b_z_radps",
                    "specific_force_tracking_error_b_x_mps2",
                    "specific_force_tracking_error_b_y_mps2",
                    "specific_force_tracking_error_b_z_mps2",
                    "angular_rate_limited_x",
                    "angular_rate_limited_y",
                    "angular_rate_limited_z",
                    "specific_force_limited_x",
                    "specific_force_limited_y",
                    "specific_force_limited_z"});
    }

    void log(const TrajectoryAutopilotVehicleLogPayload& payload)
    {
        const TrajectoryLogData& data = payload.data;
        m_csv.write_row(core::timestamp_seconds(data.t),
                        data.autopilot_q_command_b2i.w(),
                        data.autopilot_q_command_b2i.x(),
                        data.autopilot_q_command_b2i.y(),
                        data.autopilot_q_command_b2i.z(),
                        data.autopilot_q_response_b2i.w(),
                        data.autopilot_q_response_b2i.x(),
                        data.autopilot_q_response_b2i.y(),
                        data.autopilot_q_response_b2i.z(),
                        data.autopilot_q_command_b2n.w(),
                        data.autopilot_q_command_b2n.x(),
                        data.autopilot_q_command_b2n.y(),
                        data.autopilot_q_command_b2n.z(),
                        data.autopilot_q_response_b2n.w(),
                        data.autopilot_q_response_b2n.x(),
                        data.autopilot_q_response_b2n.y(),
                        data.autopilot_q_response_b2n.z(),
                        data.autopilot_angular_rate_command_b_radps.x(),
                        data.autopilot_angular_rate_command_b_radps.y(),
                        data.autopilot_angular_rate_command_b_radps.z(),
                        data.autopilot_angular_rate_feedforward_b_radps.x(),
                        data.autopilot_angular_rate_feedforward_b_radps.y(),
                        data.autopilot_angular_rate_feedforward_b_radps.z(),
                        data.autopilot_angular_rate_response_b_radps.x(),
                        data.autopilot_angular_rate_response_b_radps.y(),
                        data.autopilot_angular_rate_response_b_radps.z(),
                        data.autopilot_gyro_observation_b_radps.x(),
                        data.autopilot_gyro_observation_b_radps.y(),
                        data.autopilot_gyro_observation_b_radps.z(),
                        data.autopilot_active,
                        data.vehicle_velocity_ib_b_mps.x(),
                        data.vehicle_velocity_ib_b_mps.y(),
                        data.vehicle_velocity_ib_b_mps.z(),
                        data.vehicle_acceleration_ib_b_mps2.x(),
                        data.vehicle_acceleration_ib_b_mps2.y(),
                        data.vehicle_acceleration_ib_b_mps2.z(),
                        data.vehicle_specific_force_command_b_mps2.x(),
                        data.vehicle_specific_force_command_b_mps2.y(),
                        data.vehicle_specific_force_command_b_mps2.z(),
                        data.vehicle_specific_force_command_response_b_mps2.x(),
                        data.vehicle_specific_force_command_response_b_mps2.y(),
                        data.vehicle_specific_force_command_response_b_mps2.z(),
                        data.vehicle_specific_force_response_b_mps2.x(),
                        data.vehicle_specific_force_response_b_mps2.y(),
                        data.vehicle_specific_force_response_b_mps2.z(),
                        data.vehicle_angular_rate_command_b_radps.x(),
                        data.vehicle_angular_rate_command_b_radps.y(),
                        data.vehicle_angular_rate_command_b_radps.z(),
                        data.vehicle_angular_rate_response_b_radps.x(),
                        data.vehicle_angular_rate_response_b_radps.y(),
                        data.vehicle_angular_rate_response_b_radps.z(),
                        data.velocity_tracking_error_b_mps.x(),
                        data.velocity_tracking_error_b_mps.y(),
                        data.velocity_tracking_error_b_mps.z(),
                        data.acceleration_tracking_error_b_mps2.x(),
                        data.acceleration_tracking_error_b_mps2.y(),
                        data.acceleration_tracking_error_b_mps2.z(),
                        data.attitude_tracking_error_b_rad.x(),
                        data.attitude_tracking_error_b_rad.y(),
                        data.attitude_tracking_error_b_rad.z(),
                        data.angular_rate_tracking_error_b_radps.x(),
                        data.angular_rate_tracking_error_b_radps.y(),
                        data.angular_rate_tracking_error_b_radps.z(),
                        data.specific_force_tracking_error_b_mps2.x(),
                        data.specific_force_tracking_error_b_mps2.y(),
                        data.specific_force_tracking_error_b_mps2.z(),
                        data.angular_rate_limited.x(),
                        data.angular_rate_limited.y(),
                        data.angular_rate_limited.z(),
                        data.specific_force_limited.x(),
                        data.specific_force_limited.y(),
                        data.specific_force_limited.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    [[nodiscard]] static nlohmann::json metadata()
    {
        return {{"schema", "navkit.trajectory_autopilot_vehicle.v2"},
                {"file", "trajectory_autopilot_vehicle.csv"},
                {"signal_flow",
                 "Autopilot attitude/rate command -> Vehicle angular-rate and specific-force "
                 "response -> integrated inertial truth"},
                {"frame_convention",
                 "Quaternion data are preserved in ECI and local NED forms. Rates, specific force, "
                 "vehicle kinematics, and tracking errors are body-resolved."},
                {"tracking_error_definition",
                 "Every tracking error is command minus final post-response, post-limit truth "
                 "resolved in body."}};
    }

    [[nodiscard]] static nlohmann::json manifest_entry()
    {
        return {{"csv", "trajectory_autopilot_vehicle.csv"},
                {"manifest", "trajectory_autopilot_vehicle.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
