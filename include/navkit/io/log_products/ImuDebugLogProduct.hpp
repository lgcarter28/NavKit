// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/ImuDebugLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class ImuDebugLogProduct
{
public:
    static constexpr std::string_view LogKey = "imu_debug_ecef";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "imu_debug_ecef.csv",
                   {"time_s",
                    "dt_s",
                    "p_bar_e_x_m",
                    "p_bar_e_y_m",
                    "p_bar_e_z_m",
                    "v_bar_e_x_mps",
                    "v_bar_e_y_mps",
                    "v_bar_e_z_mps",
                    "a_bar_e_x_mps2",
                    "a_bar_e_y_mps2",
                    "a_bar_e_z_mps2",
                    "gravity_e_x_mps2",
                    "gravity_e_y_mps2",
                    "gravity_e_z_mps2",
                    "specific_force_e_x_mps2",
                    "specific_force_e_y_mps2",
                    "specific_force_e_z_mps2",
                    "specific_force_ib_b_x_mps2",
                    "specific_force_ib_b_y_mps2",
                    "specific_force_ib_b_z_mps2",
                    "delta_theta_eb_b_x_rad",
                    "delta_theta_eb_b_y_rad",
                    "delta_theta_eb_b_z_rad",
                    "delta_theta_ib_b_x_rad",
                    "delta_theta_ib_b_y_rad",
                    "delta_theta_ib_b_z_rad",
                    "delta_v_ib_b_x_mps",
                    "delta_v_ib_b_y_mps",
                    "delta_v_ib_b_z_mps",
                    "meas_delta_theta_ib_b_x_rad",
                    "meas_delta_theta_ib_b_y_rad",
                    "meas_delta_theta_ib_b_z_rad",
                    "meas_delta_v_ib_b_x_mps",
                    "meas_delta_v_ib_b_y_mps",
                    "meas_delta_v_ib_b_z_mps",
                    "truth_gyro_bias_b_x_radps",
                    "truth_gyro_bias_b_y_radps",
                    "truth_gyro_bias_b_z_radps",
                    "truth_accel_bias_b_x_mps2",
                    "truth_accel_bias_b_y_mps2",
                    "truth_accel_bias_b_z_mps2"});
    }

    void log(const ImuDebugLogPayload& payload)
    {
        const auto& ideal = payload.ideal;
        const auto& measured = payload.measured;
        m_csv.write_row(ideal.time_s,
                        ideal.dt_s,
                        ideal.p_bar_e_m.x(),
                        ideal.p_bar_e_m.y(),
                        ideal.p_bar_e_m.z(),
                        ideal.v_bar_e_mps.x(),
                        ideal.v_bar_e_mps.y(),
                        ideal.v_bar_e_mps.z(),
                        ideal.a_bar_e_mps2.x(),
                        ideal.a_bar_e_mps2.y(),
                        ideal.a_bar_e_mps2.z(),
                        ideal.gravity_e_mps2.x(),
                        ideal.gravity_e_mps2.y(),
                        ideal.gravity_e_mps2.z(),
                        ideal.specific_force_e_mps2.x(),
                        ideal.specific_force_e_mps2.y(),
                        ideal.specific_force_e_mps2.z(),
                        ideal.specific_force_ib_b_mps2.x(),
                        ideal.specific_force_ib_b_mps2.y(),
                        ideal.specific_force_ib_b_mps2.z(),
                        ideal.delta_theta_eb_b_rad.x(),
                        ideal.delta_theta_eb_b_rad.y(),
                        ideal.delta_theta_eb_b_rad.z(),
                        ideal.delta_theta_ib_b_rad.x(),
                        ideal.delta_theta_ib_b_rad.y(),
                        ideal.delta_theta_ib_b_rad.z(),
                        ideal.delta_v_ib_b_mps.x(),
                        ideal.delta_v_ib_b_mps.y(),
                        ideal.delta_v_ib_b_mps.z(),
                        measured.delta_theta_ib_b_rad.x(),
                        measured.delta_theta_ib_b_rad.y(),
                        measured.delta_theta_ib_b_rad.z(),
                        measured.delta_v_ib_b_mps.x(),
                        measured.delta_v_ib_b_mps.y(),
                        measured.delta_v_ib_b_mps.z(),
                        payload.gyro_bias_truth_radps.x(),
                        payload.gyro_bias_truth_radps.y(),
                        payload.gyro_bias_truth_radps.z(),
                        payload.accel_bias_truth_mps2.x(),
                        payload.accel_bias_truth_mps2.y(),
                        payload.accel_bias_truth_mps2.z());
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "imu_debug_ecef_interval_v1"},
                {"file", "imu_debug_ecef.csv"},
                {"description",
                 "Intermediate truth-to-IMU conversion terms for debugging frame, sign, and "
                 "gravity/specific-force behavior."}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "imu_debug_ecef.csv"}, {"manifest", "imu_debug_ecef.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
