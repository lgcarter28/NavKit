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
    static constexpr std::string_view LogKey = "imu_debug_body";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "imu_debug_body.csv",
                   {"time_s",
                    "dt_s",
                    "omega_ib_b_x_radps",
                    "omega_ib_b_y_radps",
                    "omega_ib_b_z_radps",
                    "specific_force_ib_b_x_mps2",
                    "specific_force_ib_b_y_mps2",
                    "specific_force_ib_b_z_mps2",
                    "truth_delta_theta_ib_b_x_rad",
                    "truth_delta_theta_ib_b_y_rad",
                    "truth_delta_theta_ib_b_z_rad",
                    "truth_delta_v_ib_b_x_mps",
                    "truth_delta_v_ib_b_y_mps",
                    "truth_delta_v_ib_b_z_mps",
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
        const navkit::sim::ImuInterval& interval = payload.debug.interval;
        const navkit::core::estimation::ImuIncrement& truth = payload.truth;
        const navkit::core::estimation::ImuIncrement& measured = payload.measured;
        m_csv.write_row(interval.time_s,
                        interval.dt_s,
                        interval.omega_ib_b_radps.x(),
                        interval.omega_ib_b_radps.y(),
                        interval.omega_ib_b_radps.z(),
                        interval.specific_force_ib_b_mps2.x(),
                        interval.specific_force_ib_b_mps2.y(),
                        interval.specific_force_ib_b_mps2.z(),
                        truth.delta_theta_ib_b_rad.x(),
                        truth.delta_theta_ib_b_rad.y(),
                        truth.delta_theta_ib_b_rad.z(),
                        truth.delta_v_ib_b_mps.x(),
                        truth.delta_v_ib_b_mps.y(),
                        truth.delta_v_ib_b_mps.z(),
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
        return {{"schema", "imu_debug_body_interval_v1"},
                {"file", "imu_debug_body.csv"},
                {"description",
                 "Body-resolved truth-to-IMU conversion terms for debugging frame, sign, and "
                 "specific-force behavior."}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "imu_debug_body.csv"}, {"manifest", "imu_debug_body.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
