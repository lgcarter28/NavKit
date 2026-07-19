// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/ImuIncrementLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class ImuIncrementLogProduct
{
public:
    static constexpr std::string_view LogKey = "imu_nominal";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "imu_nominal.csv",
                   {"time_s",
                    "dt_s",
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
                    "truth_cumsum_delta_theta_ib_b_x_rad",
                    "truth_cumsum_delta_theta_ib_b_y_rad",
                    "truth_cumsum_delta_theta_ib_b_z_rad",
                    "truth_cumsum_delta_v_ib_b_x_mps",
                    "truth_cumsum_delta_v_ib_b_y_mps",
                    "truth_cumsum_delta_v_ib_b_z_mps",
                    "meas_cumsum_delta_theta_ib_b_x_rad",
                    "meas_cumsum_delta_theta_ib_b_y_rad",
                    "meas_cumsum_delta_theta_ib_b_z_rad",
                    "meas_cumsum_delta_v_ib_b_x_mps",
                    "meas_cumsum_delta_v_ib_b_y_mps",
                    "meas_cumsum_delta_v_ib_b_z_mps",
                    "truth_gyro_bias_b_x_radps",
                    "truth_gyro_bias_b_y_radps",
                    "truth_gyro_bias_b_z_radps",
                    "truth_accel_bias_b_x_mps2",
                    "truth_accel_bias_b_y_mps2",
                    "truth_accel_bias_b_z_mps2"});
    }

    void log(const ImuIncrementLogPayload& payload)
    {
        m_csv.write_row(payload.measured.time_s,
                        payload.measured.dt_s,
                        payload.truth.delta_theta_ib_b_rad.x(),
                        payload.truth.delta_theta_ib_b_rad.y(),
                        payload.truth.delta_theta_ib_b_rad.z(),
                        payload.truth.delta_v_ib_b_mps.x(),
                        payload.truth.delta_v_ib_b_mps.y(),
                        payload.truth.delta_v_ib_b_mps.z(),
                        payload.measured.delta_theta_ib_b_rad.x(),
                        payload.measured.delta_theta_ib_b_rad.y(),
                        payload.measured.delta_theta_ib_b_rad.z(),
                        payload.measured.delta_v_ib_b_mps.x(),
                        payload.measured.delta_v_ib_b_mps.y(),
                        payload.measured.delta_v_ib_b_mps.z(),
                        payload.truth_cumsum_delta_theta_ib_b_rad.x(),
                        payload.truth_cumsum_delta_theta_ib_b_rad.y(),
                        payload.truth_cumsum_delta_theta_ib_b_rad.z(),
                        payload.truth_cumsum_delta_v_ib_b_mps.x(),
                        payload.truth_cumsum_delta_v_ib_b_mps.y(),
                        payload.truth_cumsum_delta_v_ib_b_mps.z(),
                        payload.measured_cumsum_delta_theta_ib_b_rad.x(),
                        payload.measured_cumsum_delta_theta_ib_b_rad.y(),
                        payload.measured_cumsum_delta_theta_ib_b_rad.z(),
                        payload.measured_cumsum_delta_v_ib_b_mps.x(),
                        payload.measured_cumsum_delta_v_ib_b_mps.y(),
                        payload.measured_cumsum_delta_v_ib_b_mps.z(),
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
        return {{"schema", "imu_nominal_increment_v1"},
                {"file", "imu_nominal.csv"},
                {"units",
                 {{"time", "s"},
                  {"dt", "s"},
                  {"delta_theta_ib_b", "rad"},
                  {"delta_v_ib_b", "m/s"},
                  {"cumsum_delta_theta_ib_b", "rad"},
                  {"cumsum_delta_v_ib_b", "m/s"},
                  {"gyro_bias_b", "rad/s"},
                  {"accel_bias_b", "m/s^2"}}},
                {"cumulative_sum_semantics", "full_rate_run_cumulative_snapshot"}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "imu_nominal.csv"}, {"manifest", "imu_nominal.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
