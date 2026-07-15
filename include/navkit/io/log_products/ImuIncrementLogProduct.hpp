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
        m_truth_delta_theta_sum += payload.ideal.delta_theta_ib_b_rad;
        m_truth_delta_v_sum += payload.ideal.delta_v_ib_b_mps;
        m_measured_delta_theta_sum += payload.measured.delta_theta_ib_b_rad;
        m_measured_delta_v_sum += payload.measured.delta_v_ib_b_mps;

        m_csv.write_row(payload.measured.time_s,
                        payload.measured.dt_s,
                        payload.ideal.delta_theta_ib_b_rad.x(),
                        payload.ideal.delta_theta_ib_b_rad.y(),
                        payload.ideal.delta_theta_ib_b_rad.z(),
                        payload.ideal.delta_v_ib_b_mps.x(),
                        payload.ideal.delta_v_ib_b_mps.y(),
                        payload.ideal.delta_v_ib_b_mps.z(),
                        payload.measured.delta_theta_ib_b_rad.x(),
                        payload.measured.delta_theta_ib_b_rad.y(),
                        payload.measured.delta_theta_ib_b_rad.z(),
                        payload.measured.delta_v_ib_b_mps.x(),
                        payload.measured.delta_v_ib_b_mps.y(),
                        payload.measured.delta_v_ib_b_mps.z(),
                        m_truth_delta_theta_sum.x(),
                        m_truth_delta_theta_sum.y(),
                        m_truth_delta_theta_sum.z(),
                        m_truth_delta_v_sum.x(),
                        m_truth_delta_v_sum.y(),
                        m_truth_delta_v_sum.z(),
                        m_measured_delta_theta_sum.x(),
                        m_measured_delta_theta_sum.y(),
                        m_measured_delta_theta_sum.z(),
                        m_measured_delta_v_sum.x(),
                        m_measured_delta_v_sum.y(),
                        m_measured_delta_v_sum.z(),
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
                  {"gyro_bias_b", "rad/s"},
                  {"accel_bias_b", "m/s^2"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "imu_nominal.csv"}, {"manifest", "imu_nominal.meta.json"}};
    }

private:
    CsvWriter m_csv;
    core::Vec3 m_truth_delta_theta_sum{core::Vec3::Zero()};
    core::Vec3 m_truth_delta_v_sum{core::Vec3::Zero()};
    core::Vec3 m_measured_delta_theta_sum{core::Vec3::Zero()};
    core::Vec3 m_measured_delta_v_sum{core::Vec3::Zero()};
};

} // namespace navkit::io
