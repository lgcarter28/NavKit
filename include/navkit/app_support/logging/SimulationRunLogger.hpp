// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/logging/MeasurementStatisticsLogger.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalSchedule.hpp"
#include "navkit/io/log_payloads/FilterCorrectionLogPayload.hpp"
#include "navkit/io/log_payloads/ImuDebugLogPayload.hpp"
#include "navkit/io/log_payloads/ImuIncrementLogPayload.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>

namespace navkit::app_support
{

template<SimulationAppConfigPolicy Config>
class SimulationRunLogger
{
public:
    using NavKit = NavKitConfig_t<Config>;
    using StateDef = typename NavKit::StateDef;
    using Filter = typename NavKit::Filter;
    using Logger = LoggerConfig_t<Config>;

    explicit SimulationRunLogger(Logger& logger, const RunSettings& run_settings)
        : m_logger(logger)
        , m_run_settings(run_settings)
    {}

    [[nodiscard]] bool initialize(const core::Timestamp& t_epoch)
    {
        if (m_run_settings.logging.console_enabled &&
            !m_console_schedule.initialize(t_epoch, m_run_settings.logging.console_rate)) {
            return false;
        }
        if (m_run_settings.logging.truth_enabled &&
            !m_truth_schedule.initialize(t_epoch, m_run_settings.logging.truth_rate)) {
            return false;
        }
        if (m_run_settings.logging.nav_enabled &&
            !m_nav_schedule.initialize(t_epoch, m_run_settings.logging.nav_rate)) {
            return false;
        }
        if (m_run_settings.logging.measurement_statistics_enabled &&
            !m_measurement_statistics_schedule.initialize(
                t_epoch, m_run_settings.logging.measurement_statistics_rate)) {
            return false;
        }
        if (m_run_settings.logging.imu_enabled &&
            !m_imu_schedule.initialize(t_epoch, m_run_settings.logging.imu_rate)) {
            return false;
        }
        if (m_run_settings.logging.imu_debug_enabled &&
            !m_imu_debug_schedule.initialize(t_epoch, m_run_settings.logging.imu_debug_rate)) {
            return false;
        }
        if (m_run_settings.logging.filter_correction_enabled &&
            !m_filter_correction_schedule.initialize(
                t_epoch, m_run_settings.logging.filter_correction_rate)) {
            return false;
        }
        return true;
    }

    void log_truth_if_due(const sim::TruthSample& sample)
    {
        if (m_run_settings.logging.truth_enabled && m_truth_schedule.due(sample.t)) {
            m_logger.log(sample);
        }
    }

    void log_imu_if_due(const sim::TruthSample& sample, const ImuRuntimeSample& imu_sample)
    {
        if (!imu_sample.generated) {
            return;
        }
        if (m_run_settings.logging.imu_enabled && m_imu_schedule.due(sample.t)) {
            log_if_supported(io::ImuIncrementLogPayload{
                .truth = imu_sample.truth,
                .measured = imu_sample.measured,
                .truth_cumsum_delta_theta_ib_b_rad = imu_sample.truth_cumsum_delta_theta_ib_b_rad,
                .truth_cumsum_delta_v_ib_b_mps = imu_sample.truth_cumsum_delta_v_ib_b_mps,
                .measured_cumsum_delta_theta_ib_b_rad =
                    imu_sample.measured_cumsum_delta_theta_ib_b_rad,
                .measured_cumsum_delta_v_ib_b_mps = imu_sample.measured_cumsum_delta_v_ib_b_mps,
                .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
        }
        if (m_run_settings.logging.imu_debug_enabled && m_imu_debug_schedule.due(sample.t)) {
            log_if_supported(
                io::ImuDebugLogPayload{.debug = imu_sample.debug,
                                       .truth = imu_sample.truth,
                                       .measured = imu_sample.measured,
                                       .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                                       .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
        }
    }

    void log_filter_if_due(const core::Timestamp& t, const Filter& filter)
    {
        if (m_run_settings.logging.measurement_statistics_enabled &&
            m_measurement_statistics_schedule.due(t)) {
            log_measurement_statistics<typename NavKit::Sensors>(m_logger, filter);
        }
        if (m_run_settings.logging.nav_enabled && m_nav_schedule.due(t)) {
            m_logger.log(io::NavEstimateLogPayload<StateDef, Filter>{
                .time_s = core::timestamp_seconds(t),
                .filter = filter,
            });
        }
        if (filter.last_correction_valid() && m_run_settings.logging.filter_correction_enabled &&
            m_filter_correction_schedule.due(t)) {
            log_if_supported(io::FilterCorrectionLogPayload<StateDef, Filter>{
                .time_s = core::timestamp_seconds(t),
                .filter = filter,
            });
        }
        if (m_run_settings.logging.console_enabled && m_console_schedule.due(t)) {
            print_console_status(core::timestamp_seconds(t), filter);
        }
    }

private:
    template<typename Payload>
    void log_if_supported(const Payload& payload)
    {
        if constexpr (requires { Logger::template matching_product_count_v<Payload>; }) {
            if constexpr (Logger::template matching_product_count_v<Payload> == 1U) {
                m_logger.log(payload);
            }
        }
    }

    static void print_console_status(const core::Time_t time_s, const Filter& filter)
    {
        using Nominal = typename StateDef::Nominal;

        int hours = 0;
        int minutes = 0;
        core::Scalar_t seconds = 0.0;
        split_time_hh_mm_ss(time_s, hours, minutes, seconds);

        const core::Vec3 p_e = filter.state().template segment<3>(Nominal::Pos::i);
        const core::Vec3 v_e = filter.state().template segment<3>(Nominal::Vel::i);
        const Eigen::Matrix<core::Scalar_t, 4, 1> q_b2e =
            filter.state().template segment<4>(Nominal::AttQuat::i);
        const Eigen::Quaternion<core::Scalar_t> q_b2e_quat{q_b2e(0), q_b2e(1), q_b2e(2), q_b2e(3)};
        const core::Mat3 C_e2n = core::frames::ecef_to_ned_matrix(p_e);
        const Eigen::Quaternion<core::Scalar_t> q_e2n{C_e2n};
        const Eigen::Quaternion<core::Scalar_t> q_b2n = q_e2n * q_b2e_quat.normalized();
        const core::Vec3 p_lla_deg_m = core::frames::ecef_m_to_lla_deg_m(p_e);
        const core::Vec3 v_n_mps = C_e2n * v_e;
        const core::Vec3 rpy_b2n_deg =
            core::math::rpy_rad_from_quaternion(q_b2n) * (180.0 / 3.14159265358979323846);

        std::printf("t=%02d:%02d:%06.3f | lat=%11.6f deg lon=%12.6f deg h=%9.2f m | "
                    "v_ned=[%9.3f %9.3f %9.3f] m/s | "
                    "rpy_b2n=[%8.3f %8.3f %8.3f] deg\n",
                    hours,
                    minutes,
                    seconds,
                    p_lla_deg_m.x(),
                    p_lla_deg_m.y(),
                    p_lla_deg_m.z(),
                    v_n_mps.x(),
                    v_n_mps.y(),
                    v_n_mps.z(),
                    rpy_b2n_deg.x(),
                    rpy_b2n_deg.y(),
                    rpy_b2n_deg.z());
    }

    static void split_time_hh_mm_ss(const core::Time_t time_s,
                                    int& hours,
                                    int& minutes,
                                    core::Scalar_t& seconds)
    {
        const core::Time_t nonnegative_time_s = time_s < 0.0 ? 0.0 : time_s;
        const auto whole_seconds = static_cast<int>(std::floor(nonnegative_time_s));
        hours = whole_seconds / 3600;
        minutes = (whole_seconds % 3600) / 60;
        seconds = nonnegative_time_s - static_cast<core::Time_t>((hours * 3600) + (minutes * 60));
    }

    Logger& m_logger;
    const RunSettings& m_run_settings;
    core::RationalSchedule m_console_schedule{};
    core::RationalSchedule m_truth_schedule{};
    core::RationalSchedule m_nav_schedule{};
    core::RationalSchedule m_measurement_statistics_schedule{};
    core::RationalSchedule m_imu_schedule{};
    core::RationalSchedule m_imu_debug_schedule{};
    core::RationalSchedule m_filter_correction_schedule{};
};

} // namespace navkit::app_support
