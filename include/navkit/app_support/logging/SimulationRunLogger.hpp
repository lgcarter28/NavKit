// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/logging/MeasurementStatisticsLogger.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/io/log_payloads/FilterCorrectionLogPayload.hpp"
#include "navkit/io/log_payloads/ImuDebugLogPayload.hpp"
#include "navkit/io/log_payloads/ImuIncrementLogPayload.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <Eigen/Dense>
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

    void log_truth_if_due(const sim::TruthSample& sample)
    {
        if (should_log_at(sample.time,
                          m_run_settings.logging.truth_enabled,
                          m_run_settings.logging.truth_dt_s,
                          m_next_truth_log_s)) {
            m_logger.log(sample);
        }
    }

    void log_imu_if_due(const sim::TruthSample& sample, const ImuRuntimeSample& imu_sample)
    {
        if (!imu_sample.generated) {
            return;
        }
        if (should_log_at(sample.time,
                          m_run_settings.logging.imu_enabled,
                          m_run_settings.logging.imu_dt_s,
                          m_next_imu_log_s)) {
            log_if_supported(io::ImuIncrementLogPayload{
                .truth = imu_sample.truth,
                .measured = imu_sample.measured,
                .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
        }
        if (should_log_at(sample.time,
                          m_run_settings.logging.imu_debug_enabled,
                          m_run_settings.logging.imu_debug_dt_s,
                          m_next_imu_debug_log_s)) {
            log_if_supported(
                io::ImuDebugLogPayload{.debug = imu_sample.debug,
                                       .truth = imu_sample.truth,
                                       .measured = imu_sample.measured,
                                       .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                                       .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
        }
    }

    void log_filter_if_due(const core::Time_t time_s, const Filter& filter)
    {
        if (should_log_at(time_s,
                          m_run_settings.logging.measurement_statistics_enabled,
                          m_run_settings.logging.measurement_statistics_dt_s,
                          m_next_measurement_statistics_log_s)) {
            log_measurement_statistics<typename NavKit::Sensors>(m_logger, filter);
        }
        if (should_log_at(time_s,
                          m_run_settings.logging.nav_enabled,
                          m_run_settings.logging.nav_dt_s,
                          m_next_nav_log_s)) {
            m_logger.log(io::NavEstimateLogPayload<StateDef, Filter>{
                .time_s = time_s,
                .filter = filter,
            });
        }
        if (filter.last_correction_valid() &&
            should_log_at(time_s,
                          m_run_settings.logging.filter_correction_enabled,
                          m_run_settings.logging.filter_correction_dt_s,
                          m_next_filter_correction_log_s)) {
            log_if_supported(io::FilterCorrectionLogPayload<StateDef, Filter>{
                .time_s = time_s,
                .filter = filter,
            });
        }
        if (should_log_at(time_s,
                          m_run_settings.logging.console_enabled,
                          m_run_settings.logging.console_dt_s,
                          m_next_console_log_s)) {
            print_console_status(time_s, filter);
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

    [[nodiscard]] static bool should_log_at(const core::Time_t time_s,
                                            const bool enabled,
                                            const core::Time_t dt_s,
                                            core::Time_t& next_time_s)
    {
        constexpr core::Time_t epsilon_s = 1.0e-12;
        if (!enabled || dt_s <= 0.0 || (time_s + epsilon_s) < next_time_s) {
            return false;
        }
        while (next_time_s <= (time_s + epsilon_s)) {
            next_time_s += dt_s;
        }
        return true;
    }

    static void print_console_status(const core::Time_t time_s, const Filter& filter)
    {
        using Nominal = typename StateDef::Nominal;

        const core::Vec3 p_e = filter.state().template segment<3>(Nominal::Pos::i);
        const core::Vec3 v_e = filter.state().template segment<3>(Nominal::Vel::i);
        const Eigen::Matrix<core::Scalar_t, 4, 1> q_b2e =
            filter.state().template segment<4>(Nominal::AttQuat::i);
        std::printf("t=%8.3f s | p_e=[%.3f %.3f %.3f] m | v_e=[%.6f %.6f %.6f] m/s | q_b2e=[%.6e "
                    "%.6e %.6e %.6e]\n",
                    time_s,
                    p_e.x(),
                    p_e.y(),
                    p_e.z(),
                    v_e.x(),
                    v_e.y(),
                    v_e.z(),
                    q_b2e(0),
                    q_b2e(1),
                    q_b2e(2),
                    q_b2e(3));
    }

    Logger& m_logger;
    const RunSettings& m_run_settings;
    core::Time_t m_next_console_log_s{0.0};
    core::Time_t m_next_truth_log_s{0.0};
    core::Time_t m_next_nav_log_s{0.0};
    core::Time_t m_next_measurement_statistics_log_s{0.0};
    core::Time_t m_next_imu_log_s{0.0};
    core::Time_t m_next_imu_debug_log_s{0.0};
    core::Time_t m_next_filter_correction_log_s{0.0};
};

} // namespace navkit::app_support
