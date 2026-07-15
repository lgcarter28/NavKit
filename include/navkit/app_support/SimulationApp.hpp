// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntime.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/logging/MeasurementStatisticsLogger.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/io/log_payloads/FilterCorrectionLogPayload.hpp"
#include "navkit/io/log_payloads/ImuDebugLogPayload.hpp"
#include "navkit/io/log_payloads/ImuIncrementLogPayload.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"

#include <Eigen/Dense>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<SimulationAppConfigPolicy Config>
class SimulationApp
{
public:
    using NavKit = NavKitConfig_t<Config>;
    using StateDef = typename NavKit::StateDef;
    using Filter = typename NavKit::Filter;
    using Navigator = typename NavKit::Navigator;
    using EmulatorBindings = typename Config::EmulatorBindings;
    using NavInitializationProvider = typename Config::NavInitializationProvider;
    using TransferAlignmentProvider = typename Config::TransferAlignmentProvider;
    using ImuSimulator = typename Config::ImuSimulator;
    using Logger = LoggerConfig_t<Config>;
    using Emulators = EmulatorRuntime<NavKit, Logger, EmulatorBindings>;
    using Imu = ImuRuntime<ImuSimulator>;

    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        validate_runtime_config<Config>(cfg);

        const RunSettings run_settings = run_settings_from_json(cfg);
        const TrajectoryRun trajectory = trajectory_run_from_json(cfg);
        auto emulator_runtimes = Emulators::make_runtimes(cfg);
        Imu imu_runtime(cfg);

        reset_profile_sink_if_configured<NavKit>();

        std::unique_ptr<Navigator> navigator_storage = std::make_unique<Navigator>();
        Navigator& navigator = *navigator_storage;
        Filter& filter = navigator.filter();
        const NavInitialization nav_initialization =
            NavInitializationProvider::initialize(cfg, trajectory);
        initialize_navigator<StateDef>(navigator, nav_initialization);
        TransferAlignmentProvider::template transfer_align<Navigator>(navigator, cfg, trajectory);

        Logger logger(run_settings.output_dir, run_settings.run_name, cfg);
        Emulators::configure(navigator, logger, cfg);

        core::Time_t next_console_log_s{0.0};
        core::Time_t next_truth_log_s{0.0};
        core::Time_t next_nav_log_s{0.0};
        core::Time_t next_measurement_statistics_log_s{0.0};
        core::Time_t next_imu_log_s{0.0};
        core::Time_t next_imu_debug_log_s{0.0};
        core::Time_t next_filter_correction_log_s{0.0};

        for (const auto& sample : trajectory.truth_samples) {
            if (should_log_at(sample.time, run_settings.logging.truth_dt_s, next_truth_log_s)) {
                logger.log(sample);
            }
            ImuRuntimeSample imu_sample{};
            if (!imu_runtime.process(sample, navigator, imu_sample)) {
                std::printf("IMU runtime failure at t=%f: %s\n",
                            sample.time,
                            imu_runtime.last_error().data());
                return 2;
            }
            if (imu_sample.generated &&
                should_log_at(sample.time, run_settings.logging.imu_dt_s, next_imu_log_s)) {
                log_if_supported(logger,
                                 io::ImuIncrementLogPayload{
                                     .ideal = imu_sample.ideal,
                                     .measured = imu_sample.measured,
                                     .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                                     .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
            }
            if (imu_sample.generated && should_log_at(sample.time,
                                                      run_settings.logging.imu_debug_dt_s,
                                                      next_imu_debug_log_s)) {
                log_if_supported(logger,
                                 io::ImuDebugLogPayload{
                                     .ideal = imu_sample.ideal,
                                     .measured = imu_sample.measured,
                                     .gyro_bias_truth_radps = imu_sample.gyro_bias_truth_radps,
                                     .accel_bias_truth_mps2 = imu_sample.accel_bias_truth_mps2});
            }
            Emulators::process(navigator, logger, emulator_runtimes, sample);

            if (!navigator.update()) {
                std::printf("Navigator propagation failed at t=%f\n", sample.time);
                return 4;
            }

            if (should_log_at(sample.time,
                              run_settings.logging.measurement_statistics_dt_s,
                              next_measurement_statistics_log_s)) {
                log_measurement_statistics<typename NavKit::Sensors>(logger, filter);
            }
            if (should_log_at(sample.time, run_settings.logging.nav_dt_s, next_nav_log_s)) {
                logger.log(io::NavEstimateLogPayload<StateDef, Filter>{
                    .time_s = sample.time,
                    .filter = filter,
                });
            }
            if (filter.last_correction_valid() &&
                should_log_at(sample.time,
                              run_settings.logging.filter_correction_dt_s,
                              next_filter_correction_log_s)) {
                log_if_supported(logger,
                                 io::FilterCorrectionLogPayload<StateDef, Filter>{
                                     .time_s = sample.time,
                                     .filter = filter,
                                 });
            }
            if (should_log_at(sample.time, run_settings.logging.console_dt_s, next_console_log_s)) {
                print_console_status(sample.time, filter);
            }
        }

        logger.close();
        export_profile_if_configured<NavKit>(run_settings.output_dir, run_settings.run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n",
                    run_settings.output_dir.string().c_str());
        return 0;
    }

private:
    template<typename Payload>
    static void log_if_supported(Logger& logger, const Payload& payload)
    {
        if constexpr (requires { Logger::template matching_product_count_v<Payload>; }) {
            if constexpr (Logger::template matching_product_count_v<Payload> == 1U) {
                logger.log(payload);
            }
        }
    }

    [[nodiscard]] static bool
    should_log_at(const core::Time_t time_s, const core::Time_t dt_s, core::Time_t& next_time_s)
    {
        constexpr core::Time_t epsilon_s = 1.0e-12;
        if (dt_s <= 0.0 || (time_s + epsilon_s) < next_time_s) {
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
};

} // namespace navkit::app_support
