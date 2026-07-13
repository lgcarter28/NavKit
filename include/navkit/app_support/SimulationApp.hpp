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

        const auto run_settings = run_settings_from_json(cfg);
        const auto trajectory = trajectory_run_from_json(cfg);
        auto emulator_runtimes = Emulators::make_runtimes(cfg);
        Imu imu_runtime(cfg);

        reset_profile_sink_if_configured<NavKit>();

        auto navigator_storage = std::make_unique<Navigator>();
        auto& navigator = *navigator_storage;
        auto& filter = navigator.filter();
        const auto nav_initialization = NavInitializationProvider::initialize(cfg, trajectory);
        initialize_navigator<StateDef>(navigator, nav_initialization);
        TransferAlignmentProvider::template transfer_align<Navigator>(navigator, cfg, trajectory);

        Logger logger(run_settings.output_dir, run_settings.run_name, cfg);
        Emulators::configure(navigator, logger, cfg);

        core::Time_t next_console_log_s{0.0};
        core::Time_t next_truth_log_s{0.0};
        core::Time_t next_nav_log_s{0.0};
        core::Time_t next_measurement_statistics_log_s{0.0};

        for (const auto& sample : trajectory.truth_samples) {
            if (should_log_at(sample.time, run_settings.logging.truth_dt_s, next_truth_log_s)) {
                logger.log(sample);
            }
            if (!imu_runtime.process(navigator, sample)) {
                std::printf("IMU runtime failure at t=%f: %s\n",
                            sample.time,
                            imu_runtime.last_error().data());
                return 2;
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
                    .truth = sample,
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
        const auto p_e = filter.state().template segment<3>(StateDef::Pos::i);
        const auto v_e = filter.state().template segment<3>(StateDef::Vel::i);
        Eigen::Matrix<core::Scalar_t, 4, 1> q_e2b{};
        if constexpr (requires { typename StateDef::Quat; }) {
            q_e2b = filter.state().template segment<4>(StateDef::Quat::i);
        }
        else {
            q_e2b << 1.0, 0.0, 0.0, 0.0;
        }
        std::printf("t=%8.3f s | p_e=[%.3f %.3f %.3f] m | v_e=[%.6f %.6f %.6f] m/s | q_e2b=[%.6e "
                    "%.6e %.6e %.6e]\n",
                    time_s,
                    p_e.x(),
                    p_e.y(),
                    p_e.z(),
                    v_e.x(),
                    v_e.y(),
                    v_e.z(),
                    q_e2b(0),
                    q_e2b(1),
                    q_e2b(2),
                    q_e2b(3));
    }
};

} // namespace navkit::app_support
