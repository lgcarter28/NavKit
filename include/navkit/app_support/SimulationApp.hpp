// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntime.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/logging/SimulationRunLogger.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

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

        Logger logger(run_settings.data_dir, run_settings.run_name, cfg);
        SimulationRunLogger<Config> run_logger(logger, run_settings);
        Emulators::configure(navigator, logger, cfg);

        for (const auto& sample : trajectory.truth_samples) {
            run_logger.log_truth_if_due(sample);
            ImuRuntimeSample imu_sample{};
            if (!imu_runtime.process(sample, navigator, imu_sample)) {
                std::printf("IMU runtime failure at t=%f: %s\n",
                            sample.time,
                            imu_runtime.last_error().data());
                return 2;
            }
            run_logger.log_imu_if_due(sample, imu_sample);
            Emulators::process(navigator, logger, emulator_runtimes, sample);

            if (!navigator.update()) {
                std::printf("Navigator propagation failed at t=%f\n", sample.time);
                return 4;
            }
            run_logger.log_filter_if_due(sample.time, filter);
        }

        logger.close();
        export_profile_if_configured<NavKit>(run_settings.data_dir, run_settings.run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n",
                    run_settings.data_dir.string().c_str());
        return 0;
    }
};

} // namespace navkit::app_support
