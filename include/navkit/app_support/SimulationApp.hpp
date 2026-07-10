// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/LoggingConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntime.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/logging/MeasurementStatisticsLogger.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"

#include <cstdio>
#include <filesystem>
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
    using Logger = LoggerConfig_t<Config>;
    using Emulators = EmulatorRuntime<NavKit, Logger, EmulatorBindings>;

    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        validate_runtime_config<Config>(cfg);

        const auto run_settings = run_settings_from_json(cfg);
        const auto trajectory = trajectory_run_from_json(cfg);
        auto emulator_runtimes = Emulators::make_runtimes(cfg);

        reset_profile_sink_if_configured<NavKit>();

        Navigator navigator;
        auto& filter = navigator.filter();
        const auto nav_initialization = NavInitializationProvider::initialize(cfg, trajectory);
        initialize_navigator<StateDef>(navigator, nav_initialization);
        TransferAlignmentProvider::template transfer_align<Navigator>(navigator, cfg, trajectory);

        Logger logger(run_settings.output_dir, run_settings.run_name, cfg);
        Emulators::configure(navigator, logger, cfg);

        for (const auto& sample : trajectory.truth_samples) {
            logger.log(sample);
            Emulators::process(navigator, logger, emulator_runtimes, sample);

            navigator.process_measurements();

            log_measurement_statistics<typename NavKit::Sensors>(logger, filter);
            logger.log(io::NavEstimateLogPayload<StateDef, Filter>{
                .time_s = sample.time,
                .filter = filter,
                .truth = sample,
            });
        }

        logger.close();
        export_profile_if_configured<NavKit>(run_settings.output_dir, run_settings.run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n",
                    run_settings.output_dir.string().c_str());
        return 0;
    }
};

} // namespace navkit::app_support
