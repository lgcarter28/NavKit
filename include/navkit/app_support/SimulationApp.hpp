// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/ConfigTraits.hpp"
#include "navkit/app_support/EmulatorBinding.hpp"
#include "navkit/app_support/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/EmulatorRuntime.hpp"
#include "navkit/app_support/FilterInitialization.hpp"
#include "navkit/app_support/JsonInput.hpp"
#include "navkit/app_support/LoggingConfigTraits.hpp"
#include "navkit/app_support/MeasurementStatisticsLogger.hpp"
#include "navkit/app_support/ProfileExport.hpp"
#include "navkit/app_support/RunSettings.hpp"
#include "navkit/app_support/RuntimeConfigValidation.hpp"
#include "navkit/app_support/TrajectoryProvider.hpp"

#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <tuple>

namespace navkit::app_support
{

template<typename Config>
concept SimulationAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    emulator_binding_ids_unique_v<typename Config::EmulatorBindings> &&
    emulator_binding_sensors_valid_v<typename Config::EmulatorBindings,
                                     typename Config::NavKit::Sensors>;

template<SimulationAppConfigPolicy Config>
class SimulationApp
{
public:
    using NavKit = NavKitConfig_t<Config>;
    using StateDef = typename NavKit::StateDef;
    using Filter = typename NavKit::Filter;
    using Navigator = typename NavKit::Navigator;
    using EmulatorBindings = typename Config::EmulatorBindings;
    using Emulators = EmulatorRuntime<NavKit, EmulatorBindings>;
    using Logger = LoggerConfig_t<Config>;

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
        configure_initial_filter_state<StateDef>(filter, cfg, trajectory.initial_position_e_m);

        Logger logger(run_settings.output_dir, run_settings.run_name, cfg);
        Emulators::configure(navigator, logger, cfg);

        for (const auto& sample : trajectory.truth_samples) {
            logger.log_truth(sample);
            Emulators::process(navigator, logger, emulator_runtimes, sample);

            navigator.process_measurements();

            log_measurement_statistics<typename NavKit::MeasurementStatisticsTuple>(logger, filter);
            logger.log_nav<StateDef>(sample.time, filter, sample);
        }

        logger.close();
        export_profile_if_configured<NavKit>(run_settings.output_dir, run_settings.run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n",
                    run_settings.output_dir.string().c_str());
        return 0;
    }
};

} // namespace navkit::app_support
