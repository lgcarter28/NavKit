// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/SimulationAppConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorRuntime.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/initialization/InitialTruthReference.hpp"
#include "navkit/app_support/logging/SimulationRunLogger.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/runtime/RuntimeConfigValidation.hpp"
#include "navkit/app_support/time/ClockFactory.hpp"
#include "navkit/app_support/trajectory/TrajectoryControlState.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/time/RationalTimeline.hpp"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <tuple>

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
    using Propagation = typename NavKit::Propagation;
    using EmulatorBindings = typename Config::EmulatorBindings;
    using NavInitializationProvider = typename Config::NavInitializationProvider;
    using TransferAlignmentProvider = typename Config::TransferAlignmentProvider;
    using ImuSimulator = typename Config::ImuSimulator;
    using Logger = RuntimeLogger<NavKit>;
    using Emulators = EmulatorRuntime<NavKit, Logger, EmulatorBindings>;
    using Imu = ImuRuntime<ImuSimulator>;

    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        validate_runtime_config<Config>(cfg);

        const RunSettings run_settings = run_settings_from_json(cfg);
        TrajectoryRun trajectory = trajectory_run_from_json(cfg, config_path.parent_path());
        auto emulator_runtimes = Emulators::make_runtimes(cfg);
        Imu imu_runtime(cfg);

        if (!trajectory.source) {
            std::printf("Simulation trajectory source was not created.\n");
            return 3;
        }
        const core::Timestamp t_start = trajectory.source->t_start();
        if (!trajectory.source->advance_to(t_start)) {
            std::printf("Simulation trajectory source failed to advance to its start time.\n");
            return 3;
        }
        sim::TruthSample initial_truth{};
        if (!trajectory.source->query(t_start, initial_truth)) {
            std::printf("Simulation trajectory source failed to provide initial truth.\n");
            return 3;
        }
        if (!imu_runtime.initialize(initial_truth)) {
            std::printf("IMU runtime initialization failed: %s\n", imu_runtime.last_error().data());
            return 2;
        }

        reset_profile_sink_if_configured<NavKit>();

        std::unique_ptr<Navigator> navigator_storage = std::make_unique<Navigator>();
        Navigator& navigator = *navigator_storage;
        Filter& filter = navigator.filter();
        const PvaInitialization pva_initialization =
            NavInitializationProvider::initialize(cfg, trajectory);
        InitialTruthReference<StateDef> truth_reference{};
        populate_initial_pva_from_truth<StateDef>(initial_truth, truth_reference);
        apply_initial_truth_reference_from_runtimes<StateDef>(
            std::tie(imu_runtime, emulator_runtimes), truth_reference);
        initialize_navigator<NavKit>(pva_initialization, cfg, truth_reference, navigator);
        TransferAlignmentProvider::template transfer_align<Navigator>(navigator, cfg, trajectory);

        Logger logger(run_settings.data_dir, run_settings.run_name, cfg);
        SimulationRunLogger<Config> run_logger(logger, run_settings);
        if (!run_logger.initialize(t_start)) {
            std::printf("Simulation run logger initialization failed\n");
            return 5;
        }
        Emulators::configure(navigator, logger, cfg);
        std::unique_ptr<Clock> clock = clock_from_mode(run_settings.clock_mode);
        if (!clock || !clock->initialize(t_start)) {
            std::printf("Simulation clock initialization failed\n");
            return 5;
        }

        core::RationalTimeline app_timeline{};
        if (!app_timeline.initialize(t_start, run_settings.application_rate)) {
            std::printf("Application timeline initialization failed\n");
            return 5;
        }

        typename Emulators::PreparedUpdates prepared_updates{};
        if (!Emulators::prepare(*trajectory.source, t_start, emulator_runtimes, prepared_updates)) {
            std::printf("Simulation emulator preparation failed at t=%f\n",
                        core::timestamp_seconds(t_start));
            return 2;
        }
        if (!clock->wait_until(t_start)) {
            std::printf("Simulation clock failed to reach initialization time t=%f\n",
                        core::timestamp_seconds(t_start));
            return 4;
        }
        if (!Emulators::publish(prepared_updates, emulator_runtimes, navigator, logger)) {
            std::printf("Simulation emulator publication failed at t=%f\n",
                        core::timestamp_seconds(t_start));
            return 4;
        }
        if (!navigator.update()) {
            std::printf("Navigator initialization update failed at t=%f\n",
                        core::timestamp_seconds(t_start));
            return 4;
        }
        if (!supply_control_state(
                run_settings, t_start, initial_truth, filter, *trajectory.source)) {
            std::printf("Trajectory control-state publication failed at t=%f\n",
                        core::timestamp_seconds(t_start));
            return 4;
        }
        run_logger.log_truth_if_due(initial_truth);
        sim::TrajectoryDiagnostics initial_trajectory_diagnostics{};
        if (!trajectory.source->query_diagnostics(t_start, initial_trajectory_diagnostics) ||
            !run_logger.log_trajectory_if_due(initial_truth, initial_trajectory_diagnostics)) {
            std::printf("Trajectory diagnostics logging failed at initialization time\n");
            return 6;
        }
        run_logger.log_filter_if_due(t_start, filter);

        bool navigator_finalized = false;
        while (!trajectory.source->is_complete()) {
            core::Timestamp t_curr{};
            if (!app_timeline.next(t_curr)) {
                std::printf("Application timeline overflow\n");
                return 7;
            }
            if (!trajectory.source->advance_to(t_curr)) {
                if (trajectory.source->is_complete()) {
                    break;
                }
                std::printf("Unable to advance trajectory source to application timestamp\n");
                return 6;
            }
            sim::TruthSample truth{};
            if (!trajectory.source->query(t_curr, truth)) {
                std::printf("Unable to query trajectory truth at application timestamp\n");
                return 6;
            }

            ImuRuntimeSample imu_sample{};
            prepared_updates = {};
            if (!imu_runtime.prepare(*trajectory.source, t_curr, imu_sample) ||
                !Emulators::prepare(
                    *trajectory.source, t_curr, emulator_runtimes, prepared_updates)) {
                std::printf("Emulator preparation failed at t=%f: %s\n",
                            core::timestamp_seconds(t_curr),
                            imu_runtime.last_error().data());
                return 2;
            }
            if (!clock->wait_until(t_curr)) {
                std::printf("Simulation clock failed to reach t=%f\n",
                            core::timestamp_seconds(t_curr));
                return 4;
            }
            if (!imu_runtime.publish(imu_sample, navigator)) {
                std::printf("IMU publication failed at t=%f: %s\n",
                            core::timestamp_seconds(t_curr),
                            imu_runtime.last_error().data());
                return 2;
            }
            if (imu_sample.generated &&
                !trajectory.source->observe_imu_increment(imu_sample.measured)) {
                std::printf("Trajectory IMU observation failed at t=%f\n",
                            core::timestamp_seconds(t_curr));
                return 4;
            }
            if (!Emulators::publish(prepared_updates, emulator_runtimes, navigator, logger)) {
                std::printf("Simulation emulator publication failed at t=%f\n",
                            core::timestamp_seconds(t_curr));
                return 4;
            }
            const bool trajectory_complete = trajectory.source->is_complete();
            const bool navigator_updated =
                trajectory_complete ? navigator.finalize() : navigator.update();
            if (!navigator_updated) {
                std::printf("Navigator update failed at t=%f\n", core::timestamp_seconds(t_curr));
                return 4;
            }
            navigator_finalized = trajectory_complete;
            if (!supply_control_state(run_settings, t_start, truth, filter, *trajectory.source)) {
                std::printf("Trajectory control-state publication failed at t=%f\n",
                            core::timestamp_seconds(t_curr));
                return 4;
            }
            run_logger.log_truth_if_due(truth);
            sim::TrajectoryDiagnostics trajectory_diagnostics{};
            if (!trajectory.source->query_diagnostics(t_curr, trajectory_diagnostics) ||
                !run_logger.log_trajectory_if_due(truth, trajectory_diagnostics)) {
                std::printf("Trajectory diagnostics logging failed at t=%f\n",
                            core::timestamp_seconds(t_curr));
                return 6;
            }
            run_logger.log_imu_if_due(truth, imu_sample);
            run_logger.log_filter_if_due(t_curr, filter);
        }

        if (!navigator_finalized && !navigator.finalize()) {
            std::printf("Navigator finalization failed after trajectory termination\n");
            return 4;
        }

        logger.close();
        export_profile_if_configured<NavKit>(run_settings.data_dir, run_settings.run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n",
                    run_settings.data_dir.string().c_str());
        return 0;
    }

private:
    [[nodiscard]] static bool supply_control_state(const RunSettings& run_settings,
                                                   const core::Timestamp& t_epoch,
                                                   const sim::TruthSample& truth,
                                                   const Filter& filter,
                                                   sim::TrajectorySource& trajectory_source)
    {
        sim::TrajectoryControlState control_state{};
        switch (run_settings.control_state_source) {
        case ControlStateSourceMode::NavigationEstimate:
            if (!trajectory_control_state_from_navigation<StateDef>(
                    truth.t, t_epoch, filter.state(), control_state)) {
                return false;
            }
            break;
        case ControlStateSourceMode::TruthPassthrough:
            if (!trajectory_control_state_from_truth(truth, t_epoch, control_state)) {
                return false;
            }
            break;
        }
        return trajectory_source.set_control_state(control_state);
    }
};

} // namespace navkit::app_support
