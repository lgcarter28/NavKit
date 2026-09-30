// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/config/MissionAppConfigPolicy.hpp"
#include "navkit/app_support/execution/ExecutionTargetJson.hpp"
#include "navkit/app_support/execution/MissionAdapter.hpp"
#include "navkit/app_support/logging/RuntimeLogger.hpp"
#include "navkit/app_support/mission/MissionRuntimeJson.hpp"
#include "navkit/app_support/profiling/ProfileExport.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/MissionRuntimeConfigValidation.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/time/ClockFactory.hpp"
#include "navkit/core/time/RationalTimeline.hpp"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

namespace navkit::app_support
{

/**
 * \brief Target-neutral planned-time host loop for one selected NavKit product.
 *
 * \details The application owns the Clock, legal mission-phase transitions, and Navigator
 * update order. A fixed-target adapter factory selected by the thin executable owns
 * target-specific validation and constructs the runtime adapter responsible for initialization,
 * data preparation, publication, feedback, and logging. Runtime input must name that same target.
 */
template<MissionAppConfigPolicy Config>
class MissionApp
{
public:
    using NavKit = NavKitConfig_t<Config>;
    using Navigator = typename NavKit::Navigator;
    using Logger = RuntimeLogger<NavKit>;
    using Adapter = MissionAdapter<Navigator, Logger>;
    template<typename AdapterFactory>
    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        const ExecutionTargetSettings execution_target = execution_target_settings_from_json(cfg);
        require_execution_target_type(
            execution_target, AdapterFactory::target_type, AdapterFactory::application_name);
        validate_common_mission_runtime_config<Config>(cfg);
        AdapterFactory::template validate_runtime_config<Config>(cfg);

        const RunSettings run_settings = run_settings_from_json(cfg);
        MissionRuntime mission_runtime = mission_runtime_from_json(cfg);

        reset_profile_sink_if_configured<NavKit>();

        std::unique_ptr<Navigator> navigator_storage = std::make_unique<Navigator>();
        Navigator& navigator = *navigator_storage;
        Logger logger(run_settings.data_dir, run_settings.run_name, cfg);
        std::unique_ptr<Adapter> adapter =
            AdapterFactory::template create<Config>(cfg, config_path.parent_path(), run_settings);

        bool adapter_finalization_attempted = false;
        bool logger_closed = false;
        const auto cleanup = [&]() noexcept {
            if (adapter && !adapter_finalization_attempted) {
                adapter_finalization_attempted = true;
                static_cast<void>(adapter->finalize());
            }
            if (!logger_closed) {
                try {
                    logger.close();
                    logger_closed = true;
                }
                catch (...) {
                    // Cleanup is best effort; preserve the original mission failure code.
                }
            }
        };
        const auto error_result = [&](const int code) {
            cleanup();
            return code;
        };

        if (!adapter || !adapter->initialize(navigator, logger)) {
            const std::string error =
                adapter ? std::string{adapter->last_error()} : "adapter was not created";
            std::printf("Mission adapter initialization failed: %s\n", error.c_str());
            return error_result(3);
        }

        std::string initial_phase_id{};
        if (!adapter->active_mission_phase_id(initial_phase_id) ||
            !mission_runtime.template initialize<NavKit>(initial_phase_id, navigator)) {
            std::printf("Initial mission phase did not match the configured mission graph\n");
            return error_result(4);
        }

        const core::Timestamp t_start = adapter->t_start();
        std::unique_ptr<Clock> clock = clock_from_mode(execution_target.clock_mode);
        if (!clock || !clock->initialize(t_start)) {
            std::printf("Mission clock initialization failed\n");
            return error_result(5);
        }

        core::RationalTimeline app_timeline{};
        if (!app_timeline.initialize(t_start, execution_target.application_rate)) {
            std::printf("Application timeline initialization failed\n");
            return error_result(5);
        }

        bool navigator_finalized = false;
        int result = execute_epoch(
            t_start, *adapter, mission_runtime, *clock, navigator, logger, navigator_finalized);
        if (result != 0) {
            return error_result(result);
        }

        while (!adapter->is_complete()) {
            core::Timestamp t_curr{};
            if (!app_timeline.next(t_curr)) {
                std::printf("Application timeline overflow\n");
                return error_result(7);
            }
            result = execute_epoch(
                t_curr, *adapter, mission_runtime, *clock, navigator, logger, navigator_finalized);
            if (result != 0) {
                return error_result(result);
            }
        }

        if (!navigator_finalized && !navigator.finalize()) {
            std::printf("Navigator finalization failed after mission termination\n");
            return error_result(4);
        }
        adapter_finalization_attempted = true;
        if (!adapter->finalize()) {
            const std::string error{adapter->last_error()};
            std::printf("Mission adapter finalization failed: %s\n", error.c_str());
            return error_result(4);
        }

        logger.close();
        logger_closed = true;
        export_profile_if_configured<NavKit>(run_settings.data_dir, run_settings.run_name);

        std::printf("Wrote NavKit mission logs to: %s\n", run_settings.data_dir.string().c_str());
        return 0;
    }

private:
    [[nodiscard]] static int execute_epoch(const core::Timestamp& t,
                                           Adapter& adapter,
                                           MissionRuntime& mission_runtime,
                                           Clock& clock,
                                           Navigator& navigator,
                                           Logger& logger,
                                           bool& navigator_finalized)
    {
        core::Timestamp prepared_t{};
        if (!adapter.prepare_before_deadline(t, prepared_t)) {
            const std::string error{adapter.last_error()};
            std::printf("Mission data preparation failed at t=%f: %s\n",
                        core::timestamp_seconds(t),
                        error.c_str());
            return 2;
        }

        if (!clock.wait_until(prepared_t)) {
            std::printf("Mission clock failed to reach t=%f\n",
                        core::timestamp_seconds(prepared_t));
            return 4;
        }

        std::string reported_phase_id{};
        if (!adapter.active_mission_phase_id(reported_phase_id) ||
            !mission_runtime.template synchronize<NavKit>(reported_phase_id, navigator)) {
            std::printf("Mission phase synchronization failed at t=%f\n",
                        core::timestamp_seconds(prepared_t));
            return 4;
        }
        if (!adapter.publish_at_deadline(prepared_t, navigator, logger)) {
            const std::string error{adapter.last_error()};
            std::printf("Mission data publication failed at t=%f: %s\n",
                        core::timestamp_seconds(prepared_t),
                        error.c_str());
            return 4;
        }

        const bool mission_complete = adapter.is_complete();
        if (mission_complete && !mission_runtime.active_phase_is_terminal()) {
            std::printf("Mission source completed in a nonterminal phase at t=%f\n",
                        core::timestamp_seconds(prepared_t));
            return 4;
        }
        const bool navigator_updated = mission_complete ? navigator.finalize() : navigator.update();
        if (!navigator_updated) {
            std::printf("Navigator update failed at t=%f\n", core::timestamp_seconds(prepared_t));
            return 4;
        }
        navigator_finalized = mission_complete;

        if (!adapter.after_navigation_update(prepared_t, navigator)) {
            const std::string error{adapter.last_error()};
            std::printf("Mission post-navigation processing failed at t=%f: %s\n",
                        core::timestamp_seconds(prepared_t),
                        error.c_str());
            return 4;
        }
        return 0;
    }
};

} // namespace navkit::app_support
