// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/config/ConfigTraits.hpp"
#include "navkit/app_support/emulation/EmulatorRuntime.hpp"
#include "navkit/app_support/emulation/concrete/ImuRuntime.hpp"
#include "navkit/app_support/execution/MissionAdapter.hpp"
#include "navkit/app_support/initialization/FilterInitialization.hpp"
#include "navkit/app_support/initialization/InitialTruthReference.hpp"
#include "navkit/app_support/logging/SimulationRunLogger.hpp"
#include "navkit/app_support/runtime/RunSettings.hpp"
#include "navkit/app_support/trajectory/TrajectoryControlState.hpp"
#include "navkit/swil/SwilAppConfigPolicy.hpp"
#include "navkit/swil/SwilRuntime.hpp"

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace navkit::swil
{

/**
 * \brief Concrete software-in-the-loop implementation of the mission execution boundary.
 *
 * \details This adapter exclusively owns synthetic truth, IMU and aiding emulators, prepared
 * updates, truth-based initialization, SWIL feedback, and simulation-only logging. The host
 * application owns the Clock, legal mission-phase transitions, and Navigator update cadence.
 * No simulation truth types cross the target-neutral `MissionAdapter` interface.
 */
template<SwilAppConfigPolicy Config>
class SwilMissionAdapter final
    : public app_support::MissionAdapter<
          typename app_support::NavKitConfig_t<Config>::Navigator,
          app_support::RuntimeLogger<app_support::NavKitConfig_t<Config>>>
{
public:
    using NavKit = app_support::NavKitConfig_t<Config>;
    using StateDef = typename NavKit::StateDef;
    using Filter = typename NavKit::Filter;
    using Navigator = typename NavKit::Navigator;
    using Logger = app_support::RuntimeLogger<NavKit>;
    using EmulatorBindings = typename Config::EmulatorBindings;
    using NavInitializationProvider = typename Config::NavInitializationProvider;
    using TransferAlignmentProvider = typename Config::TransferAlignmentProvider;
    using ImuSimulator = typename Config::ImuSimulator;
    using Emulators = app_support::EmulatorRuntime<NavKit, Logger, EmulatorBindings>;
    using Imu = app_support::ImuRuntime<ImuSimulator>;
    using EmulatorRuntimes =
        decltype(Emulators::make_runtimes(std::declval<const nlohmann::json&>()));

    SwilMissionAdapter(const nlohmann::json& cfg,
                       const std::filesystem::path& source_base_dir,
                       app_support::RunSettings run_settings)
        : m_cfg(cfg)
        , m_run_settings{std::move(run_settings)}
        , m_runtime{swil_runtime_from_json(cfg, source_base_dir)}
        , m_mission_phase_ids{mission_phase_ids_from_json(cfg)}
        , m_emulator_runtimes{Emulators::make_runtimes(cfg)}
        , m_imu{cfg}
    {}

    [[nodiscard]] core::Timestamp t_start() const override
    {
        return m_runtime.source ? m_runtime.source->t_start() : core::Timestamp{};
    }

    [[nodiscard]] bool is_complete() const override
    {
        return !m_runtime.source || m_completion_published;
    }

    [[nodiscard]] bool initialize(Navigator& navigator, Logger& logger) override
    {
        if (m_initialized || !m_runtime.source) {
            return fail("SWIL adapter is already initialized or has no trajectory source");
        }
        if (!m_cfg.contains("mission")) {
            return fail("SWIL adapter did not retain the resolved runtime configuration");
        }

        const core::Timestamp start = m_runtime.source->t_start();
        if (!m_runtime.source->advance_to(start) || !m_runtime.source->query(start, m_truth) ||
            !m_runtime.source->query_diagnostics(start, m_diagnostics)) {
            return fail("SWIL trajectory source did not provide its initial epoch");
        }
        m_diagnostics_valid = true;

        if (!m_imu.initialize(m_truth)) {
            return fail(std::string{"IMU runtime initialization failed: "} +
                        std::string{m_imu.last_error()});
        }

        const app_support::PvaInitialization pva_initialization =
            NavInitializationProvider::initialize(m_cfg, m_runtime.initial_truth);
        app_support::InitialTruthReference<StateDef> truth_reference{};
        app_support::populate_initial_pva_from_truth<StateDef>(m_runtime.initial_truth,
                                                               truth_reference);
        app_support::apply_initial_truth_reference_from_runtimes<StateDef>(
            std::tie(m_imu, m_emulator_runtimes), truth_reference);
        app_support::initialize_navigator<NavKit>(
            pva_initialization, m_cfg, truth_reference, navigator);
        TransferAlignmentProvider::template transfer_align<Navigator>(navigator, m_cfg);

        Emulators::configure(navigator, logger, m_cfg);
        m_run_logger =
            std::make_unique<app_support::SimulationRunLogger<Config>>(logger, m_run_settings);
        if (!m_run_logger->initialize(start)) {
            return fail("SWIL run logger initialization failed");
        }

        m_initialized = true;
        m_initial_epoch = true;
        return true;
    }

    [[nodiscard]] bool prepare_before_deadline(const core::Timestamp& requested_t,
                                               core::Timestamp& prepared_t) override
    {
        prepared_t = {};
        if (!m_initialized || m_prepared || m_published || m_completion_published ||
            !m_runtime.source) {
            return fail("SWIL prepare called outside the valid lifecycle");
        }

        core::Timestamp source_t = requested_t;
        if (core::timestamp_less(m_runtime.source->t_end(), source_t)) {
            source_t = m_runtime.source->t_end();
        }
        if (source_t != m_runtime.source->t_start() && !m_runtime.source->advance_to(source_t)) {
            return fail("SWIL trajectory source could not advance to the planned timestamp");
        }

        // A generated source may discover an early terminal event while advancing. Publish that
        // final valid sample instead of turning normal source completion into an application
        // error at the later requested epoch.
        if (m_runtime.source->is_complete() &&
            core::timestamp_less(m_runtime.source->t_end(), source_t)) {
            source_t = m_runtime.source->t_end();
        }
        if (!m_runtime.source->query(source_t, m_truth) ||
            !m_runtime.source->query_diagnostics(source_t, m_diagnostics)) {
            return fail("SWIL trajectory source could not provide the planned epoch");
        }
        m_diagnostics_valid = true;

        m_imu_sample = {};
        m_prepared_updates = {};
        if (!m_initial_epoch && !m_imu.prepare(*m_runtime.source, source_t, m_imu_sample)) {
            return fail(std::string{"SWIL IMU preparation failed: "} +
                        std::string{m_imu.last_error()});
        }
        if (!Emulators::prepare(
                *m_runtime.source, source_t, m_emulator_runtimes, m_prepared_updates)) {
            return fail("SWIL aiding-emulator preparation failed");
        }

        m_prepared_time = source_t;
        m_prepared_is_terminal =
            m_runtime.source->is_complete() && source_t == m_runtime.source->t_end();
        m_prepared = true;
        prepared_t = source_t;
        return true;
    }

    [[nodiscard]] bool
    publish_at_deadline(const core::Timestamp& t, Navigator& navigator, Logger& logger) override
    {
        if (!m_prepared || m_published || t != m_prepared_time) {
            return fail("SWIL publish did not match the prepared epoch");
        }

        if (!m_initial_epoch && !m_imu.publish(m_imu_sample, navigator)) {
            return fail(std::string{"SWIL IMU publication failed: "} +
                        std::string{m_imu.last_error()});
        }
        if (!m_initial_epoch && m_imu_sample.generated &&
            !m_runtime.source->observe_imu_increment(m_imu_sample.measured)) {
            return fail("SWIL trajectory source rejected the IMU observation");
        }
        if (!Emulators::publish(m_prepared_updates, m_emulator_runtimes, navigator, logger)) {
            return fail("SWIL aiding-emulator publication failed");
        }

        m_published = true;
        m_completion_published = m_prepared_is_terminal;
        return true;
    }

    [[nodiscard]] bool after_navigation_update(const core::Timestamp& t,
                                               const Navigator& navigator) override
    {
        if (!m_prepared || !m_published || t != m_prepared_time || !m_run_logger) {
            return fail("SWIL post-navigation feedback called outside the valid lifecycle");
        }
        if (!publish_trajectory_control_state_after_navigation_update(
                m_runtime.control_state_source,
                t,
                m_truth,
                navigator.filter(),
                *m_runtime.source)) {
            return fail("SWIL trajectory source rejected post-navigation control feedback");
        }

        m_run_logger->log_truth_if_due(m_truth);
        if (!m_run_logger->log_trajectory_if_due(m_truth, m_diagnostics)) {
            return fail("SWIL trajectory diagnostics logging failed");
        }
        if (!m_initial_epoch) {
            m_run_logger->log_imu_if_due(m_truth, m_imu_sample);
        }
        m_run_logger->log_filter_if_due(t, navigator);

        m_initial_epoch = false;
        m_prepared = false;
        m_published = false;
        return true;
    }

    [[nodiscard]] bool active_mission_phase_id(std::string& phase_id) const override
    {
        if (!m_diagnostics_valid ||
            m_diagnostics.mission_phase_index >= m_mission_phase_ids.size()) {
            return false;
        }
        phase_id = m_mission_phase_ids.at(m_diagnostics.mission_phase_index);
        return true;
    }

    [[nodiscard]] bool finalize() override
    {
        if (!m_initialized || m_finalized) {
            return fail("SWIL adapter finalization called outside the valid lifecycle");
        }
        m_prepared = false;
        m_published = false;
        m_finalized = true;
        return true;
    }

    [[nodiscard]] std::string_view last_error() const override
    {
        return m_last_error;
    }

private:
    [[nodiscard]] static std::vector<std::string>
    mission_phase_ids_from_json(const nlohmann::json& cfg)
    {
        std::vector<std::string> phase_ids{};
        if (!cfg.contains("mission") || !cfg.at("mission").is_object() ||
            !cfg.at("mission").contains("phases") || !cfg.at("mission").at("phases").is_array()) {
            return phase_ids;
        }
        const nlohmann::json& phases = cfg.at("mission").at("phases");
        phase_ids.reserve(phases.size());
        for (const nlohmann::json& phase : phases) {
            if (!phase.is_object() || !phase.contains("id") || !phase.at("id").is_string()) {
                phase_ids.clear();
                return phase_ids;
            }
            phase_ids.push_back(phase.at("id").get<std::string>());
        }
        return phase_ids;
    }

    [[nodiscard]] static bool publish_trajectory_control_state_after_navigation_update(
        const app_support::ControlStateSourceMode source_mode,
        const core::Timestamp& t,
        const sim::TruthSample& truth,
        const Filter& filter,
        sim::TrajectorySource& trajectory_source)
    {
        sim::TrajectoryControlState control_state{};
        switch (source_mode) {
        case app_support::ControlStateSourceMode::NavigationEstimate:
            if (!app_support::trajectory_control_state_from_navigation<StateDef>(
                    truth.t, t, filter.state(), control_state)) {
                return false;
            }
            break;
        case app_support::ControlStateSourceMode::TruthPassthrough:
            if (!app_support::trajectory_control_state_from_truth(truth, t, control_state)) {
                return false;
            }
            break;
        }
        return trajectory_source.set_control_state(control_state);
    }

    [[nodiscard]] bool fail(std::string message)
    {
        m_last_error = std::move(message);
        return false;
    }

    nlohmann::json m_cfg{};
    app_support::RunSettings m_run_settings{};
    SwilRuntime m_runtime{};
    std::vector<std::string> m_mission_phase_ids{};
    EmulatorRuntimes m_emulator_runtimes;
    Imu m_imu;
    typename Emulators::PreparedUpdates m_prepared_updates{};
    app_support::ImuRuntimeSample m_imu_sample{};
    sim::TruthSample m_truth{};
    sim::TrajectoryDiagnostics m_diagnostics{};
    std::unique_ptr<app_support::SimulationRunLogger<Config>> m_run_logger{};
    core::Timestamp m_prepared_time{};
    std::string m_last_error{};
    bool m_initialized{false};
    bool m_initial_epoch{false};
    bool m_diagnostics_valid{false};
    bool m_prepared{false};
    bool m_prepared_is_terminal{false};
    bool m_published{false};
    bool m_completion_published{false};
    bool m_finalized{false};
};

} // namespace navkit::swil
