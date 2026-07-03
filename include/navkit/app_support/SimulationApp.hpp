// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/app_support/ConfigTraits.hpp"
#include "navkit/app_support/JsonInput.hpp"
#include "navkit/app_support/ProfileExport.hpp"
#include "navkit/app_support/RuntimeConfigValidation.hpp"
#include "navkit/app_support/SensorId.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/TrajectoryGenerator.hpp"

#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

namespace navkit::app_support
{

namespace detail
{

template<typename BindingTuple, typename SensorTuple>
struct emulator_binding_sensors_valid;

template<typename SensorTuple, typename... Bindings>
struct emulator_binding_sensors_valid<std::tuple<Bindings...>, SensorTuple>
    : std::bool_constant<(
          (navkit::api::config::sensor_type_contains_v<typename Bindings::Sensor_t, SensorTuple> &&
           navkit::api::config::sensor_id_exists_v<Bindings::Id, SensorTuple> &&
           (Bindings::Id == Bindings::Sensor_t::Id)) &&
          ...)>
{};

template<typename BindingTuple, typename SensorTuple>
inline constexpr bool emulator_binding_sensors_valid_v =
    emulator_binding_sensors_valid<BindingTuple, SensorTuple>::value;

} // namespace detail

template<typename Config>
concept SimulationAppConfigPolicy =
    requires {
        typename Config::NavKit;
        typename Config::EmulatorBindings;
    } && navkit::api::config::NavKitProductConfigPolicy<typename Config::NavKit> &&
    emulator_binding_ids_unique_v<typename Config::EmulatorBindings> &&
    detail::emulator_binding_sensors_valid_v<typename Config::EmulatorBindings,
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

    static int run(const std::filesystem::path& config_path)
    {
        const nlohmann::json cfg = load_json_file(config_path);
        validate_runtime_config<Config>(cfg);

        const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
        const std::filesystem::path output_dir =
            cfg.value("output_dir", std::string("data/logs/") + run_name);

        const auto traj_cfg = stationary_trajectory_config_from_json(cfg);
        const auto truth = sim::TrajectoryGenerator::stationary(traj_cfg);
        auto emulator_runtimes = make_emulator_runtimes(cfg);

        reset_profile_sink_if_configured<NavKit>();

        Navigator navigator;
        auto& filter = navigator.filter();
        configure_initial_filter_state(filter, cfg, traj_cfg.p_e);

        io::RunLogger logger(output_dir, run_name, cfg);
        configure_emulators(navigator, logger, cfg);

        for (const auto& sample : truth) {
            logger.log_truth(sample);
            process_emulators(navigator, logger, emulator_runtimes, sample);

            navigator.process_measurements();

            log_measurement_statistics(logger, filter);
            logger.log_nav<StateDef>(sample.time, filter, sample);
        }

        logger.close();
        export_profile_if_configured<NavKit>(output_dir, run_name);

        std::printf("Wrote NavKit simulation logs to: %s\n", output_dir.string().c_str());
        return 0;
    }

private:
    static sim::StationaryTrajectoryConfig
    stationary_trajectory_config_from_json(const nlohmann::json& cfg)
    {
        const auto& trajectory_config = cfg.at("trajectory");
        sim::StationaryTrajectoryConfig traj_cfg;
        traj_cfg.duration_s = trajectory_config.value("duration_s", 60.0);
        traj_cfg.dt_s = trajectory_config.value("dt_s", 1.0);
        traj_cfg.p_e =
            vec3_from_json<Eigen::Matrix<core::Scalar_t, 3, 1>>(trajectory_config.at("p_e_m"));
        return traj_cfg;
    }

    static void configure_initial_filter_state(Filter& filter,
                                               const nlohmann::json& cfg,
                                               const Eigen::Matrix<core::Scalar_t, 3, 1>& p_e)
    {
        typename Filter::State_t initial_state = Filter::State_t::Zero();
        initial_state.template segment<3>(StateDef::Pos::i) = p_e;

        const auto filter_config = cfg.find("filter");
        if (filter_config != cfg.end() && filter_config->contains("initial_position_offset_m")) {
            initial_state.template segment<3>(StateDef::Pos::i) +=
                vec3_from_json<Eigen::Matrix<core::Scalar_t, 3, 1>>(
                    filter_config->at("initial_position_offset_m"));
        }

        filter.set_state(initial_state);

        typename Filter::P_t initial_covariance = Filter::P_t::Identity();
        const core::Scalar_t sigma_p0 =
            cfg.value("filter", nlohmann::json::object()).value("initial_position_sigma_m", 100.0);
        initial_covariance *= 1.0e-6;
        initial_covariance.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) =
            (sigma_p0 * sigma_p0) * Eigen::Matrix<core::Scalar_t, 3, 3>::Identity();
        filter.set_covariance(initial_covariance);
    }

    static auto make_emulator_runtimes(const nlohmann::json& cfg)
    {
        return make_emulator_runtimes_impl(cfg, EmulatorBindings{});
    }

    template<typename... Bindings>
    static auto make_emulator_runtimes_impl(const nlohmann::json& cfg, std::tuple<Bindings...>)
    {
        return std::tuple<typename Bindings::Emulator_t::Runtime...>{
            Bindings::Emulator_t::make_runtime(cfg)...};
    }

    static void
    configure_emulators(Navigator& navigator, io::RunLogger& logger, const nlohmann::json& cfg)
    {
        configure_emulators_impl(navigator, logger, cfg, EmulatorBindings{});
    }

    template<typename... Bindings>
    static void configure_emulators_impl(Navigator& navigator,
                                         io::RunLogger& logger,
                                         const nlohmann::json& cfg,
                                         std::tuple<Bindings...>)
    {
        (configure_emulator<Bindings>(navigator, logger, cfg), ...);
    }

    template<typename Binding>
    static void
    configure_emulator(Navigator& navigator, io::RunLogger& logger, const nlohmann::json& cfg)
    {
        auto& sensor = sensor_for_binding<Binding>(navigator);
        Binding::Emulator_t::configure_sensor(sensor, cfg);
        Binding::Emulator_t::configure_logger(logger, cfg);
    }

    template<typename Binding>
    static auto& sensor_for_binding(Navigator& navigator)
    {
        constexpr auto SensorIndex =
            navkit::api::config::SensorIndexFromId_v<Binding::Id, typename NavKit::Sensors>;
        return navigator.template sensor<SensorIndex>();
    }

    template<typename EmulatorRuntimes>
    static void process_emulators(Navigator& navigator,
                                  io::RunLogger& logger,
                                  EmulatorRuntimes& runtimes,
                                  const sim::TruthSample& sample)
    {
        process_emulators_impl(navigator,
                               logger,
                               runtimes,
                               sample,
                               EmulatorBindings{},
                               std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    template<typename EmulatorRuntimes, typename... Bindings, std::size_t... Is>
    static void process_emulators_impl(Navigator& navigator,
                                       io::RunLogger& logger,
                                       EmulatorRuntimes& runtimes,
                                       const sim::TruthSample& sample,
                                       std::tuple<Bindings...>,
                                       std::index_sequence<Is...>)
    {
        (process_emulator<Bindings>(navigator, logger, std::get<Is>(runtimes), sample), ...);
    }

    template<typename Binding, typename EmulatorRuntime>
    static void process_emulator(Navigator& navigator,
                                 io::RunLogger& logger,
                                 EmulatorRuntime& runtime,
                                 const sim::TruthSample& sample)
    {
        auto& sensor = sensor_for_binding<Binding>(navigator);
        const auto measurement = Binding::Emulator_t::generate(runtime, sample);
        Binding::Emulator_t::log_measurement(logger, measurement);

        if (!sensor.push(measurement)) {
            throw std::runtime_error("Sensor buffer overflow for configured emulator");
        }
    }

    static void log_measurement_statistics(io::RunLogger& logger, const Filter& filter)
    {
        log_measurement_statistics_impl(logger, filter, typename NavKit::MeasurementModels{});
    }

    template<typename... Models>
    static void log_measurement_statistics_impl(io::RunLogger& logger,
                                                const Filter& filter,
                                                std::tuple<Models...>)
    {
        (log_measurement_statistics_for_model<Models>(logger, filter), ...);
    }

    template<typename Model>
    static void log_measurement_statistics_for_model(io::RunLogger& logger, const Filter& filter)
    {
        if constexpr (requires {
                          logger.log_gnss_pos_statistics(
                              filter.template measurement_statistics<Model>());
                      }) {
            if (filter.template has_measurement_statistics<Model>()) {
                logger.log_gnss_pos_statistics(filter.template measurement_statistics<Model>());
            }
        }
    }
};

} // namespace navkit::app_support
