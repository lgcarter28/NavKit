// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/sim/TrajectorySource.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <concepts>
#include <nlohmann/json.hpp>
#include <tuple>
#include <utility>

namespace navkit::app_support
{

namespace detail
{

template<typename Binding>
struct PreparedEmulatorUpdate
{
    bool generated{false};
    sim::TruthSample truth{};
    typename Binding::Sensor_t::Measurement_t measurement{};
};

template<typename BindingTuple>
struct PreparedEmulatorUpdates;

template<typename... Bindings>
struct PreparedEmulatorUpdates<std::tuple<Bindings...>>
{
    using Type = std::tuple<PreparedEmulatorUpdate<Bindings>...>;
};

} // namespace detail

template<navkit::api::config::NavKitProductConfigPolicy NavKit,
         navkit::io::LoggerPolicy Logger,
         EmulatorBindingTuplePolicy<typename NavKit::Sensors, Logger> EmulatorBindings>
class EmulatorRuntime
{
public:
    using Navigator = typename NavKit::Navigator;
    using PreparedUpdates = typename detail::PreparedEmulatorUpdates<EmulatorBindings>::Type;

    static auto make_runtimes(const nlohmann::json& cfg)
    {
        return make_runtimes_impl<EmulatorBindings>(
            cfg, std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    static void configure(Navigator& navigator, Logger& logger, const nlohmann::json& cfg)
    {
        configure_impl<EmulatorBindings>(
            navigator,
            logger,
            cfg,
            std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    /** Prepares all due synthetic emulator updates without modifying Navigator queues. */
    template<typename EmulatorRuntimes>
    [[nodiscard]] static bool prepare(const sim::TrajectorySource& source,
                                      const core::Timestamp& t,
                                      EmulatorRuntimes& runtimes,
                                      PreparedUpdates& prepared_updates)
    {
        sim::TruthSample sample{};
        if (!source.query(t, sample)) {
            return false;
        }
        return prepare_impl<EmulatorBindings>(
            runtimes,
            sample,
            prepared_updates,
            std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    /** Publishes prepared updates to sensors and logging after the planned deadline. */
    template<typename EmulatorRuntimes>
    [[nodiscard]] static bool publish(const PreparedUpdates& prepared_updates,
                                      const EmulatorRuntimes& runtimes,
                                      Navigator& navigator,
                                      Logger& logger)
    {
        return publish_impl<EmulatorBindings>(
            prepared_updates,
            runtimes,
            navigator,
            logger,
            std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

private:
    template<typename BindingTuple, std::size_t... Is>
    static auto make_runtimes_impl(const nlohmann::json& cfg, std::index_sequence<Is...>)
    {
        return std::tuple<typename std::tuple_element_t<Is, BindingTuple>::Emulator_t::Runtime...>{
            std::tuple_element_t<Is, BindingTuple>::Emulator_t::make_runtime(cfg)...};
    }

    template<typename BindingTuple, std::size_t... Is>
    static void configure_impl(Navigator& navigator,
                               Logger& logger,
                               const nlohmann::json& cfg,
                               std::index_sequence<Is...>)
    {
        (configure_one<std::tuple_element_t<Is, BindingTuple>>(navigator, logger, cfg), ...);
    }

    template<typename Binding>
        requires EmulatorBindingPolicy<Binding, Logger>
    static void configure_one(Navigator& navigator, Logger& logger, const nlohmann::json& cfg)
    {
        typename Binding::Sensor_t& sensor = sensor_for_binding<Binding>(navigator);
        Binding::Emulator_t::configure_sensor(sensor, cfg);
        Binding::Emulator_t::configure_logger(logger, cfg);
    }

    template<typename Binding>
        requires EmulatorBindingPolicy<Binding, Logger>
    static typename Binding::Sensor_t& sensor_for_binding(Navigator& navigator)
    {
        constexpr auto SensorIndex =
            navkit::core::estimation::SensorIndexFromId_v<Binding::Id, typename NavKit::Sensors>;
        return navigator.template sensor<SensorIndex>();
    }

    template<typename BindingTuple, typename EmulatorRuntimes, std::size_t... Is>
    [[nodiscard]] static bool prepare_impl(EmulatorRuntimes& runtimes,
                                           const sim::TruthSample& sample,
                                           PreparedUpdates& prepared_updates,
                                           std::index_sequence<Is...>)
    {
        bool success = true;
        ((success = success && prepare_one<std::tuple_element_t<Is, BindingTuple>>(
                                   std::get<Is>(runtimes), sample, std::get<Is>(prepared_updates))),
         ...);
        return success;
    }

    template<typename Binding, typename Runtime>
        requires EmulatorBindingPolicy<Binding, Logger>
    [[nodiscard]] static bool prepare_one(Runtime& runtime,
                                          const sim::TruthSample& sample,
                                          detail::PreparedEmulatorUpdate<Binding>& prepared)
    {
        prepared = {};
        if constexpr (requires {
                          {
                              Binding::Emulator_t::should_generate(runtime, sample)
                          } -> std::same_as<bool>;
                      }) {
            if (!Binding::Emulator_t::should_generate(runtime, sample)) {
                return true;
            }
        }

        prepared.generated = true;
        prepared.truth = sample;
        prepared.measurement = Binding::Emulator_t::generate(runtime, sample);
        return true;
    }

    template<typename BindingTuple, typename EmulatorRuntimes, std::size_t... Is>
    [[nodiscard]] static bool publish_impl(const PreparedUpdates& prepared_updates,
                                           const EmulatorRuntimes& runtimes,
                                           Navigator& navigator,
                                           Logger& logger,
                                           std::index_sequence<Is...>)
    {
        bool success = true;
        ((success = success &&
                    publish_one<std::tuple_element_t<Is, BindingTuple>>(
                        std::get<Is>(prepared_updates), std::get<Is>(runtimes), navigator, logger)),
         ...);
        return success;
    }

    template<typename Binding, typename Runtime>
        requires EmulatorBindingPolicy<Binding, Logger>
    [[nodiscard]] static bool publish_one(const detail::PreparedEmulatorUpdate<Binding>& prepared,
                                          const Runtime& runtime,
                                          Navigator& navigator,
                                          Logger& logger)
    {
        if (!prepared.generated) {
            return true;
        }

        typename Binding::Sensor_t& sensor = sensor_for_binding<Binding>(navigator);
        if constexpr (requires {
                          Binding::Emulator_t::update_sensor_context(
                              runtime, prepared.truth, prepared.measurement, navigator, sensor);
                      }) {
            Binding::Emulator_t::update_sensor_context(
                runtime, prepared.truth, prepared.measurement, navigator, sensor);
        }
        else if constexpr (requires {
                               Binding::Emulator_t::update_sensor_context(
                                   runtime, prepared.truth, prepared.measurement, sensor);
                           }) {
            Binding::Emulator_t::update_sensor_context(
                runtime, prepared.truth, prepared.measurement, sensor);
        }

        if constexpr (requires {
                          Binding::Emulator_t::log_sample(
                              runtime, prepared.truth, prepared.measurement, logger);
                      }) {
            Binding::Emulator_t::log_sample(runtime, prepared.truth, prepared.measurement, logger);
        }
        else {
            Binding::Emulator_t::log_measurement(logger, prepared.measurement);
        }
        return sensor.push(prepared.measurement);
    }
};

} // namespace navkit::app_support
