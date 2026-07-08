// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingPolicy.hpp"
#include "navkit/app_support/emulation/EmulatorBindingTuplePolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace navkit::app_support
{

template<navkit::api::config::NavKitProductConfigPolicy NavKit,
         typename Logger,
         EmulatorBindingTuplePolicy<typename NavKit::Sensors, Logger> EmulatorBindings>
class EmulatorRuntime
{
public:
    using Navigator = typename NavKit::Navigator;

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

    template<typename EmulatorRuntimes>
    static void process(Navigator& navigator,
                        Logger& logger,
                        EmulatorRuntimes& runtimes,
                        const sim::TruthSample& sample)
    {
        process_impl<EmulatorBindings>(
            navigator,
            logger,
            runtimes,
            sample,
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
        auto& sensor = sensor_for_binding<Binding>(navigator);
        Binding::Emulator_t::configure_sensor(sensor, cfg);
        Binding::Emulator_t::configure_logger(logger, cfg);
    }

    template<typename Binding>
        requires EmulatorBindingPolicy<Binding, Logger>
    static auto& sensor_for_binding(Navigator& navigator)
    {
        constexpr auto SensorIndex =
            navkit::core::estimation::SensorIndexFromId_v<Binding::Id, typename NavKit::Sensors>;
        return navigator.template sensor<SensorIndex>();
    }

    template<typename BindingTuple, typename EmulatorRuntimes, std::size_t... Is>
    static void process_impl(Navigator& navigator,
                             Logger& logger,
                             EmulatorRuntimes& runtimes,
                             const sim::TruthSample& sample,
                             std::index_sequence<Is...>)
    {
        (process_one<std::tuple_element_t<Is, BindingTuple>>(
             navigator, logger, std::get<Is>(runtimes), sample),
         ...);
    }

    template<typename Binding, typename Runtime>
        requires EmulatorBindingPolicy<Binding, Logger>
    static void process_one(Navigator& navigator,
                            Logger& logger,
                            Runtime& runtime,
                            const sim::TruthSample& sample)
    {
        auto& sensor = sensor_for_binding<Binding>(navigator);
        const auto measurement = Binding::Emulator_t::generate(runtime, sample);
        Binding::Emulator_t::log_measurement(logger, measurement);

        if (!sensor.push(measurement)) {
            throw std::runtime_error("Sensor buffer overflow for configured emulator");
        }
    }
};

} // namespace navkit::app_support
