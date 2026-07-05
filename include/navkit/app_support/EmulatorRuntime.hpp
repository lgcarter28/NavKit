// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <nlohmann/json.hpp>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace navkit::app_support
{

template<typename NavKit, typename EmulatorBindings>
class EmulatorRuntime
{
public:
    using Navigator = typename NavKit::Navigator;

    static auto make_runtimes(const nlohmann::json& cfg)
    {
        return make_runtimes_impl<EmulatorBindings>(
            cfg, std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    static void configure(Navigator& navigator, io::RunLogger& logger, const nlohmann::json& cfg)
    {
        configure_impl<EmulatorBindings>(
            navigator,
            logger,
            cfg,
            std::make_index_sequence<std::tuple_size_v<EmulatorBindings>>{});
    }

    template<typename EmulatorRuntimes>
    static void process(Navigator& navigator,
                        io::RunLogger& logger,
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
                               io::RunLogger& logger,
                               const nlohmann::json& cfg,
                               std::index_sequence<Is...>)
    {
        (configure_one<std::tuple_element_t<Is, BindingTuple>>(navigator, logger, cfg), ...);
    }

    template<typename Binding>
    static void
    configure_one(Navigator& navigator, io::RunLogger& logger, const nlohmann::json& cfg)
    {
        auto& sensor = sensor_for_binding<Binding>(navigator);
        Binding::Emulator_t::configure_sensor(sensor, cfg);
        Binding::Emulator_t::configure_logger(logger, cfg);
    }

    template<typename Binding>
    static auto& sensor_for_binding(Navigator& navigator)
    {
        constexpr auto SensorIndex =
            navkit::core::estimation::SensorIndexFromId_v<Binding::Id, typename NavKit::Sensors>;
        return navigator.template sensor<SensorIndex>();
    }

    template<typename BindingTuple, typename EmulatorRuntimes, std::size_t... Is>
    static void process_impl(Navigator& navigator,
                             io::RunLogger& logger,
                             EmulatorRuntimes& runtimes,
                             const sim::TruthSample& sample,
                             std::index_sequence<Is...>)
    {
        (process_one<std::tuple_element_t<Is, BindingTuple>>(
             navigator, logger, std::get<Is>(runtimes), sample),
         ...);
    }

    template<typename Binding, typename Runtime>
    static void process_one(Navigator& navigator,
                            io::RunLogger& logger,
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
