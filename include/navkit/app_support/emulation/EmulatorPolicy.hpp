// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <concepts>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::app_support
{

template<typename Candidate, typename Sensor, typename Logger>
concept EmulatorPolicy = requires(typename Candidate::Runtime& runtime,
                                  Sensor& sensor,
                                  Logger& logger,
                                  const nlohmann::json& cfg,
                                  const sim::TruthSample& truth,
                                  const typename Sensor::Measurement_t& measurement) {
    { Candidate::Id } -> std::convertible_to<decltype(Sensor::Id)>;
    { Candidate::RuntimeKey } -> std::convertible_to<std::string_view>;
    typename Candidate::Runtime;
    typename Candidate::RuntimeConfig;

    { Candidate::validate_runtime_config(cfg) } -> std::same_as<void>;
    { Candidate::make_runtime(cfg) } -> std::same_as<typename Candidate::Runtime>;
    { Candidate::configure_sensor(sensor, cfg) } -> std::same_as<void>;
    { Candidate::configure_logger(logger, cfg) } -> std::same_as<void>;
    { Candidate::generate(runtime, truth) } -> std::same_as<typename Sensor::Measurement_t>;
    { Candidate::log_measurement(logger, measurement) } -> std::same_as<void>;
} && navkit::io::LoggerPolicy<Logger>;

} // namespace navkit::app_support
