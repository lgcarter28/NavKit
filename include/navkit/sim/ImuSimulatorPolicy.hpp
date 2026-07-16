// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/sim/ImuSimulator.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <concepts>

namespace navkit::sim
{

template<typename Candidate>
concept ImuSimulatorPolicy = std::constructible_from<Candidate, ImuSimulatorConfig> &&
                             requires(Candidate& simulator,
                                      const TruthSample& truth,
                                      navkit::core::estimation::ImuIncrement& increment,
                                      ImuInterval& interval,
                                      ImuIntervalDebug& debug) {
                                 {
                                     Candidate::output_coning_sculling_compensated_v
                                 } -> std::convertible_to<bool>;
                                 { simulator.initialize(truth) } -> std::same_as<void>;
                                 { simulator.generate(truth, increment) } -> std::same_as<bool>;
                                 {
                                     simulator.generate(truth, increment, interval)
                                 } -> std::same_as<bool>;
                                 {
                                     simulator.generate(truth, increment, interval, debug)
                                 } -> std::same_as<bool>;
                             };

} // namespace navkit::sim
