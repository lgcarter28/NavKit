// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <tuple>
#include <type_traits>

namespace navkit::app_support
{

/// Full nominal truth-state reference used only by simulation/analysis startup paths.
///
/// The reusable Navigator receives only the resulting nominal state. This app-support
/// boundary keeps simulator truth and realized sensor calibration terms outside NavKit core.
template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
struct InitialTruthReference
{
    core::Time_t time_s{0.0};
    navkit::core::estimation::NominalState<StateDef> truth_state{
        navkit::core::estimation::NominalState<StateDef>::Zero()};
};

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void populate_initial_pva_from_truth(const navkit::sim::TruthSample& truth,
                                            InitialTruthReference<StateDef>& reference)
{
    using Nominal = typename StateDef::Nominal;

    reference.time_s = truth.time;
    navkit::core::estimation::segment<typename Nominal::Pos>(reference.truth_state) = truth.p_e;
    navkit::core::estimation::segment<typename Nominal::Vel>(reference.truth_state) = truth.v_e;
    if constexpr (requires { typename Nominal::AttQuat; }) {
        navkit::core::estimation::segment<typename Nominal::AttQuat>(reference.truth_state)
            << truth.q_b2e.w(),
            truth.q_b2e.x(), truth.q_b2e.y(), truth.q_b2e.z();
    }
}

namespace detail
{

template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename Runtime>
inline void apply_initial_truth_reference_contribution(const Runtime& runtime,
                                                       InitialTruthReference<StateDef>& reference)
{
    if constexpr (requires {
                      runtime.template apply_initial_truth_reference<StateDef>(reference);
                  }) {
        runtime.template apply_initial_truth_reference<StateDef>(reference);
    }
    else if constexpr (requires { std::tuple_size<std::remove_cvref_t<Runtime>>::value; }) {
        std::apply(
            [&reference](const auto&... nested_runtimes) {
                (apply_initial_truth_reference_contribution<StateDef>(nested_runtimes, reference),
                 ...);
            },
            runtime);
    }
}

} // namespace detail

/// Apply configured runtime truth-state contributions to an initial truth reference.
///
/// A runtime contributes only the nominal-state terms it owns, such as realized IMU
/// bias truth. Nested runtime tuples are intentionally traversed so SimulationApp does
/// not need sensor-specific initialization knowledge.
template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename RuntimeTuple>
inline void apply_initial_truth_reference_from_runtimes(const RuntimeTuple& runtimes,
                                                        InitialTruthReference<StateDef>& reference)
{
    detail::apply_initial_truth_reference_contribution<StateDef>(runtimes, reference);
}

} // namespace navkit::app_support
