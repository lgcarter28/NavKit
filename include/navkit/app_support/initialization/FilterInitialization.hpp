// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/CovarianceFloorJson.hpp"
#include "navkit/app_support/initialization/InitialCovarianceJson.hpp"
#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/NominalStateOverrideJson.hpp"
#include "navkit/app_support/runtime/PropagationRuntimeConfigJson.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<typename InitializerConfig, navkit::core::estimation::FilterPolicy Filter>
void initialize_filter_from_nav_initialization(
    const NavInitialization<typename InitializerConfig::StateDef>& nav_init,
    const typename Filter::P_t& covariance_floor,
    Filter& filter)
{
    using StateDef = typename InitializerConfig::StateDef;
    using Nominal = typename StateDef::Nominal;

    typename Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state.template segment<3>(Nominal::Pos::i) = core::estimation::pos_e_m(nav_init.pva);
    initial_state.template segment<3>(Nominal::Vel::i) = core::estimation::vel_e_mps(nav_init.pva);
    const Eigen::Quaternion<core::Scalar_t> q_b2e =
        core::math::quaternion_from_rpy_rad(core::estimation::rpy_b2e_rad(nav_init.pva));
    initial_state.template segment<4>(Nominal::AttQuat::i) << q_b2e.w(), q_b2e.x(), q_b2e.y(),
        q_b2e.z();

    filter.set_state(initial_state);

    filter.set_covariance(nav_init.covariance);
    filter.set_covariance_floor(covariance_floor);
}

template<typename InitializerConfig, typename Propagation>
void initialize_propagator(const nlohmann::json& cfg, Propagation& propagation)
{
    propagation.set_runtime_config(detail::propagation_runtime_config_from_json<Propagation>(
        cfg, InitializerConfig::Propagation::runtime_config));
}

template<typename InitializerConfig, typename Navigator>
void initialize_navigator(const PvaInitialization& pva_init,
                          const nlohmann::json& cfg,
                          Navigator& navigator)
{
    using StateDef = typename InitializerConfig::StateDef;
    const typename Navigator::Filter_t::P_t initial_covariance =
        detail::initial_covariance_from_json<StateDef>(
            cfg,
            InitializerConfig::InitialCovariance::initial_covariance,
            core::estimation::pos_e_m(pva_init.pva));
    const typename Navigator::Filter_t::P_t covariance_floor =
        detail::covariance_floor_from_json<StateDef>(
            cfg,
            InitializerConfig::CovarianceFloor::covariance_floor,
            core::estimation::pos_e_m(pva_init.pva));
    const NavInitialization<StateDef> nav_init{
        .time_s = pva_init.time_s,
        .pva = pva_init.pva,
        .covariance = initial_covariance,
    };
    initialize_filter_from_nav_initialization<InitializerConfig>(
        nav_init, covariance_floor, navigator.filter());
    detail::apply_runtime_nominal_state_override<StateDef>(cfg, navigator.filter().state());
    initialize_propagator<InitializerConfig>(cfg, navigator.propagation());
}

} // namespace navkit::app_support
