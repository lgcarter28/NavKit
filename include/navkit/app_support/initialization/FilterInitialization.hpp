// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/InitialCovarianceJson.hpp"
#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<typename InitializerConfig, navkit::core::estimation::FilterPolicy Filter>
void initialize_filter_from_nav_initialization(const NavInitialization& nav_init,
                                               const nlohmann::json& cfg,
                                               Filter& filter)
{
    using StateDef = typename InitializerConfig::StateDef;
    using Nominal = typename StateDef::Nominal;

    typename Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state.template segment<3>(Nominal::Pos::i) = core::estimation::pos_e_m(nav_init.pva);
    initial_state.template segment<3>(Nominal::Vel::i) = core::estimation::vel_e_mps(nav_init.pva);
    const auto q_b2e =
        core::math::quaternion_from_rpy_rad(core::estimation::rpy_b2e_rad(nav_init.pva));
    initial_state.template segment<4>(Nominal::AttQuat::i) << q_b2e.w(), q_b2e.x(), q_b2e.y(),
        q_b2e.z();

    filter.set_state(initial_state);

    const typename Filter::P_t initial_covariance =
        detail::initial_covariance_from_json<StateDef>(cfg, InitializerConfig::initial_covariance);
    filter.set_covariance(initial_covariance);
}

template<typename InitializerConfig, typename Navigator>
void initialize_navigator(const NavInitialization& nav_init,
                          const nlohmann::json& cfg,
                          Navigator& navigator)
{
    initialize_filter_from_nav_initialization<InitializerConfig>(nav_init, cfg, navigator.filter());
}

} // namespace navkit::app_support
