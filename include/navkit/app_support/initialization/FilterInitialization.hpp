// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <Eigen/Dense>

namespace navkit::app_support
{

namespace detail
{

template<typename TargetRow, typename TargetCol, typename SourceRow, typename SourceCol, typename P>
void copy_pva_covariance_block(P& target_covariance,
                               const core::estimation::PvaCovariance& pva_covariance)
{
    target_covariance.template block<TargetRow::sz, TargetCol::sz>(TargetRow::i, TargetCol::i) =
        pva_covariance.template block<SourceRow::sz, SourceCol::sz>(SourceRow::i, SourceCol::i);
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename P>
void copy_pva_covariance(P& target_covariance,
                         const core::estimation::PvaCovariance& pva_covariance)
{
    using Error = typename StateDef::Error;

    copy_pva_covariance_block<typename Error::Pos,
                              typename Error::Pos,
                              core::estimation::PvaStateDef::Pos,
                              core::estimation::PvaStateDef::Pos>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::Pos,
                              typename Error::Vel,
                              core::estimation::PvaStateDef::Pos,
                              core::estimation::PvaStateDef::Vel>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::Pos,
                              typename Error::AttRotVec,
                              core::estimation::PvaStateDef::Pos,
                              core::estimation::PvaStateDef::Rpy>(target_covariance,
                                                                  pva_covariance);

    copy_pva_covariance_block<typename Error::Vel,
                              typename Error::Pos,
                              core::estimation::PvaStateDef::Vel,
                              core::estimation::PvaStateDef::Pos>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::Vel,
                              typename Error::Vel,
                              core::estimation::PvaStateDef::Vel,
                              core::estimation::PvaStateDef::Vel>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::Vel,
                              typename Error::AttRotVec,
                              core::estimation::PvaStateDef::Vel,
                              core::estimation::PvaStateDef::Rpy>(target_covariance,
                                                                  pva_covariance);

    copy_pva_covariance_block<typename Error::AttRotVec,
                              typename Error::Pos,
                              core::estimation::PvaStateDef::Rpy,
                              core::estimation::PvaStateDef::Pos>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::AttRotVec,
                              typename Error::Vel,
                              core::estimation::PvaStateDef::Rpy,
                              core::estimation::PvaStateDef::Vel>(target_covariance,
                                                                  pva_covariance);
    copy_pva_covariance_block<typename Error::AttRotVec,
                              typename Error::AttRotVec,
                              core::estimation::PvaStateDef::Rpy,
                              core::estimation::PvaStateDef::Rpy>(target_covariance,
                                                                  pva_covariance);
}

} // namespace detail

template<navkit::core::estimation::StateSpaceDefPolicy StateDef,
         navkit::core::estimation::FilterPolicy Filter>
void initialize_filter_from_nav_initialization(Filter& filter, const NavInitialization& nav_init)
{
    using Nominal = typename StateDef::Nominal;

    typename Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state.template segment<3>(Nominal::Pos::i) = core::estimation::pos_e_m(nav_init.pva);
    initial_state.template segment<3>(Nominal::Vel::i) = core::estimation::vel_e_mps(nav_init.pva);
    const auto q_b2e =
        core::math::quaternion_from_rpy_rad(core::estimation::rpy_b2e_rad(nav_init.pva));
    initial_state.template segment<4>(Nominal::AttQuat::i) << q_b2e.w(), q_b2e.x(), q_b2e.y(),
        q_b2e.z();

    filter.set_state(initial_state);

    typename Filter::P_t initial_covariance = Filter::P_t::Identity();
    initial_covariance *= 1.0e-6;
    detail::copy_pva_covariance<StateDef>(initial_covariance, nav_init.pva_cov);
    filter.set_covariance(initial_covariance);
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename Navigator>
void initialize_navigator(Navigator& navigator, const NavInitialization& nav_init)
{
    initialize_filter_from_nav_initialization<StateDef>(navigator.filter(), nav_init);
}

} // namespace navkit::app_support
