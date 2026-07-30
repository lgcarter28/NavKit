// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
#include "navkit/app_support/initialization/NavInitializationProviders.hpp"
#include "navkit/app_support/initialization/TransferAlignmentProviders.hpp"
#include "navkit/products/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
#include "navkit/sim/ImuSimulator.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct EcefInsGnssLcGyroAccelBiasDefaultConfig
{
    using NavKit = ::navkit::config::navkit::EcefInsGnssLcGyroAccelBiasDefaultConfig;

    using PrimaryGnssPositionSensor = typename NavKit::PrimaryGnssPositionSensor;
    using PrimaryGnssVelocitySensor = typename NavKit::PrimaryGnssVelocitySensor;
    using PrimaryGnssPositionEmulator =
        ::navkit::app_support::GnssEmulator<PrimaryGnssPositionSensor::Id>;
    using PrimaryGnssVelocityEmulator =
        ::navkit::app_support::GnssVelocityEmulator<PrimaryGnssVelocitySensor::Id>;
    using PrimaryGnssPositionBinding =
        ::navkit::app_support::EmulatorBinding<PrimaryGnssPositionEmulator,
                                               PrimaryGnssPositionSensor>;
    using PrimaryGnssVelocityBinding =
        ::navkit::app_support::EmulatorBinding<PrimaryGnssVelocityEmulator,
                                               PrimaryGnssVelocitySensor>;
    using EmulatorBindings = std::tuple<PrimaryGnssPositionBinding, PrimaryGnssVelocityBinding>;
    using ImuSimulator =
        ::navkit::sim::ImuSimulator<!NavKit::Propagation::apply_coning_sculling_compensation>;
    using NavInitializationProvider = ::navkit::app_support::PvaRuntimeInitializationProvider;
    using TransferAlignmentProvider = ::navkit::app_support::NoTransferAlignmentProvider;

    using App = ::navkit::app_support::SimulationApp<EcefInsGnssLcGyroAccelBiasDefaultConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::EcefInsGnssLcGyroAccelBiasDefaultConfig;

} // namespace navkit::config
