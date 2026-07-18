// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
#include "navkit/app_support/initialization/NavInitializationProviders.hpp"
#include "navkit/app_support/initialization/TransferAlignmentProviders.hpp"
#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/io/log_products/FilterCorrectionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionDebugLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionUpdateLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityDebugLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityLogProduct.hpp"
#include "navkit/io/log_products/GnssVelocityUpdateLogProduct.hpp"
#include "navkit/io/log_products/ImuDebugLogProduct.hpp"
#include "navkit/io/log_products/ImuIncrementLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/products/EcefInsGnss.hpp"
#include "navkit/sim/ImuSimulator.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct EcefInsGnssConfig
{
    using NavKit = ::navkit::config::navkit::EcefInsGnssConfig;

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
    using PrimaryGnssPositionStatistics =
        ::navkit::core::estimation::MeasurementStatistics<PrimaryGnssPositionSensor>;
    using PrimaryGnssVelocityStatistics =
        ::navkit::core::estimation::MeasurementStatistics<PrimaryGnssVelocitySensor>;
    using Filter = typename NavKit::Filter;
    using StateDef = typename NavKit::StateDef;

    using EmulatorBindings = std::tuple<PrimaryGnssPositionBinding, PrimaryGnssVelocityBinding>;
    using ImuSimulator =
        ::navkit::sim::ImuSimulator<!NavKit::Propagation::apply_coning_sculling_compensation>;

    using NavInitializationProvider = ::navkit::app_support::PvaRuntimeInitializationProvider;
    using TransferAlignmentProvider = ::navkit::app_support::NoTransferAlignmentProvider;

    using Logger = ::navkit::io::RunLogger<
        ::navkit::io::TruthLogProduct,
        ::navkit::io::GnssPositionLogProduct,
        ::navkit::io::GnssPositionDebugLogProduct,
        ::navkit::io::GnssVelocityLogProduct,
        ::navkit::io::GnssVelocityDebugLogProduct,
        ::navkit::io::NavEstimateLogProduct<StateDef, Filter>,
        ::navkit::io::ImuIncrementLogProduct,
        ::navkit::io::ImuDebugLogProduct,
        ::navkit::io::FilterCorrectionLogProduct<StateDef, Filter>,
        ::navkit::io::GnssPositionUpdateLogProduct<PrimaryGnssPositionStatistics>,
        ::navkit::io::GnssVelocityUpdateLogProduct<PrimaryGnssVelocityStatistics>>;
    using App = ::navkit::app_support::SimulationApp<EcefInsGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::EcefInsGnssConfig;

} // namespace navkit::config
