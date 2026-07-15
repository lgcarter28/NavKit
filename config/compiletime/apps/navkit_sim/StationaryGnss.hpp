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
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionUpdateLogProduct.hpp"
#include "navkit/io/log_products/ImuDebugLogProduct.hpp"
#include "navkit/io/log_products/ImuIncrementLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/products/StationaryGnss.hpp"
#include "navkit/sim/ImuSimulator.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct StationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::StationaryGnssConfig;

    using PrimaryGnssSensor = typename NavKit::PrimaryGnssSensor;
    using PrimaryGnssEmulator = ::navkit::app_support::GnssEmulator<PrimaryGnssSensor::Id>;
    using PrimaryGnssBinding =
        ::navkit::app_support::EmulatorBinding<PrimaryGnssEmulator, PrimaryGnssSensor>;
    using PrimaryGnssStatistics =
        ::navkit::core::estimation::MeasurementStatistics<PrimaryGnssSensor>;
    using Filter = typename NavKit::Filter;
    using StateDef = typename NavKit::StateDef;

    using EmulatorBindings = std::tuple<PrimaryGnssBinding>;
    using ImuSimulator = ::navkit::sim::ImuSimulator;

    using NavInitializationProvider = ::navkit::app_support::PvaExplicitInitializationProvider;
    using TransferAlignmentProvider = ::navkit::app_support::NoTransferAlignmentProvider;

    using Logger =
        ::navkit::io::RunLogger<::navkit::io::TruthLogProduct,
                                ::navkit::io::GnssPositionLogProduct,
                                ::navkit::io::NavEstimateLogProduct<StateDef, Filter>,
                                ::navkit::io::ImuIncrementLogProduct,
                                ::navkit::io::ImuDebugLogProduct,
                                ::navkit::io::FilterCorrectionLogProduct<StateDef, Filter>,
                                ::navkit::io::GnssPositionUpdateLogProduct<PrimaryGnssStatistics>>;
    using App = ::navkit::app_support::SimulationApp<StationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::StationaryGnssConfig;

} // namespace navkit::config
