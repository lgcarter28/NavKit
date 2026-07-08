// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/SimulationApp.hpp"
#include "navkit/app_support/emulation/EmulatorBinding.hpp"
#include "navkit/app_support/emulation/concrete/GnssEmulator.hpp"
#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/io/RunLogger.hpp"
#include "navkit/io/log_products/GnssPositionLogProduct.hpp"
#include "navkit/io/log_products/GnssPositionUpdateLogProduct.hpp"
#include "navkit/io/log_products/NavEstimateLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"
#include "navkit/products/ProfiledStationaryGnss.hpp"

#include <tuple>

namespace navkit::config::apps::navkit_sim
{

struct ProfiledStationaryGnssConfig
{
    using NavKit = ::navkit::config::navkit::ProfiledStationaryGnssConfig;

    using PrimaryGnssSensor = typename NavKit::PrimaryGnssSensor;
    using PrimaryGnssEmulator = ::navkit::app_support::GnssEmulator<PrimaryGnssSensor::Id>;
    using PrimaryGnssBinding =
        ::navkit::app_support::EmulatorBinding<PrimaryGnssEmulator, PrimaryGnssSensor>;
    using PrimaryGnssStatistics =
        ::navkit::core::estimation::MeasurementStatistics<PrimaryGnssSensor>;

    using EmulatorBindings = std::tuple<PrimaryGnssBinding>;

    using Logger =
        ::navkit::io::RunLogger<::navkit::io::TruthLogProduct,
                                ::navkit::io::GnssPositionLogProduct,
                                ::navkit::io::NavEstimateLogProduct,
                                ::navkit::io::GnssPositionUpdateLogProduct<PrimaryGnssStatistics>>;
    using App = ::navkit::app_support::SimulationApp<ProfiledStationaryGnssConfig>;
};

} // namespace navkit::config::apps::navkit_sim

namespace navkit::config
{

using SelectedConfig = apps::navkit_sim::ProfiledStationaryGnssConfig;

} // namespace navkit::config
