// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

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
#include "navkit/io/log_products/TrajectoryAutopilotVehicleLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryBodyLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryEcefLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryEciLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryGuidanceLogProduct.hpp"
#include "navkit/io/log_products/TrajectoryNedLogProduct.hpp"
#include "navkit/io/log_products/TruthLogProduct.hpp"

namespace navkit::app_support
{

template<typename NavKit>
using RuntimeLogger = navkit::io::RunLogger<
    navkit::io::TruthLogProduct,
    navkit::io::GnssPositionLogProduct,
    navkit::io::GnssPositionDebugLogProduct,
    navkit::io::GnssVelocityLogProduct,
    navkit::io::GnssVelocityDebugLogProduct,
    navkit::io::NavEstimateLogProduct<typename NavKit::StateDef, typename NavKit::Filter>,
    navkit::io::ImuIncrementLogProduct,
    navkit::io::ImuDebugLogProduct,
    navkit::io::FilterCorrectionLogProduct<typename NavKit::StateDef, typename NavKit::Filter>,
    navkit::io::GnssPositionUpdateLogProduct<navkit::core::estimation::MeasurementStatistics<
        typename NavKit::PrimaryGnssPositionSensor>>,
    navkit::io::GnssVelocityUpdateLogProduct<navkit::core::estimation::MeasurementStatistics<
        typename NavKit::PrimaryGnssVelocitySensor>>,
    navkit::io::TrajectoryEcefLogProduct,
    navkit::io::TrajectoryEciLogProduct,
    navkit::io::TrajectoryNedLogProduct,
    navkit::io::TrajectoryBodyLogProduct,
    navkit::io::TrajectoryGuidanceLogProduct,
    navkit::io::TrajectoryAutopilotVehicleLogProduct>;

} // namespace navkit::app_support
