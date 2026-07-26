// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/products/components/filter/InsGyroAccelBiasCovarianceFloorDefault.hpp"
#include "navkit/products/components/filter/InsGyroAccelBiasInitialCovarianceDefault.hpp"
#include "navkit/products/components/profiling/Profilers.hpp"
#include "navkit/products/components/propagation/EcefInsPropagationConfig.hpp"
#include "navkit/products/variants/ecef_ins_gnss_lc/EcefInsGnssLc.hpp"

namespace navkit::config::navkit::products
{

using EcefInsGnssLcGyroAccelBiasProfiled =
    EcefInsGnssLc<core::estimation::InsGyroAccelBiasStateDef,
                  components::InsGyroAccelBiasInitialCovarianceDefault,
                  components::InsGyroAccelBiasCovarianceFloorDefault,
                  components::EcefInsPropagationConfig,
                  components::HostRingBufferProfiling>;

static_assert(api::config::NavKitProductConfigPolicy<EcefInsGnssLcGyroAccelBiasProfiled>);

} // namespace navkit::config::navkit::products

namespace navkit::config::navkit
{

using EcefInsGnssLcGyroAccelBiasProfiledConfig = products::EcefInsGnssLcGyroAccelBiasProfiled;

} // namespace navkit::config::navkit
