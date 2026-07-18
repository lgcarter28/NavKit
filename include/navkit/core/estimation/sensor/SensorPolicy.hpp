// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorDiagnosticsPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate>
concept SensorPolicy =
    requires {
        { Candidate::Id } -> std::convertible_to<SensorId>;
        typename Candidate::MeasurementModel_t;
        typename Candidate::Measurement_t;
        typename Candidate::ObservationContext_t;
        typename Candidate::Diagnostics_t;
    } && SensorDiagnosticsPolicy<typename Candidate::Diagnostics_t> &&
    requires(Candidate sensor, typename Candidate::Measurement_t measurement) {
        { sensor.push(measurement) } -> std::same_as<bool>;
        { sensor.has_measurement() } -> std::same_as<bool>;
        { sensor.pop(measurement) } -> std::same_as<bool>;
        sensor.update_observation_context(measurement);
        sensor.observation_context();
    };

} // namespace navkit::core::estimation
