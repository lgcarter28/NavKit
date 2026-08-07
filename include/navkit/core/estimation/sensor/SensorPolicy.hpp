// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/sensor/InnovationGate.hpp"
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
        typename Candidate::InnovationGate_t;
        typename Candidate::Diagnostics_t;
    } && SensorDiagnosticsPolicy<typename Candidate::Diagnostics_t> &&
    std::same_as<typename Candidate::InnovationGate_t,
                 InnovationGate<Candidate::MeasurementModel_t::M>> &&
    requires(Candidate sensor,
             const Candidate& const_sensor,
             typename Candidate::Measurement_t measurement,
             const Scalar_t probability) {
        { sensor.push(measurement) } -> std::same_as<bool>;
        { sensor.has_measurement() } -> std::same_as<bool>;
        { sensor.pop(measurement) } -> std::same_as<bool>;
        sensor.update_observation_context(measurement);
        sensor.observation_context();
        { sensor.configure_innovation_gate_probability(probability) } -> std::same_as<bool>;
        sensor.disable_innovation_gate();
        {
            const_sensor.innovation_gate()
        } -> std::same_as<const typename Candidate::InnovationGate_t&>;
    };

} // namespace navkit::core::estimation
