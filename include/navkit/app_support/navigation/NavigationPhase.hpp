// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace navkit::app_support
{

/**
 * \brief Complete innovation-gate selection for one sensor role in a Navigation phase.
 *
 * \details `enabled` is the final gate state for the phase. When `probability` has a value, that
 * value is the phase override or the sensor's resolved baseline and is applied before the enabled
 * state. A disabled phase may omit the probability entirely. No field inherits mutable state from
 * whichever phase happened to execute previously.
 */
struct NavigationInnovationGatePhase
{
    std::optional<core::Scalar_t> probability{};
    bool enabled{false};
};

/**
 * \brief Navigation controls selected by one named mission phase.
 *
 * \details This first concrete seam deliberately names the two sensor roles supported by the
 * current ECEF INS/GNSS product. The resolved mission phase owns these sensor-specific controls;
 * app support preserves mission phase order instead of joining a second phase catalog by name.
 */
struct NavigationPhase
{
    std::string id{};
    NavigationInnovationGatePhase primary_gnss_position{};
    NavigationInnovationGatePhase primary_gnss_velocity{};
};

/** \brief Reports whether a Navigation-phase chi-square probability is valid. */
[[nodiscard]] inline bool navigation_phase_probability_is_valid(const core::Scalar_t probability)
{
    return std::isfinite(probability) && probability > 0.0 && probability < 1.0;
}

/** \brief Reports whether one complete runtime gate selection can be applied. */
[[nodiscard]] inline bool
navigation_innovation_gate_phase_is_valid(const NavigationInnovationGatePhase& gate)
{
    return (!gate.enabled || gate.probability.has_value()) &&
           (!gate.probability.has_value() ||
            navigation_phase_probability_is_valid(*gate.probability));
}

/**
 * \brief Applies one complete Navigation phase to the current product's primary GNSS sensors.
 *
 * \details Both probabilities are validated before either sensor is mutated. The two sensor
 * probabilities are then configured before their final enabled states are selected. Given the
 * existing bool-returning sensor API, this is the narrowest fail-before-mutation transaction
 * boundary available without moving mission knowledge into embedded NavKit.
 */
template<navkit::api::config::NavKitProductConfigPolicy NavKit>
[[nodiscard]] bool apply_navigation_phase(const NavigationPhase& phase,
                                          typename NavKit::Navigator& navigator)
{
    using PositionSensor = typename NavKit::PrimaryGnssPositionSensor;
    using VelocitySensor = typename NavKit::PrimaryGnssVelocitySensor;

    if (!navigation_innovation_gate_phase_is_valid(phase.primary_gnss_position) ||
        !navigation_innovation_gate_phase_is_valid(phase.primary_gnss_velocity)) {
        return false;
    }

    constexpr std::size_t position_sensor_index =
        core::estimation::SensorIndexFromId_v<PositionSensor::Id, typename NavKit::Sensors>;
    constexpr std::size_t velocity_sensor_index =
        core::estimation::SensorIndexFromId_v<VelocitySensor::Id, typename NavKit::Sensors>;
    PositionSensor& position_sensor = navigator.template sensor<position_sensor_index>();
    VelocitySensor& velocity_sensor = navigator.template sensor<velocity_sensor_index>();

    if (phase.primary_gnss_position.probability.has_value() &&
        !position_sensor.configure_innovation_gate_probability(
            *phase.primary_gnss_position.probability)) {
        return false;
    }
    if (phase.primary_gnss_velocity.probability.has_value() &&
        !velocity_sensor.configure_innovation_gate_probability(
            *phase.primary_gnss_velocity.probability)) {
        return false;
    }
    return position_sensor.set_innovation_gate_enabled(phase.primary_gnss_position.enabled) &&
           velocity_sensor.set_innovation_gate_enabled(phase.primary_gnss_velocity.enabled);
}

} // namespace navkit::app_support
