// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/RingBuffer.hpp"
#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/estimation/sensor/SensorDiagnosticsPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicies.hpp"
#include "navkit/core/estimation/sensor/noise/NoisePolicy.hpp"

#include <cstddef>

namespace navkit::core::estimation
{

template<SensorId IdValue,
         typename MeasurementModel,
         std::size_t BufferSize,
         NoisePolicy<MeasurementModel, Measurement<MeasurementModel::M>> Noise = DefaultNoisePolicy,
         SensorDiagnosticsPolicy Diagnostics = DefaultSensorDiagnostics>
class Sensor
{
public:
    static constexpr SensorId Id = IdValue;

    using MeasurementModel_t = MeasurementModel;
    using O_t = typename MeasurementModel_t::O_t;
    using H_t = typename MeasurementModel_t::H_t;
    using R_t = typename MeasurementModel_t::R_t;
    using Measurement_t = Measurement<MeasurementModel_t::M>;
    using ObservationContext_t = typename MeasurementModel_t::ObservationContext;
    using Diagnostics_t = Diagnostics;

    bool push(const Measurement_t& meas)
    {
        return m_buffer.push(meas);
    }

    [[nodiscard]] bool has_measurement() const
    {
        return !m_buffer.empty();
    }

    bool pop(Measurement_t& meas)
    {
        return m_buffer.pop(meas);
    }

    [[nodiscard]] const ObservationContext_t& observation_context() const
    {
        return m_observation_ctx;
    }

    ObservationContext_t& observation_context()
    {
        return m_observation_ctx;
    }

    void update_observation_context(const Measurement_t& meas)
    {
        Noise::update(m_observation_ctx, meas);
    }

private:
    navkit::core::containers::RingBuffer<Measurement_t, BufferSize> m_buffer{};
    ObservationContext_t m_observation_ctx{};
};

} // namespace navkit::core::estimation
