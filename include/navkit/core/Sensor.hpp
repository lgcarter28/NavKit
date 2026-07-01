// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/Measurement.hpp"
#include "navkit/core/NoisePolicy.hpp"
#include "navkit/core/RingBuffer.hpp"
#include "navkit/core/policies/NoisePolicies.hpp"

#include <cstddef>

namespace navkit
{

template<typename Model,
         std::size_t BufferSize,
         NoisePolicy<Model, Measurement<Model::M>> Noise = DefaultNoisePolicy>
class Sensor
{
public:
    using Model_t = Model;
    using O_t = typename Model_t::O_t;
    using H_t = typename Model_t::H_t;
    using R_t = typename Model_t::R_t;
    using Measurement_t = Measurement<Model_t::M>;
    using NoiseContext_t = typename Model_t::NoiseContext;

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

    [[nodiscard]] const NoiseContext_t& noise_context() const
    {
        return m_noise_ctx;
    }

    NoiseContext_t& noise_context()
    {
        return m_noise_ctx;
    }

    void update_noise_context(const Measurement_t& meas)
    {
        Noise::update(m_noise_ctx, meas);
    }

private:
    RingBuffer<Measurement_t, BufferSize> m_buffer{};
    NoiseContext_t m_noise_ctx{};
};

} // namespace navkit
