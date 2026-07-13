// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/emulation/concrete/ImuRuntimeConfig.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/sim/ImuSimulatorPolicy.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::app_support
{

template<navkit::sim::ImuSimulatorPolicy ImuSimulator>
class ImuRuntime
{
public:
    explicit ImuRuntime(const nlohmann::json& cfg)
        : m_simulator(imu_simulator_config_from_json(cfg))
    {}

    template<typename Navigator>
    [[nodiscard]] bool process(Navigator& navigator, const navkit::sim::TruthSample& sample)
    {
        if (!m_initialized) {
            m_simulator.initialize(sample);
            m_initialized = true;
            return true;
        }

        core::estimation::ImuIncrement increment;
        if (!m_simulator.generate(sample, increment)) {
            m_last_error = "failed to generate IMU increment";
            return false;
        }
        if (!navigator.push_imu(increment)) {
            m_last_error = "navigator IMU buffer is full";
            return false;
        }
        return true;
    }

    [[nodiscard]] std::string_view last_error() const
    {
        return m_last_error;
    }

private:
    ImuSimulator m_simulator;
    bool m_initialized{false};
    std::string_view m_last_error{};
};

} // namespace navkit::app_support
