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

struct ImuRuntimeSample
{
    bool generated{false};
    navkit::sim::ImuInterval interval{};
    navkit::sim::ImuIntervalDebug debug{};
    core::estimation::ImuIncrement truth{};
    core::estimation::ImuIncrement measured{};
    core::Vec3 gyro_bias_truth_radps{core::Vec3::Zero()};
    core::Vec3 accel_bias_truth_mps2{core::Vec3::Zero()};
};

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
        ImuRuntimeSample unused{};
        return process(sample, navigator, unused);
    }

    template<typename Navigator>
    [[nodiscard]] bool
    process(const navkit::sim::TruthSample& sample, Navigator& navigator, ImuRuntimeSample& output)
    {
        if (!m_initialized) {
            m_simulator.initialize(sample);
            m_initialized = true;
            output = {};
            return true;
        }

        core::estimation::ImuIncrement increment;
        navkit::sim::ImuInterval interval;
        navkit::sim::ImuIntervalDebug debug;
        if (!m_simulator.generate(sample, increment, interval, debug)) {
            m_last_error = "failed to generate IMU increment";
            output = {};
            return false;
        }
        if (!navigator.push_imu(increment)) {
            m_last_error = "navigator IMU buffer is full";
            output = {};
            return false;
        }
        output = {};
        output.generated = true;
        output.interval = interval;
        output.debug = debug;
        output.truth = ImuSimulator::increment_from_interval(interval);
        output.measured = increment;
        output.gyro_bias_truth_radps = m_simulator.gyro_bias_radps();
        output.accel_bias_truth_mps2 = m_simulator.accel_bias_mps2();
        return true;
    }

    [[nodiscard]] std::string_view last_error() const
    {
        return m_last_error;
    }

    [[nodiscard]] const core::Vec3& gyro_bias_truth_radps() const
    {
        return m_simulator.gyro_bias_radps();
    }

    [[nodiscard]] const core::Vec3& accel_bias_truth_mps2() const
    {
        return m_simulator.accel_bias_mps2();
    }

private:
    ImuSimulator m_simulator;
    bool m_initialized{false};
    std::string_view m_last_error{};
};

} // namespace navkit::app_support
