// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/emulation/concrete/ImuRuntimeConfig.hpp"
#include "navkit/app_support/initialization/InitialTruthReference.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
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
    core::Vec3 truth_cumsum_delta_theta_ib_b_rad{core::Vec3::Zero()};
    core::Vec3 truth_cumsum_delta_v_ib_b_mps{core::Vec3::Zero()};
    core::Vec3 measured_cumsum_delta_theta_ib_b_rad{core::Vec3::Zero()};
    core::Vec3 measured_cumsum_delta_v_ib_b_mps{core::Vec3::Zero()};
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
        const core::estimation::ImuIncrement truth_increment =
            ImuSimulator::increment_from_interval(interval);
        m_truth_delta_theta_sum += truth_increment.delta_theta_ib_b_rad;
        m_truth_delta_v_sum += truth_increment.delta_v_ib_b_mps;
        m_measured_delta_theta_sum += increment.delta_theta_ib_b_rad;
        m_measured_delta_v_sum += increment.delta_v_ib_b_mps;

        output = {};
        output.generated = true;
        output.interval = interval;
        output.debug = debug;
        output.truth = truth_increment;
        output.measured = increment;
        output.truth_cumsum_delta_theta_ib_b_rad = m_truth_delta_theta_sum;
        output.truth_cumsum_delta_v_ib_b_mps = m_truth_delta_v_sum;
        output.measured_cumsum_delta_theta_ib_b_rad = m_measured_delta_theta_sum;
        output.measured_cumsum_delta_v_ib_b_mps = m_measured_delta_v_sum;
        output.gyro_bias_truth_radps = m_simulator.gyro_bias_radps();
        output.accel_bias_truth_mps2 = m_simulator.accel_bias_mps2();
        return true;
    }

    [[nodiscard]] std::string_view last_error() const
    {
        return m_last_error;
    }

    template<core::estimation::StateSpaceDefPolicy StateDef>
    void apply_initial_truth_reference(InitialTruthReference<StateDef>& reference) const
    {
        using Nominal = typename StateDef::Nominal;

        if constexpr (requires { typename Nominal::GyroB; }) {
            core::estimation::segment<typename Nominal::GyroB>(reference.truth_state) =
                m_simulator.gyro_bias_radps();
        }
        if constexpr (requires { typename Nominal::AccB; }) {
            core::estimation::segment<typename Nominal::AccB>(reference.truth_state) =
                m_simulator.accel_bias_mps2();
        }
    }

private:
    ImuSimulator m_simulator;
    core::Vec3 m_truth_delta_theta_sum{core::Vec3::Zero()};
    core::Vec3 m_truth_delta_v_sum{core::Vec3::Zero()};
    core::Vec3 m_measured_delta_theta_sum{core::Vec3::Zero()};
    core::Vec3 m_measured_delta_v_sum{core::Vec3::Zero()};
    bool m_initialized{false};
    std::string_view m_last_error{};
};

} // namespace navkit::app_support
