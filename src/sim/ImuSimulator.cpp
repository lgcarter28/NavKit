// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/ImuSimulator.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/TriadCalibration.hpp"

#include <algorithm>
#include <cmath>

namespace navkit::sim
{

using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;

template<bool OutputConingScullingCompensated>
ImuSimulator<OutputConingScullingCompensated>::ImuSimulator(const ImuSimulatorConfig& cfg)
    : m_cfg(cfg)
    , m_gyro_bias_radps(cfg.gyro.bias)
    , m_accel_bias_mps2(cfg.accel.bias)
    , m_rng(cfg.seed)
{}

template<bool OutputConingScullingCompensated>
ImuIncrement
ImuSimulator<OutputConingScullingCompensated>::increment_from_interval(const ImuInterval& interval)
{
    ImuIncrement increment{};
    increment.time_s = interval.time_s;
    increment.dt_s = interval.dt_s;
    increment.delta_theta_ib_b_rad = interval.omega_ib_b_radps * interval.dt_s;
    increment.delta_v_ib_b_mps = interval.specific_force_ib_b_mps2 * interval.dt_s;
    return increment;
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::interval_from_truth_ecef(
    const TruthSample& previous, const TruthSample& current, ImuInterval& interval)
{
    ImuIntervalDebug debug{};
    return interval_from_truth_ecef(previous, current, interval, debug);
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::interval_from_truth_ecef(
    const TruthSample& previous,
    const TruthSample& current,
    ImuInterval& interval,
    ImuIntervalDebug& debug)
{
    const Time_t dt_s = current.time - previous.time;
    if (dt_s <= 0.0) {
        interval = {};
        debug = {};
        return false;
    }

    const Eigen::Quaternion<Scalar_t> q_prev = previous.q_b2e.normalized();
    const Eigen::Quaternion<Scalar_t> q_current = current.q_b2e.normalized();
    const Eigen::Quaternion<Scalar_t> q_body_relative = q_prev.conjugate() * q_current;
    const Vec3 delta_theta_eb_b = core::math::rotvec_rad_from_quaternion(q_body_relative);

    const Eigen::Quaternion<Scalar_t> q_mid = q_prev.slerp(0.5, q_current).normalized();
    const Vec3 omega_ie_b =
        q_mid.conjugate() * navkit::core::environment::planet_rate_fixed_radps<Wgs84>();
    const Vec3 delta_theta_ib_b = delta_theta_eb_b + (omega_ie_b * dt_s);

    const Vec3 a_bar_e = (current.v_e - previous.v_e) / dt_s;
    const Vec3 v_bar_e = 0.5 * (previous.v_e + current.v_e);
    const Vec3 p_bar_e = 0.5 * (previous.p_e + current.p_e);
    const Vec3 gravity_e = J2<Wgs84>::acceleration(p_bar_e);
    const Vec3 specific_force_e =
        a_bar_e +
        (2.0 * navkit::core::environment::planet_rate_fixed_radps<Wgs84>().cross(v_bar_e)) -
        gravity_e;
    const Vec3 specific_force_b = q_mid.conjugate() * specific_force_e;

    interval = {};
    interval.time_s = current.time;
    interval.dt_s = dt_s;
    interval.omega_ib_b_radps = delta_theta_ib_b / dt_s;
    interval.specific_force_ib_b_mps2 = specific_force_b;

    debug = {};
    debug.interval = interval;
    debug.p_bar_e_m = p_bar_e;
    debug.v_bar_e_mps = v_bar_e;
    debug.a_bar_e_mps2 = a_bar_e;
    debug.gravity_e_mps2 = gravity_e;
    debug.specific_force_e_mps2 = specific_force_e;
    debug.delta_theta_eb_b_rad = delta_theta_eb_b;
    return true;
}

template<bool OutputConingScullingCompensated>
Vec3 ImuSimulator<OutputConingScullingCompensated>::calibration_matrix_apply(
    const Vec3& input, const ImuTriadErrorConfig& config)
{
    return core::math::apply_triad_calibration(
        input, config.scale_factor, config.misalignment_rad, config.nonorthogonality);
}

template<bool OutputConingScullingCompensated>
Vec3 ImuSimulator<OutputConingScullingCompensated>::quantize(const Vec3& value, const Vec3& quantum)
{
    Vec3 result = value;
    for (Eigen::Index axis = 0; axis < result.size(); ++axis) {
        if (quantum(axis) > 0.0) {
            result(axis) = quantum(axis) * std::round(result(axis) / quantum(axis));
        }
    }
    return result;
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& previous,
                                                             const TruthSample& current,
                                                             ImuIncrement& increment)
{
    ImuInterval interval;
    return generate(previous, current, increment, interval);
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& previous,
                                                             const TruthSample& current,
                                                             ImuIncrement& increment,
                                                             ImuInterval& interval)
{
    ImuIntervalDebug debug{};
    return generate(previous, current, increment, interval, debug);
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& previous,
                                                             const TruthSample& current,
                                                             ImuIncrement& increment,
                                                             ImuInterval& interval,
                                                             ImuIntervalDebug& debug)
{
    if (!interval_from_truth_ecef(previous, current, interval, debug)) {
        increment = {};
        return false;
    }

    update_biases(interval.dt_s);

    const Vec3 gyro_noise = draw_normal_vector(m_cfg.gyro.white_noise_psd / interval.dt_s);
    const Vec3 accel_noise = draw_normal_vector(m_cfg.accel.white_noise_psd / interval.dt_s);

    const Vec3 raw_gyro_radps = calibration_matrix_apply(interval.omega_ib_b_radps, m_cfg.gyro) +
                                m_gyro_bias_radps + gyro_noise;
    const Vec3 raw_accel_mps2 =
        calibration_matrix_apply(interval.specific_force_ib_b_mps2, m_cfg.accel) +
        m_accel_bias_mps2 + accel_noise;

    increment = {};
    increment.time_s = interval.time_s;
    increment.dt_s = interval.dt_s;
    increment.delta_theta_ib_b_rad =
        quantize(raw_gyro_radps * interval.dt_s, m_cfg.gyro.quantization);
    increment.delta_v_ib_b_mps = quantize(raw_accel_mps2 * interval.dt_s, m_cfg.accel.quantization);
    return true;
}

template<bool OutputConingScullingCompensated>
void ImuSimulator<OutputConingScullingCompensated>::initialize(const TruthSample& initial)
{
    m_previous = initial;
    m_initialized = true;
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& current,
                                                             ImuIncrement& increment)
{
    ImuInterval interval;
    return generate(current, increment, interval);
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& current,
                                                             ImuIncrement& increment,
                                                             ImuInterval& interval)
{
    ImuIntervalDebug debug{};
    return generate(current, increment, interval, debug);
}

template<bool OutputConingScullingCompensated>
bool ImuSimulator<OutputConingScullingCompensated>::generate(const TruthSample& current,
                                                             ImuIncrement& increment,
                                                             ImuInterval& interval,
                                                             ImuIntervalDebug& debug)
{
    if (!m_initialized) {
        increment = {};
        interval = {};
        debug = {};
        return false;
    }

    if (!generate(m_previous, current, increment, interval, debug)) {
        return false;
    }
    m_previous = current;
    return true;
}

template<bool OutputConingScullingCompensated>
Vec3 ImuSimulator<OutputConingScullingCompensated>::draw_normal_vector(const Vec3& covariance_diag)
{
    return draw_normal_diag_cov<3>(covariance_diag, m_rng);
}

template<bool OutputConingScullingCompensated>
void ImuSimulator<OutputConingScullingCompensated>::update_biases(Time_t dt_s)
{
    m_gyro_bias_radps += draw_normal_vector(m_cfg.gyro.bias_random_walk_psd * dt_s);
    m_accel_bias_mps2 += draw_normal_vector(m_cfg.accel.bias_random_walk_psd * dt_s);
}

template class ImuSimulator<false>;
template class ImuSimulator<true>;

} // namespace navkit::sim
