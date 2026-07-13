// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/ImuSimulator.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Skew.hpp"

#include <algorithm>
#include <cmath>

namespace navkit::sim
{

namespace
{

using navkit::core::environment::J2;
using navkit::core::environment::Wgs84;

[[nodiscard]] Eigen::Matrix<Scalar_t, 3, 3> scale_matrix(const Vec3& scale_factor)
{
    Eigen::Matrix<Scalar_t, 3, 3> scale = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    scale.diagonal() += scale_factor;
    return scale;
}

[[nodiscard]] Eigen::Matrix<Scalar_t, 3, 3> nonorthogonality_matrix(const Vec3& nonorthogonality)
{
    Eigen::Matrix<Scalar_t, 3, 3> matrix = Eigen::Matrix<Scalar_t, 3, 3>::Identity();
    matrix(1, 0) = nonorthogonality.x();
    matrix(2, 0) = nonorthogonality.y();
    matrix(2, 1) = nonorthogonality.z();
    return matrix;
}

[[nodiscard]] Eigen::Matrix<Scalar_t, 3, 3> misalignment_matrix(const Vec3& misalignment_rad)
{
    return Eigen::Matrix<Scalar_t, 3, 3>::Identity() - core::math::skew_symmetric(misalignment_rad);
}

} // namespace

ImuSimulator::ImuSimulator(const ImuSimulatorConfig& cfg)
    : m_cfg(cfg)
    , m_gyro_bias_radps(cfg.gyro.bias)
    , m_accel_bias_mps2(cfg.accel.bias)
    , m_rng(cfg.seed)
{}

bool ImuSimulator::ideal_interval_from_truth_ecef(const TruthSample& previous,
                                                  const TruthSample& current,
                                                  IdealImuInterval& interval)
{
    const auto dt_s = current.time - previous.time;
    if (dt_s <= 0.0) {
        interval = {};
        return false;
    }

    const auto q_prev = previous.q_eb.normalized();
    const auto q_current = current.q_eb.normalized();
    const auto q_body_relative = q_current * q_prev.conjugate();
    const auto delta_theta_eb_b = core::math::rotation_vector_from_quaternion(q_body_relative);

    const auto q_mid = q_prev.slerp(0.5, q_current).normalized();
    const auto omega_ie_b = q_mid * navkit::core::environment::planet_rate_fixed_radps<Wgs84>();
    const auto delta_theta_ib_b = delta_theta_eb_b + (omega_ie_b * dt_s);

    const auto a_bar_e = (current.v_e - previous.v_e) / dt_s;
    const auto v_bar_e = 0.5 * (previous.v_e + current.v_e);
    const auto p_bar_e = 0.5 * (previous.p_e + current.p_e);
    const auto gravity_e = J2<Wgs84>::acceleration(p_bar_e);
    const auto specific_force_e =
        a_bar_e +
        (2.0 * navkit::core::environment::planet_rate_fixed_radps<Wgs84>().cross(v_bar_e)) -
        gravity_e;
    const auto specific_force_b = q_mid * specific_force_e;

    interval = {.time_s = current.time,
                .dt_s = dt_s,
                .omega_ib_b_radps = delta_theta_ib_b / dt_s,
                .specific_force_ib_b_mps2 = specific_force_b,
                .delta_theta_ib_b_rad = delta_theta_ib_b,
                .delta_v_ib_b_mps = specific_force_b * dt_s};
    return true;
}

Vec3 ImuSimulator::calibration_matrix_apply(const Vec3& input, const ImuTriadErrorConfig& config)
{
    return scale_matrix(config.scale_factor) * nonorthogonality_matrix(config.nonorthogonality) *
           misalignment_matrix(config.misalignment_rad) * input;
}

Vec3 ImuSimulator::quantize(const Vec3& value, const Vec3& quantum)
{
    Vec3 result = value;
    for (Eigen::Index axis = 0; axis < result.size(); ++axis) {
        if (quantum(axis) > 0.0) {
            result(axis) = quantum(axis) * std::round(result(axis) / quantum(axis));
        }
    }
    return result;
}

bool ImuSimulator::generate(const TruthSample& previous,
                            const TruthSample& current,
                            ImuIncrement& increment)
{
    IdealImuInterval ideal;
    if (!ideal_interval_from_truth_ecef(previous, current, ideal)) {
        increment = {};
        return false;
    }

    update_biases(ideal.dt_s);

    const auto gyro_noise = draw_normal_vector(m_cfg.gyro.white_noise_psd / ideal.dt_s);
    const auto accel_noise = draw_normal_vector(m_cfg.accel.white_noise_psd / ideal.dt_s);

    const auto raw_gyro_radps = calibration_matrix_apply(ideal.omega_ib_b_radps, m_cfg.gyro) +
                                m_gyro_bias_radps + gyro_noise;
    const auto raw_accel_mps2 =
        calibration_matrix_apply(ideal.specific_force_ib_b_mps2, m_cfg.accel) + m_accel_bias_mps2 +
        accel_noise;

    increment = {
        .time_s = ideal.time_s,
        .dt_s = ideal.dt_s,
        .delta_theta_ib_b_rad = quantize(raw_gyro_radps * ideal.dt_s, m_cfg.gyro.quantization),
        .delta_v_ib_b_mps = quantize(raw_accel_mps2 * ideal.dt_s, m_cfg.accel.quantization)};
    return true;
}

void ImuSimulator::initialize(const TruthSample& initial)
{
    m_previous = initial;
    m_initialized = true;
}

bool ImuSimulator::generate(const TruthSample& current, ImuIncrement& increment)
{
    if (!m_initialized) {
        increment = {};
        return false;
    }

    if (!generate(m_previous, current, increment)) {
        return false;
    }
    m_previous = current;
    return true;
}

Vec3 ImuSimulator::draw_normal_vector(const Vec3& covariance_diag)
{
    return draw_normal_diag_cov<3>(covariance_diag, m_rng);
}

void ImuSimulator::update_biases(Time_t dt_s)
{
    m_gyro_bias_radps += draw_normal_vector(m_cfg.gyro.bias_random_walk_psd * dt_s);
    m_accel_bias_mps2 += draw_normal_vector(m_cfg.accel.bias_random_walk_psd * dt_s);
}

} // namespace navkit::sim
