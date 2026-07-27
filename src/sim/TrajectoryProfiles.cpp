// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/TrajectoryProfiles.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <utility>

namespace navkit::sim
{

namespace
{

using Planet = core::environment::Wgs84;
using Gravity = core::environment::J2<Planet>;

struct GeodeticPosition
{
    core::Scalar_t lat_rad{};
    core::Scalar_t lon_rad{};
    core::Scalar_t h_m{};
};

[[nodiscard]] bool profile_is_valid(const TrajectoryProfileConfig& cfg)
{
    return cfg.duration_s > 0.0 && core::rational_cadence_is_valid(cfg.t_epoch, cfg.rate) &&
           cfg.p_e_m.norm() > 0.0 && cfg.q_b2e.norm() > 0.0;
}

[[nodiscard]] GeodeticPosition geodetic_from_ecef_m(const core::Vec3& p_e_m)
{
    const core::Scalar_t radial_distance_m = std::hypot(p_e_m.x(), p_e_m.y());
    GeodeticPosition result{};
    result.lon_rad = std::atan2(p_e_m.y(), p_e_m.x());
    result.lat_rad = std::atan2(p_e_m.z(), radial_distance_m * (1.0 - Planet::e2));
    for (int iteration = 0; iteration < 8; ++iteration) {
        const core::Scalar_t sin_lat = std::sin(result.lat_rad);
        const core::Scalar_t prime_vertical_radius_m =
            Planet::a_m / std::sqrt(1.0 - (Planet::e2 * sin_lat * sin_lat));
        result.h_m = (radial_distance_m / std::cos(result.lat_rad)) - prime_vertical_radius_m;
        result.lat_rad =
            std::atan2(p_e_m.z(),
                       radial_distance_m * (1.0 - ((Planet::e2 * prime_vertical_radius_m) /
                                                   (prime_vertical_radius_m + result.h_m))));
    }
    return result;
}

[[nodiscard]] core::Vec3 ecef_from_geodetic(const GeodeticPosition& position)
{
    const core::Scalar_t sin_lat = std::sin(position.lat_rad);
    const core::Scalar_t cos_lat = std::cos(position.lat_rad);
    const core::Scalar_t sin_lon = std::sin(position.lon_rad);
    const core::Scalar_t cos_lon = std::cos(position.lon_rad);
    const core::Scalar_t prime_vertical_radius_m =
        Planet::a_m / std::sqrt(1.0 - (Planet::e2 * sin_lat * sin_lat));
    return core::Vec3{(prime_vertical_radius_m + position.h_m) * cos_lat * cos_lon,
                      (prime_vertical_radius_m + position.h_m) * cos_lat * sin_lon,
                      ((1.0 - Planet::e2) * prime_vertical_radius_m + position.h_m) * sin_lat};
}

[[nodiscard]] core::Mat3 ecef_to_ned_matrix(const GeodeticPosition& position)
{
    const core::Scalar_t sin_lat = std::sin(position.lat_rad);
    const core::Scalar_t cos_lat = std::cos(position.lat_rad);
    const core::Scalar_t sin_lon = std::sin(position.lon_rad);
    const core::Scalar_t cos_lon = std::cos(position.lon_rad);

    core::Mat3 C_e2n{};
    C_e2n << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon, 0.0,
        -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
    return C_e2n;
}

[[nodiscard]] core::Mat3 ecef_to_ned_matrix(const core::Vec3& p_e_m)
{
    return ecef_to_ned_matrix(geodetic_from_ecef_m(p_e_m));
}

[[nodiscard]] Eigen::Quaternion<core::Scalar_t>
attitude_b2e_from_local_rpy(const core::Vec3& p_e_m, const core::Vec3& rpy_b2n_rad)
{
    const Eigen::Quaternion<core::Scalar_t> q_n2e{ecef_to_ned_matrix(p_e_m).transpose()};
    return core::math::normalized_with_positive_scalar(
        q_n2e * core::math::quaternion_from_rpy_rad(rpy_b2n_rad));
}

[[nodiscard]] core::Vec3 local_rpy_from_attitude(const core::Vec3& p_e_m,
                                                 const Eigen::Quaternion<core::Scalar_t>& q_b2e)
{
    const Eigen::Quaternion<core::Scalar_t> q_e2n{ecef_to_ned_matrix(p_e_m)};
    return core::math::rpy_rad_from_quaternion(q_e2n * q_b2e);
}

[[nodiscard]] core::Scalar_t wrap_angle_rad(core::Scalar_t angle_rad)
{
    while (angle_rad > std::numbers::pi_v<core::Scalar_t>) {
        angle_rad -= 2.0 * std::numbers::pi_v<core::Scalar_t>;
    }
    while (angle_rad < -std::numbers::pi_v<core::Scalar_t>) {
        angle_rad += 2.0 * std::numbers::pi_v<core::Scalar_t>;
    }
    return angle_rad;
}

[[nodiscard]] bool append_timestamp(const TrajectoryProfileConfig& cfg,
                                    const core::SampleIndex sample_index,
                                    core::Timestamp& t,
                                    core::Time_t& elapsed_s)
{
    if (!core::timestamp_at_sample_index(cfg.t_epoch, cfg.rate, sample_index, t)) {
        return false;
    }
    core::Duration elapsed{};
    if (!core::elapsed_time(t, cfg.t_epoch, elapsed)) {
        return false;
    }
    elapsed_s = core::duration_seconds(elapsed);
    return elapsed_s <= (cfg.duration_s + 1.0e-12);
}

[[nodiscard]] core::Time_t dt_to_next_sample(const TrajectoryProfileConfig& cfg,
                                             const core::SampleIndex sample_index)
{
    core::Timestamp current{};
    core::Timestamp next{};
    core::Duration duration{};
    if (!core::timestamp_at_sample_index(cfg.t_epoch, cfg.rate, sample_index, current) ||
        !core::timestamp_at_sample_index(cfg.t_epoch, cfg.rate, sample_index + 1U, next) ||
        !core::elapsed_time(next, current, duration)) {
        return 0.0;
    }
    return core::duration_seconds(duration);
}

[[nodiscard]] core::Vec3 velocity_n_mps(const core::Scalar_t speed_mps,
                                        const core::Scalar_t pitch_rad,
                                        const core::Scalar_t heading_rad)
{
    const core::Scalar_t horizontal_speed_mps = speed_mps * std::cos(pitch_rad);
    return core::Vec3{horizontal_speed_mps * std::cos(heading_rad),
                      horizontal_speed_mps * std::sin(heading_rad),
                      -speed_mps * std::sin(pitch_rad)};
}

void integrate_geodetic(const core::Vec3& v_n_mps,
                        const core::Time_t dt_s,
                        GeodeticPosition& position)
{
    const core::Scalar_t sin_lat = std::sin(position.lat_rad);
    const core::Scalar_t denominator = 1.0 - (Planet::e2 * sin_lat * sin_lat);
    const core::Scalar_t prime_vertical_radius_m = Planet::a_m / std::sqrt(denominator);
    const core::Scalar_t meridian_radius_m =
        Planet::a_m * (1.0 - Planet::e2) / (denominator * std::sqrt(denominator));
    position.lat_rad += (v_n_mps.x() / (meridian_radius_m + position.h_m)) * dt_s;
    const core::Scalar_t cos_lat = std::cos(position.lat_rad);
    if (std::abs(cos_lat) > 1.0e-12) {
        position.lon_rad +=
            (v_n_mps.y() / ((prime_vertical_radius_m + position.h_m) * cos_lat)) * dt_s;
    }
    position.h_m -= v_n_mps.z() * dt_s;
}

[[nodiscard]] TruthSample make_local_sample(const core::Timestamp& t,
                                            const GeodeticPosition& position,
                                            const core::Vec3& rpy_b2n_rad,
                                            const core::Vec3& v_n_mps)
{
    TruthSample sample{};
    sample.t = t;
    sample.p_e = ecef_from_geodetic(position);
    sample.v_e = ecef_to_ned_matrix(position).transpose() * v_n_mps;
    sample.q_b2e = attitude_b2e_from_local_rpy(sample.p_e, rpy_b2n_rad);
    return sample;
}

[[nodiscard]] TruthTrajectory from_samples(std::vector<TruthSample> samples)
{
    if (samples.size() < 2U) {
        return {};
    }
    populate_truth_angular_rates(samples);
    return TruthTrajectory{std::move(samples)};
}

} // namespace

void populate_truth_angular_rates(std::vector<TruthSample>& samples)
{
    if (samples.empty()) {
        return;
    }
    const core::Vec3 planet_rate_e_radps = core::environment::planet_rate_fixed_radps<Planet>();
    if (samples.size() == 1U) {
        samples.front().w_ib_b_radps = samples.front().q_b2e.conjugate() * planet_rate_e_radps;
        return;
    }

    for (std::size_t index = 0U; index < samples.size(); ++index) {
        const std::size_t first_index = index + 1U < samples.size() ? index : index - 1U;
        const std::size_t second_index = index + 1U < samples.size() ? index + 1U : index;
        const TruthSample& first = samples.at(first_index);
        const TruthSample& second = samples.at(second_index);
        core::Duration duration{};
        if (!core::elapsed_time(second.t, first.t, duration)) {
            samples.at(index).w_ib_b_radps = {};
            continue;
        }
        const core::Time_t dt_s = core::duration_seconds(duration);
        if (dt_s <= 0.0) {
            samples.at(index).w_ib_b_radps = {};
            continue;
        }
        const Eigen::Quaternion<core::Scalar_t> q_mid =
            core::math::normalized_with_positive_scalar(first.q_b2e.slerp(0.5, second.q_b2e));
        const Eigen::Quaternion<core::Scalar_t> q_body_relative =
            first.q_b2e.conjugate() * second.q_b2e;
        samples.at(index).w_ib_b_radps =
            (core::math::rotvec_rad_from_quaternion(q_body_relative) / dt_s) +
            (q_mid.conjugate() * planet_rate_e_radps);
    }
}

TruthTrajectory ballistic_trajectory(const BallisticTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.launch_pad_duration_s < 0.0 ||
        cfg.boost_duration_s <= 0.0 || cfg.boost_acceleration_b_x_mps2 <= 0.0) {
        return {};
    }

    std::vector<TruthSample> samples{};
    core::Vec3 p_e_m = cfg.profile.p_e_m;
    core::Vec3 v_e_mps = cfg.profile.v_e_mps;
    const Eigen::Quaternion<core::Scalar_t> q_b2e =
        core::math::normalized_with_positive_scalar(cfg.profile.q_b2e);
    const core::Vec3 omega_ie_e = core::environment::planet_rate_fixed_radps<Planet>();

    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp t{};
        core::Time_t elapsed_s{};
        if (!append_timestamp(cfg.profile, index, t, elapsed_s)) {
            break;
        }

        TruthSample sample{};
        sample.t = t;
        sample.p_e = p_e_m;
        sample.v_e = v_e_mps;
        sample.q_b2e = q_b2e;
        samples.push_back(sample);

        const core::Time_t dt_s = dt_to_next_sample(cfg.profile, index);
        if (dt_s <= 0.0 || elapsed_s >= cfg.profile.duration_s) {
            continue;
        }
        if (elapsed_s < cfg.launch_pad_duration_s) {
            continue;
        }

        const bool boost_active = elapsed_s < (cfg.launch_pad_duration_s + cfg.boost_duration_s);
        const core::Vec3 specific_force_e =
            boost_active ? q_b2e * core::Vec3{cfg.boost_acceleration_b_x_mps2, 0.0, 0.0}
                         : core::Vec3::Zero();
        const core::Vec3 acceleration_e =
            specific_force_e + Gravity::acceleration(p_e_m) - (2.0 * omega_ie_e.cross(v_e_mps));
        p_e_m += (v_e_mps * dt_s) + (0.5 * acceleration_e * dt_s * dt_s);
        v_e_mps += acceleration_e * dt_s;
    }
    return from_samples(std::move(samples));
}

TruthTrajectory constant_altitude_trajectory(const ConstantAltitudeTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.speed_mps <= 0.0) {
        return {};
    }

    const GeodeticPosition initial_position = geodetic_from_ecef_m(cfg.profile.p_e_m);
    const core::Vec3 initial_rpy = local_rpy_from_attitude(cfg.profile.p_e_m, cfg.profile.q_b2e);
    const core::Scalar_t initial_heading_rad = initial_rpy.z();
    const core::Scalar_t great_circle_radius_m = Planet::a_m + initial_position.h_m;
    std::vector<TruthSample> samples{};

    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp t{};
        core::Time_t elapsed_s{};
        if (!append_timestamp(cfg.profile, index, t, elapsed_s)) {
            break;
        }
        const core::Scalar_t arc_rad = (cfg.speed_mps * elapsed_s) / great_circle_radius_m;
        GeodeticPosition position{};
        position.lat_rad = std::asin((std::sin(initial_position.lat_rad) * std::cos(arc_rad)) +
                                     (std::cos(initial_position.lat_rad) * std::sin(arc_rad) *
                                      std::cos(initial_heading_rad)));
        position.lon_rad = initial_position.lon_rad +
                           std::atan2(std::sin(initial_heading_rad) * std::sin(arc_rad) *
                                          std::cos(initial_position.lat_rad),
                                      std::cos(arc_rad) - (std::sin(initial_position.lat_rad) *
                                                           std::sin(position.lat_rad)));
        position.h_m = initial_position.h_m;
        const core::Vec3 p_e_m = ecef_from_geodetic(position);
        const core::Scalar_t heading_rad =
            std::atan2(std::sin(initial_heading_rad) * std::cos(initial_position.lat_rad),
                       (std::cos(arc_rad) * std::cos(initial_heading_rad) *
                        std::cos(initial_position.lat_rad)) -
                           (std::sin(arc_rad) * std::sin(initial_position.lat_rad)));
        const core::Vec3 v_n_mps = velocity_n_mps(cfg.speed_mps, 0.0, heading_rad);
        samples.push_back(
            make_local_sample(t, position, core::Vec3{initial_rpy.x(), 0.0, heading_rad}, v_n_mps));
        samples.back().p_e = p_e_m;
    }
    return from_samples(std::move(samples));
}

TruthTrajectory calibration_trajectory(const CalibrationTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.speed_mps <= 0.0 || cfg.amplitude_rad <= 0.0 ||
        cfg.period_s <= 0.0) {
        return {};
    }

    GeodeticPosition position = geodetic_from_ecef_m(cfg.profile.p_e_m);
    const core::Vec3 initial_rpy = local_rpy_from_attitude(cfg.profile.p_e_m, cfg.profile.q_b2e);
    const core::Scalar_t initial_heading_rad = initial_rpy.z();
    const core::Scalar_t angular_frequency_radps =
        (2.0 * std::numbers::pi_v<core::Scalar_t>) / cfg.period_s;
    std::vector<TruthSample> samples{};

    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp t{};
        core::Time_t elapsed_s{};
        if (!append_timestamp(cfg.profile, index, t, elapsed_s)) {
            break;
        }

        core::Scalar_t roll_rad{};
        core::Scalar_t pitch_rad{};
        core::Scalar_t heading_rad = initial_heading_rad;
        if (cfg.maneuver == CalibrationManeuver::HorizontalSTurn) {
            heading_rad += cfg.amplitude_rad * std::sin(angular_frequency_radps * elapsed_s);
            const core::Scalar_t heading_rate_radps = cfg.amplitude_rad * angular_frequency_radps *
                                                      std::cos(angular_frequency_radps * elapsed_s);
            roll_rad = std::atan((cfg.speed_mps * heading_rate_radps) / 9.80665);
        }
        else if (cfg.maneuver == CalibrationManeuver::VerticalSTurn) {
            pitch_rad = cfg.amplitude_rad * std::sin(angular_frequency_radps * elapsed_s);
        }
        else {
            roll_rad = cfg.amplitude_rad * std::sin(angular_frequency_radps * elapsed_s);
        }

        const core::Vec3 v_n_mps = velocity_n_mps(cfg.speed_mps, pitch_rad, heading_rad);
        samples.push_back(
            make_local_sample(t, position, core::Vec3{roll_rad, pitch_rad, heading_rad}, v_n_mps));

        const core::Time_t dt_s = dt_to_next_sample(cfg.profile, index);
        if (dt_s > 0.0 && elapsed_s < cfg.profile.duration_s) {
            integrate_geodetic(v_n_mps, dt_s, position);
        }
    }
    return from_samples(std::move(samples));
}

TruthTrajectory waypoint_trajectory(const WaypointTrajectoryConfig& cfg)
{
    if (!profile_is_valid(cfg.profile) || cfg.waypoint_e_m.empty() || cfg.speed_mps <= 0.0 ||
        cfg.bank_limit_rad <= 0.0 || cfg.acceptance_radius_m <= 0.0) {
        return {};
    }

    GeodeticPosition position = geodetic_from_ecef_m(cfg.profile.p_e_m);
    core::Scalar_t heading_rad = local_rpy_from_attitude(cfg.profile.p_e_m, cfg.profile.q_b2e).z();
    std::size_t target_index = 0U;
    std::vector<TruthSample> samples{};

    for (core::SampleIndex index = 0U;; ++index) {
        core::Timestamp t{};
        core::Time_t elapsed_s{};
        if (!append_timestamp(cfg.profile, index, t, elapsed_s)) {
            break;
        }

        const core::Vec3 p_e_m = ecef_from_geodetic(position);
        const core::Mat3 C_e2n = ecef_to_ned_matrix(position);
        const core::Vec3 target_n_m = C_e2n * (cfg.waypoint_e_m.at(target_index) - p_e_m);
        const core::Scalar_t target_distance_m = target_n_m.head<2>().norm();
        if (target_distance_m <= cfg.acceptance_radius_m &&
            target_index + 1U < cfg.waypoint_e_m.size()) {
            ++target_index;
        }

        const core::Vec3 active_target_n_m = C_e2n * (cfg.waypoint_e_m.at(target_index) - p_e_m);
        const core::Scalar_t desired_heading_rad =
            std::atan2(active_target_n_m.y(), active_target_n_m.x());
        const core::Scalar_t heading_error_rad = wrap_angle_rad(desired_heading_rad - heading_rad);
        const core::Scalar_t bank_rad =
            std::clamp(heading_error_rad, -cfg.bank_limit_rad, cfg.bank_limit_rad);
        const core::Scalar_t heading_rate_radps = (9.80665 * std::tan(bank_rad)) / cfg.speed_mps;
        const core::Vec3 v_n_mps = velocity_n_mps(cfg.speed_mps, 0.0, heading_rad);
        samples.push_back(
            make_local_sample(t, position, core::Vec3{bank_rad, 0.0, heading_rad}, v_n_mps));

        const core::Time_t dt_s = dt_to_next_sample(cfg.profile, index);
        if (dt_s > 0.0 && elapsed_s < cfg.profile.duration_s) {
            const core::Scalar_t heading_delta_rad = heading_rate_radps * dt_s;
            if (std::abs(heading_delta_rad) >= std::abs(heading_error_rad)) {
                heading_rad = desired_heading_rad;
            }
            else {
                heading_rad = wrap_angle_rad(heading_rad + heading_delta_rad);
            }
            integrate_geodetic(v_n_mps, dt_s, position);
        }
    }
    return from_samples(std::move(samples));
}

} // namespace navkit::sim
