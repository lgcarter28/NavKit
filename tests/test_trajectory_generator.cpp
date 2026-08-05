// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/sim/trajectory/GeneratedTrajectorySource.hpp"
#include "navkit/sim/trajectory/StationaryTrajectorySource.hpp"
#include "navkit/sim/trajectory/TrajectoryProfiles.hpp"
#include "test_main.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <numeric>

namespace navkit::sim::test
{

using navkit::core::RationalRate;
using navkit::core::Timestamp;
using navkit::core::timestamp_at_sample_index;
using navkit::core::timestamp_seconds;
using navkit::core::Vec3;

namespace
{

class RecordingGuidance final : public GuidanceModel
{
public:
    explicit RecordingGuidance(std::vector<core::Time_t>& observation_times_s)
        : m_observation_times_s(&observation_times_s)
    {}

    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override
    {
        return core::timestamp_is_valid(initial_state.t) && environment.p_e_m.allFinite();
    }

    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const core::Time_t dt_s,
                               GuidanceOutput& output) override
    {
        if (!std::isfinite(dt_s) || dt_s <= 0.0 || !environment.p_e_m.allFinite()) {
            return false;
        }
        const core::Time_t observation_time_s = core::timestamp_seconds(state.t);
        m_observation_times_s->push_back(observation_time_s);
        output = {};
        output.diagnostics.a_cmd_i_mps2.x() = observation_time_s;
        output.execution.pad_constraint_active = true;
        output.execution.autopilot_active = true;
        return true;
    }

private:
    std::vector<core::Time_t>* m_observation_times_s{};
};

class RecordingAutopilot final : public AutopilotModel
{
public:
    explicit RecordingAutopilot(std::vector<core::Time_t>& observation_times_s)
        : m_observation_times_s(&observation_times_s)
    {}

    [[nodiscard]] bool initialize(const AutopilotState& initial_state) override
    {
        m_observation_time_s = 0.0;
        return initial_state.q_b2i.coeffs().allFinite();
    }

    [[nodiscard]] bool
    observe_imu_increment(const core::estimation::ImuIncrement& increment) override
    {
        return increment.dt_s > 0.0;
    }

    [[nodiscard]] bool advance(const GuidanceCommand& guidance,
                               const AutopilotState& state,
                               const AutopilotExecutionState& execution,
                               const core::Time_t dt_s,
                               VehicleCommand& command,
                               AutopilotOutput& output) override
    {
        if (!std::isfinite(dt_s) || dt_s <= 0.0 || !state.q_b2i.coeffs().allFinite()) {
            return false;
        }
        m_observation_times_s->push_back(m_observation_time_s);
        m_observation_time_s += dt_s;
        output = {};
        output.q_command_b2i = state.q_b2i;
        output.active = execution.active;
        command = {};
        command.w_command_ib_b_radps = state.w_ib_b_radps;
        command.specific_force_command_ib_b_mps2 = guidance.specific_force_command_ib_b_mps2;
        return true;
    }

private:
    std::vector<core::Time_t>* m_observation_times_s{};
    core::Time_t m_observation_time_s{};
};

class ControlStateGuidance final : public GuidanceModel
{
public:
    explicit ControlStateGuidance(core::Vec3& observed_body_x_i)
        : m_observed_body_x_i(&observed_body_x_i)
    {}

    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override
    {
        *m_observed_body_x_i = initial_state.q_b2i * core::Vec3::UnitX();
        return core::timestamp_is_valid(initial_state.t) && environment.gravity_i_mps2.allFinite();
    }

    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const core::Time_t dt_s,
                               GuidanceOutput& output) override
    {
        if (!std::isfinite(dt_s) || dt_s <= 0.0 || !environment.gravity_i_mps2.allFinite()) {
            return false;
        }
        *m_observed_body_x_i = state.q_b2i * core::Vec3::UnitX();
        output = {};
        output.diagnostics.a_cmd_i_mps2 = environment.gravity_i_mps2 + core::Vec3::UnitX();
        output.execution.guidance_active = true;
        return true;
    }

private:
    core::Vec3* m_observed_body_x_i{};
};

} // namespace

TEST_CASE("Stationary trajectory source makes planned truth available without eager samples")
{
    StationaryTrajectoryConfig config;
    config.duration_s = 2.0;
    config.rate = RationalRate{.samples = 2U, .s = 1U};
    config.p_e << 1.0, 2.0, 3.0;

    StationaryTrajectorySource trajectory(config);
    TruthSample sample{};

    CHECK(timestamp_seconds(trajectory.t_start()) == doctest::Approx(0.0));
    CHECK(timestamp_seconds(trajectory.t_end()) == doctest::Approx(2.0));
    CHECK_FALSE(trajectory.query(Timestamp{}, sample));

    REQUIRE(trajectory.advance_to(Timestamp{}));
    REQUIRE(trajectory.query(Timestamp{}, sample));
    CHECK(sample.p_e.isApprox(config.p_e));
    CHECK(sample.v_e.isZero());
    CHECK(sample.q_b2e.isApprox(Eigen::Quaterniond::Identity()));

    const Timestamp half_second{.ns = 500'000'000U};
    REQUIRE(trajectory.advance_to(half_second));
    REQUIRE(trajectory.query(half_second, sample));
    CHECK(timestamp_seconds(sample.t) == doctest::Approx(0.5));
    CHECK_FALSE(trajectory.query(Timestamp{.s = 1U}, sample));
    CHECK_FALSE(trajectory.is_complete());

    REQUIRE(trajectory.advance_to(Timestamp{.s = 2U}));
    CHECK(trajectory.is_complete());
    CHECK_FALSE(trajectory.advance_to(Timestamp{.s = 3U}));
}

TEST_CASE(
    "Truth trajectory interpolates ECEF states and quaternion attitude at arbitrary timestamps")
{
    TruthSample first{};
    first.t = Timestamp{};
    first.p_e = Vec3{1.0, 2.0, 3.0};
    first.v_e = Vec3{4.0, 5.0, 6.0};
    first.q_b2e = Eigen::Quaterniond::Identity();
    first.w_ib_b_radps = Vec3{1.0, 2.0, 3.0};

    TruthSample second{};
    second.t = Timestamp{.s = 2U};
    second.p_e = Vec3{5.0, 6.0, 7.0};
    second.v_e = Vec3{8.0, 9.0, 10.0};
    second.q_b2e =
        core::math::quaternion_from_rpy_rad(Vec3{0.0, 0.0, std::numbers::pi_v<core::Scalar_t>});
    second.w_ib_b_radps = Vec3{5.0, 6.0, 7.0};

    TruthTrajectory trajectory{std::vector<TruthSample>{first, second}};
    TruthSample midpoint{};
    REQUIRE(trajectory.sample_at(Timestamp{.s = 1U}, midpoint));
    CHECK(midpoint.p_e.isApprox(Vec3{3.0, 4.0, 5.0}));
    CHECK(midpoint.v_e.isApprox(Vec3{6.0, 7.0, 8.0}));
    CHECK(midpoint.w_ib_b_radps.isApprox(Vec3{3.0, 4.0, 5.0}));
    CHECK(core::math::rpy_rad_from_quaternion(midpoint.q_b2e).z() ==
          doctest::Approx(std::numbers::pi_v<core::Scalar_t> / 2.0));

    Timestamp imu_timestamp{};
    REQUIRE(core::timestamp_at_sample_index(
        Timestamp{}, core::RationalRate{.samples = 600U, .s = 1U}, 300U, imu_timestamp));
    TruthSample imu_truth{};
    REQUIRE(trajectory.sample_at(imu_timestamp, imu_truth));
    CHECK(core::timestamp_seconds(imu_truth.t) == doctest::Approx(0.5));
    CHECK(imu_truth.p_e.x() == doctest::Approx(2.0));

    TruthSample out_of_range{};
    CHECK_FALSE(trajectory.sample_at(Timestamp{.s = 3U}, out_of_range));
    CHECK_FALSE(
        trajectory.sample_at(Timestamp{.scale = core::TimeScale::Gps, .s = 1U}, out_of_range));
}

TEST_CASE("Truth trajectory uses previous-value hold for commands and interpolates responses")
{
    TruthSample first{};
    first.t = Timestamp{};
    TruthSample second{};
    second.t = Timestamp{.s = 2U};

    TrajectoryDiagnostics first_diagnostics{};
    first_diagnostics.guidance_acceleration_command_i_mps2 = Vec3::Constant(1.0);
    first_diagnostics.guidance_reference_index = 3U;
    first_diagnostics.guidance_state_index = 4U;
    first_diagnostics.autopilot_angular_rate_command_b_radps = Vec3::Constant(2.0);
    first_diagnostics.vehicle_specific_force_command_b_mps2 = Vec3::Constant(3.0);
    first_diagnostics.guidance_acceleration_response_i_mps2 = Vec3::Constant(4.0);
    first_diagnostics.vehicle_specific_force_response_b_mps2 = Vec3::Constant(5.0);

    TrajectoryDiagnostics second_diagnostics{};
    second_diagnostics.guidance_acceleration_command_i_mps2 = Vec3::Constant(11.0);
    second_diagnostics.guidance_reference_index = 7U;
    second_diagnostics.guidance_state_index = 8U;
    second_diagnostics.autopilot_angular_rate_command_b_radps = Vec3::Constant(12.0);
    second_diagnostics.vehicle_specific_force_command_b_mps2 = Vec3::Constant(13.0);
    second_diagnostics.guidance_acceleration_response_i_mps2 = Vec3::Constant(14.0);
    second_diagnostics.vehicle_specific_force_response_b_mps2 = Vec3::Constant(15.0);

    TruthTrajectory trajectory{
        std::vector<TruthSample>{first, second},
        std::vector<TrajectoryDiagnostics>{first_diagnostics, second_diagnostics}};
    TrajectoryDiagnostics midpoint{};
    REQUIRE(trajectory.diagnostics_at(Timestamp{.s = 1U}, midpoint));
    CHECK(midpoint.guidance_acceleration_command_i_mps2.isApprox(Vec3::Constant(1.0)));
    CHECK(midpoint.guidance_reference_index == 3U);
    CHECK(midpoint.guidance_state_index == 4U);
    CHECK(midpoint.autopilot_angular_rate_command_b_radps.isApprox(Vec3::Constant(2.0)));
    CHECK(midpoint.vehicle_specific_force_command_b_mps2.isApprox(Vec3::Constant(3.0)));
    CHECK(midpoint.guidance_acceleration_response_i_mps2.isApprox(Vec3::Constant(9.0)));
    CHECK(midpoint.vehicle_specific_force_response_b_mps2.isApprox(Vec3::Constant(10.0)));
}

TEST_CASE("Generated source evaluates scheduled commands at the causal interval start")
{
    std::vector<core::Time_t> guidance_observation_times_s{};
    std::vector<core::Time_t> autopilot_observation_times_s{};
    FirstOrderVehicleResponseConfig vehicle_config{};
    GeneratedTrajectorySource source{
        RationalRate{.samples = 10U, .s = 1U},
        RationalRate{.samples = 5U, .s = 1U},
        RationalRate{.samples = 10U, .s = 1U},
        Timestamp{},
        Timestamp{.ns = 400'000'000U},
        TranslationalIntegrationMethod::TrapezoidalPredictorCorrector,
        TrajectoryInitialCondition{.p_e_m = Vec3{6378137.0, 0.0, 0.0}},
        std::make_unique<RecordingGuidance>(guidance_observation_times_s),
        std::make_unique<RecordingAutopilot>(autopilot_observation_times_s),
        std::make_unique<FirstOrderVehicleResponseModel>(vehicle_config)};
    REQUIRE(source.initialize());
    REQUIRE(source.advance_to(Timestamp{.ns = 300'000'000U}));

    REQUIRE(guidance_observation_times_s.size() == 2U);
    CHECK(guidance_observation_times_s.at(0U) == doctest::Approx(0.0));
    CHECK(guidance_observation_times_s.at(1U) == doctest::Approx(0.2));
    REQUIRE(autopilot_observation_times_s.size() == 3U);
    CHECK(autopilot_observation_times_s.at(0U) == doctest::Approx(0.0));
    CHECK(autopilot_observation_times_s.at(1U) == doctest::Approx(0.1));
    CHECK(autopilot_observation_times_s.at(2U) == doctest::Approx(0.2));

    TrajectoryDiagnostics at_guidance_epoch{};
    REQUIRE(source.query_diagnostics(Timestamp{.ns = 200'000'000U}, at_guidance_epoch));
    CHECK(at_guidance_epoch.guidance_acceleration_command_i_mps2.x() == doctest::Approx(0.0));
    TrajectoryDiagnostics after_guidance_epoch{};
    REQUIRE(source.query_diagnostics(Timestamp{.ns = 300'000'000U}, after_guidance_epoch));
    CHECK(after_guidance_epoch.guidance_acceleration_command_i_mps2.x() == doctest::Approx(0.2));
}

TEST_CASE("Generated source separates selected control state from physical force realization")
{
    core::Vec3 observed_body_x_i = core::Vec3::Zero();
    std::vector<core::Time_t> autopilot_observation_times_s{};
    FirstOrderVehicleResponseConfig vehicle_config{};
    GeneratedTrajectorySource source{
        RationalRate{.samples = 10U, .s = 1U},
        RationalRate{.samples = 10U, .s = 1U},
        RationalRate{.samples = 10U, .s = 1U},
        Timestamp{},
        Timestamp{.ns = 200'000'000U},
        TranslationalIntegrationMethod::TrapezoidalPredictorCorrector,
        TrajectoryInitialCondition{.p_e_m = Vec3{6378137.0, 0.0, 0.0}},
        std::make_unique<ControlStateGuidance>(observed_body_x_i),
        std::make_unique<RecordingAutopilot>(autopilot_observation_times_s),
        std::make_unique<FirstOrderVehicleResponseModel>(vehicle_config)};
    REQUIRE(source.initialize());

    TrajectoryDiagnostics initial_diagnostics{};
    REQUIRE(source.query_diagnostics(Timestamp{}, initial_diagnostics));
    TrajectoryControlState external_control{};
    external_control.t = Timestamp{};
    external_control.p_i_m = initial_diagnostics.p_i_m;
    external_control.v_i_mps = initial_diagnostics.v_i_mps;
    external_control.a_i_mps2 = initial_diagnostics.a_i_mps2;
    external_control.q_b2i = Eigen::Quaternion<core::Scalar_t>{Eigen::AngleAxis<core::Scalar_t>{
        std::numbers::pi_v<core::Scalar_t> / 2.0, core::Vec3::UnitZ()}};
    REQUIRE(source.set_control_state(external_control));

    CHECK(observed_body_x_i.isApprox(core::Vec3::UnitY(), 1.0e-12));
    REQUIRE(source.query_diagnostics(Timestamp{}, initial_diagnostics));
    const core::Vec3 expected_specific_force_command_b_mps2 =
        initial_diagnostics.q_b2i.conjugate() * core::Vec3::UnitX();
    CHECK(initial_diagnostics.vehicle_specific_force_command_b_mps2.isApprox(
        expected_specific_force_command_b_mps2, 1.0e-12));

    REQUIRE(source.advance_to(Timestamp{.ns = 100'000'000U}));
    TrajectoryDiagnostics first_endpoint{};
    REQUIRE(source.query_diagnostics(Timestamp{.ns = 100'000'000U}, first_endpoint));
    CHECK(first_endpoint.vehicle_specific_force_response_b_mps2.isApprox(
        expected_specific_force_command_b_mps2, 1.0e-12));
}

TEST_CASE("Generated source accepts initial navigation attitude error after physical validation")
{
    core::Vec3 observed_body_x_i = core::Vec3::Zero();
    FirstOrderAutopilotConfig autopilot_config{};
    autopilot_config.initial_velocity_alignment_tolerance_rad = 0.05;
    FirstOrderVehicleResponseConfig vehicle_config{};
    GeneratedTrajectorySource source{
        RationalRate{.samples = 10U, .s = 1U},
        RationalRate{.samples = 10U, .s = 1U},
        RationalRate{.samples = 10U, .s = 1U},
        Timestamp{},
        Timestamp{.ns = 200'000'000U},
        TranslationalIntegrationMethod::TrapezoidalPredictorCorrector,
        TrajectoryInitialCondition{.p_e_m = Vec3{6378137.0, 0.0, 0.0},
                                   .v_e_mps = Vec3{100.0, 0.0, 0.0}},
        std::make_unique<ControlStateGuidance>(observed_body_x_i),
        std::make_unique<FirstOrderAutopilotModel>(autopilot_config),
        std::make_unique<FirstOrderVehicleResponseModel>(vehicle_config)};
    REQUIRE(source.initialize());

    TrajectoryDiagnostics initial_diagnostics{};
    REQUIRE(source.query_diagnostics(Timestamp{}, initial_diagnostics));
    TrajectoryControlState navigation_estimate{};
    navigation_estimate.t = Timestamp{};
    navigation_estimate.p_i_m = initial_diagnostics.p_i_m;
    navigation_estimate.v_i_mps = initial_diagnostics.v_i_mps;
    navigation_estimate.a_i_mps2 = initial_diagnostics.a_i_mps2;
    navigation_estimate.q_b2i = Eigen::Quaternion<core::Scalar_t>{Eigen::AngleAxis<core::Scalar_t>{
        std::numbers::pi_v<core::Scalar_t> / 2.0, core::Vec3::UnitZ()}};
    REQUIRE(source.set_control_state(navigation_estimate));
    REQUIRE(source.advance_to(Timestamp{.ns = 100'000'000U}));
}

TEST_CASE("CSV trajectory source uses the shared truth trajectory contract")
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "navkit_truth_trajectory_source.csv";
    {
        std::ofstream stream(path);
        REQUIRE(stream.good());
        stream << "time_s,p_e_x_m,p_e_y_m,p_e_z_m,v_e_x_mps,v_e_y_mps,v_e_z_mps,q_b2e_w,q_b2e_x,q_"
                  "b2e_y,"
                  "q_b2e_z\n";
        stream << "0,6378137,0,0,0,0,0,1,0,0,0\n";
        stream << "2,6378139,0,0,2,0,0,1,0,0,0\n";
    }

    const nlohmann::json cfg{
        {"trajectory", {{"type", "csv"}, {"csv_path", path.filename().string()}}}};
    app_support::TrajectoryRun run = app_support::trajectory_run_from_json(cfg, path.parent_path());
    REQUIRE(run.source);
    REQUIRE(run.source->advance_to(Timestamp{.s = 1U}));
    TruthSample midpoint{};
    REQUIRE(run.source->query(Timestamp{.s = 1U}, midpoint));
    CHECK(midpoint.p_e.x() == doctest::Approx(6378138.0));
    CHECK(midpoint.v_e.x() == doctest::Approx(1.0));
    CHECK_FALSE(run.source->query(Timestamp{.s = 2U}, midpoint));

    std::filesystem::remove(path);
}

TEST_CASE("Generated ballistic trajectory preserves a launch-pad dwell before boost")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 3.0;
    config.profile.rate = RationalRate{.samples = 10U, .s = 1U};
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    config.launch_pad_duration_s = 1.0;
    config.boost_duration_s = 1.0;
    config.boost_acceleration_b_x_mps2 = 30.0;

    const TruthTrajectory trajectory = ballistic_trajectory(config);
    REQUIRE(trajectory.size() > 20U);
    TruthSample launch_pad_truth{};
    const Timestamp launch_pad_time{.ns = 500'000'000U};
    REQUIRE(trajectory.sample_at(launch_pad_time, launch_pad_truth));
    CHECK(launch_pad_truth.p_e.isApprox(config.profile.p_e_m));
    CHECK((launch_pad_truth.v_e - config.profile.v_e_mps).norm() < 0.05);
    TrajectoryDiagnostics launch_pad_diagnostics{};
    REQUIRE(trajectory.diagnostics_at(launch_pad_time, launch_pad_diagnostics));
    CHECK(launch_pad_diagnostics.pad_constraint_active);
    CHECK_FALSE(launch_pad_diagnostics.guidance_active);
    CHECK_FALSE(launch_pad_diagnostics.autopilot_active);
    core::Mat3 c_e2i{};
    REQUIRE(core::frames::fixed_to_inertial_matrix<core::environment::Wgs84>(0.5, c_e2i));
    const Vec3 gravity_i_mps2 =
        c_e2i * core::environment::J2<core::environment::Wgs84>::acceleration(launch_pad_truth.p_e);
    const Vec3 expected_specific_force_b_mps2 = launch_pad_diagnostics.q_b2i.conjugate() *
                                                (launch_pad_diagnostics.a_i_mps2 - gravity_i_mps2);
    CHECK(launch_pad_diagnostics.specific_force_ib_b_mps2.isApprox(expected_specific_force_b_mps2,
                                                                   1.0e-10));
    CHECK(launch_pad_diagnostics.specific_force_ib_b_mps2.norm() > 9.0);

    TruthSample boosted_truth{};
    REQUIRE(trajectory.sample_at(Timestamp{.s = 2U}, boosted_truth));
    CHECK_FALSE(boosted_truth.p_e.isApprox(config.profile.p_e_m));
    CHECK(boosted_truth.v_e.norm() > 1.0);
}

TEST_CASE("Ballistic release overlaps launch-pad support and boost for one physics step")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 1.2;
    config.profile.rate = RationalRate{.samples = 10U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.profile.vehicle_response.specific_force_command_time_constant_b_s = Vec3::Constant(0.1);
    config.profile.vehicle_response.specific_force_response_time_constant_b_s = Vec3::Constant(0.1);
    config.launch_pad_duration_s = 1.0;
    config.boost_duration_s = 1.0;
    config.boost_acceleration_b_x_mps2 = 30.0;

    const TruthTrajectory trajectory = ballistic_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    TrajectoryDiagnostics handoff{};
    REQUIRE(trajectory.diagnostics_at(Timestamp{.s = 1U, .ns = 100'000'000U}, handoff));
    CHECK(handoff.vehicle_specific_force_response_b_mps2.x() > 0.0);
    TruthSample handoff_truth{};
    REQUIRE(trajectory.sample_at(Timestamp{.s = 1U, .ns = 100'000'000U}, handoff_truth));
    CHECK(handoff_truth.p_e.isApprox(config.profile.p_e_m, 1.0e-8));

    TrajectoryDiagnostics release{};
    REQUIRE(trajectory.diagnostics_at(Timestamp{.s = 1U, .ns = 200'000'000U}, release));
    CHECK(release.vehicle_specific_force_response_b_mps2.x() > 0.0);
    CHECK(std::abs(release.vehicle_specific_force_command_b_mps2.y()) < 1.0e-10);
    CHECK(std::abs(release.vehicle_specific_force_command_b_mps2.z()) < 1.0e-10);
    CHECK(std::abs(release.vehicle_specific_force_response_b_mps2.z()) <
          std::abs(handoff.vehicle_specific_force_response_b_mps2.z()));
}

TEST_CASE("Ballistic trajectory terminates on its first return to launch elevation")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 20.0;
    config.profile.rate = RationalRate{.samples = 20U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    config.profile.q_b2e = Eigen::Quaterniond::Identity();
    config.launch_pad_duration_s = 0.2;
    config.boost_duration_s = 0.8;
    config.boost_acceleration_b_x_mps2 = 30.0;

    const TruthTrajectory trajectory = ballistic_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    CHECK(timestamp_seconds(trajectory.last().t) < config.profile.duration_s);
    const Vec3 final_lla_deg_m = core::frames::ecef_m_to_lla_deg_m(trajectory.last().p_e);
    CHECK(final_lla_deg_m.z() <= 0.5);
    CHECK(final_lla_deg_m.z() > -5.0);
}

TEST_CASE("A coarse ballistic advance clamps availability to discovered ground impact")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 20.0;
    config.profile.rate = RationalRate{.samples = 20U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    config.profile.q_b2e = Eigen::Quaterniond::Identity();
    config.launch_pad_duration_s = 0.2;
    config.boost_duration_s = 0.8;
    config.boost_acceleration_b_x_mps2 = 30.0;

    std::unique_ptr<TrajectorySource> source = ballistic_trajectory_source(config);
    REQUIRE(source);
    REQUIRE(source->advance_to(Timestamp{.s = 20U}));
    CHECK(source->is_complete());
    CHECK(timestamp_seconds(source->t_end()) < config.profile.duration_s);

    TruthSample impact{};
    REQUIRE(source->query(source->t_end(), impact));
    const Vec3 impact_lla_deg_m = core::frames::ecef_m_to_lla_deg_m(impact.p_e);
    CHECK(impact_lla_deg_m.z() <= 0.5);
    CHECK(impact_lla_deg_m.z() > -5.0);
}

TEST_CASE("Ballistic launch program clears the low-speed gravity-turn singularity")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 90.0;
    config.profile.rate = RationalRate{.samples = 100U, .s = 1U};
    config.profile.guidance_rate = RationalRate{.samples = 20U, .s = 1U};
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    const Eigen::Quaterniond q_b2n =
        core::math::quaternion_from_rpy_rad(Vec3{0.0, std::numbers::pi_v<double> / 3.0, 0.0});
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()} * q_b2n;
    config.profile.autopilot.velocity_alignment_speed_threshold_mps = 50.0;
    config.launch_pad_duration_s = 5.0;
    config.boost_duration_s = 10.0;
    config.boost_acceleration_b_x_mps2 = 30.0;

    const TruthTrajectory trajectory = ballistic_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    CHECK(timestamp_seconds(trajectory.last().t) > 30.0);

    core::Scalar_t maximum_altitude_m = 0.0;
    for (const TruthSample& sample : trajectory.samples()) {
        const Vec3 lla_deg_m = core::frames::ecef_m_to_lla_deg_m(sample.p_e);
        maximum_altitude_m = std::max(maximum_altitude_m, lla_deg_m.z());
    }
    CHECK(maximum_altitude_m > 500.0);

    TruthSample coast_ascent{};
    TruthSample coast_descent{};
    REQUIRE(trajectory.sample_at(Timestamp{.s = 20U}, coast_ascent));
    REQUIRE(trajectory.sample_at(Timestamp{.s = 28U}, coast_descent));
    core::Mat3 c_e2n_ascent{};
    core::Mat3 c_e2n_descent{};
    REQUIRE(core::frames::ecef_to_ned_matrix(coast_ascent.p_e, c_e2n_ascent));
    REQUIRE(core::frames::ecef_to_ned_matrix(coast_descent.p_e, c_e2n_descent));
    const core::Scalar_t ascent_pitch_rad =
        core::math::rpy_rad_from_quaternion(Eigen::Quaterniond{c_e2n_ascent} * coast_ascent.q_b2e)
            .y();
    const core::Scalar_t descent_pitch_rad =
        core::math::rpy_rad_from_quaternion(Eigen::Quaterniond{c_e2n_descent} * coast_descent.q_b2e)
            .y();
    CHECK(ascent_pitch_rad > 0.0);
    CHECK(descent_pitch_rad < 0.0);
}

TEST_CASE("Ballistic free-inertial coast removes force and leaves Autopilot inactive")
{
    BallisticTrajectoryConfig config{};
    config.profile.duration_s = 0.5;
    config.profile.rate = RationalRate{.samples = 10U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    config.launch_pad_duration_s = 0.0;
    config.boost_duration_s = 0.1;
    config.boost_acceleration_b_x_mps2 = 30.0;
    config.coast_mode = BallisticCoastMode::FreeInertial;

    const TruthTrajectory trajectory = ballistic_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    TrajectoryDiagnostics coast{};
    REQUIRE(trajectory.diagnostics_at(Timestamp{.ns = 300'000'000U}, coast));
    CHECK(coast.guidance_active);
    CHECK_FALSE(coast.autopilot_active);
    CHECK(coast.vehicle_specific_force_command_b_mps2.isZero(1.0e-10));
    CHECK(coast.vehicle_specific_force_response_b_mps2.isZero(1.0e-10));
}

TEST_CASE("Generated curved-Earth and calibration trajectories produce bounded truth")
{
    TrajectoryProfileConfig profile{};
    profile.duration_s = 5.0;
    profile.rate = RationalRate{.samples = 20U, .s = 1U};
    profile.guidance_rate = profile.rate;
    profile.autopilot_rate = profile.rate;
    profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(profile.p_e_m, c_e2n));
    profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};

    ConstantAltitudeTrajectoryConfig constant_altitude{};
    constant_altitude.profile = profile;
    constant_altitude.speed_mps = 100.0;
    const TruthTrajectory constant_altitude_truth = constant_altitude_trajectory(constant_altitude);
    REQUIRE(constant_altitude_truth.size() > 2U);
    CHECK(constant_altitude_truth.first().p_e.norm() ==
          doctest::Approx(constant_altitude_truth.last().p_e.norm()).epsilon(1.0e-4));
    CHECK(constant_altitude_truth.last().v_e.norm() == doctest::Approx(100.0).epsilon(1.0e-3));
    TrajectoryDiagnostics constant_altitude_diagnostics{};
    REQUIRE(constant_altitude_truth.diagnostics_at(constant_altitude_truth.first().t,
                                                   constant_altitude_diagnostics));
    CHECK(constant_altitude_diagnostics.guidance_acceleration_command_n_mps2.isApprox(
        c_e2n * constant_altitude_diagnostics.guidance_acceleration_command_i_mps2, 1.0e-12));

    CalibrationTrajectoryConfig calibration{};
    calibration.profile = profile;
    calibration.speed_mps = 100.0;
    calibration.horizontal_amplitude_rad = 0.2;
    calibration.period_s = 2.0;
    calibration.maneuver = CalibrationManeuver::HorizontalSTurn;
    const TruthTrajectory calibration_truth = calibration_trajectory(calibration);
    REQUIRE(calibration_truth.size() > 2U);
    CHECK_FALSE(calibration_truth.first().q_b2e.isApprox(calibration_truth.samples().at(5U).q_b2e));
    CHECK(calibration_truth.samples().at(5U).w_ib_b_radps.norm() > 0.0);
}

TEST_CASE("Horizontal calibration exposes distinct skid-to-turn and bank-to-turn realizations")
{
    TrajectoryProfileConfig profile{};
    profile.duration_s = 5.0;
    profile.rate = RationalRate{.samples = 100U, .s = 1U};
    profile.guidance_rate = RationalRate{.samples = 100U, .s = 1U};
    profile.autopilot_rate = profile.rate;
    profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(profile.p_e_m, c_e2n));
    profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    profile.guidance_command_filter.specific_force_time_constant_b_s = Vec3::Constant(0.2);
    profile.guidance_command_filter.bank_time_constant_s = 0.2;

    CalibrationTrajectoryConfig skid_config{};
    skid_config.profile = profile;
    skid_config.maneuver = CalibrationManeuver::HorizontalSTurn;
    skid_config.speed_mps = 100.0;
    skid_config.horizontal_amplitude_rad = 0.3490658504;
    skid_config.period_s = 20.0;

    CalibrationTrajectoryConfig bank_config = skid_config;
    bank_config.bank_to_turn_enabled = true;
    bank_config.body_y_specific_force_enabled = false;

    const TruthTrajectory skid_truth = calibration_trajectory(skid_config);
    const TruthTrajectory bank_truth = calibration_trajectory(bank_config);
    REQUIRE_FALSE(skid_truth.empty());
    REQUIRE_FALSE(bank_truth.empty());

    core::Scalar_t maximum_skid_y_specific_force_mps2{};
    core::Scalar_t maximum_bank_y_specific_force_mps2{};
    core::Scalar_t maximum_bank_z_specific_force_mps2{};
    core::Scalar_t maximum_bank_command_rad{};
    core::Scalar_t maximum_skid_heading_rad{};
    core::Scalar_t maximum_bank_heading_rad{};
    core::Scalar_t maximum_skid_altitude_error_m{};
    core::Scalar_t maximum_bank_altitude_error_m{};
    const core::Scalar_t initial_altitude_m = core::frames::ecef_m_to_lla_deg_m(profile.p_e_m).z();
    for (core::SampleIndex sample_index = 1U; sample_index <= 500U; ++sample_index) {
        core::Timestamp t{};
        REQUIRE(timestamp_at_sample_index(profile.t_epoch, profile.rate, sample_index, t));
        TruthSample skid_sample{};
        TruthSample bank_sample{};
        TrajectoryDiagnostics skid_diagnostics{};
        TrajectoryDiagnostics bank_diagnostics{};
        REQUIRE(skid_truth.sample_at(t, skid_sample));
        REQUIRE(bank_truth.sample_at(t, bank_sample));
        REQUIRE(skid_truth.diagnostics_at(t, skid_diagnostics));
        REQUIRE(bank_truth.diagnostics_at(t, bank_diagnostics));
        const core::Vec3 skid_velocity_n_mps = c_e2n * skid_sample.v_e;
        const core::Vec3 bank_velocity_n_mps = c_e2n * bank_sample.v_e;
        maximum_skid_heading_rad =
            std::max(maximum_skid_heading_rad,
                     std::abs(std::atan2(skid_velocity_n_mps.y(), skid_velocity_n_mps.x())));
        maximum_bank_heading_rad =
            std::max(maximum_bank_heading_rad,
                     std::abs(std::atan2(bank_velocity_n_mps.y(), bank_velocity_n_mps.x())));
        maximum_skid_altitude_error_m = std::max(
            maximum_skid_altitude_error_m,
            std::abs(core::frames::ecef_m_to_lla_deg_m(skid_sample.p_e).z() - initial_altitude_m));
        maximum_bank_altitude_error_m = std::max(
            maximum_bank_altitude_error_m,
            std::abs(core::frames::ecef_m_to_lla_deg_m(bank_sample.p_e).z() - initial_altitude_m));
        maximum_skid_y_specific_force_mps2 =
            std::max(maximum_skid_y_specific_force_mps2,
                     std::abs(skid_diagnostics.vehicle_specific_force_command_b_mps2.y()));
        maximum_bank_y_specific_force_mps2 =
            std::max(maximum_bank_y_specific_force_mps2,
                     std::abs(bank_diagnostics.vehicle_specific_force_command_b_mps2.y()));
        maximum_bank_z_specific_force_mps2 =
            std::max(maximum_bank_z_specific_force_mps2,
                     std::abs(bank_diagnostics.vehicle_specific_force_command_b_mps2.z()));
        maximum_bank_command_rad = std::max(maximum_bank_command_rad,
                                            std::abs(bank_diagnostics.guidance_bank_command_n_rad));
    }

    CHECK(maximum_skid_y_specific_force_mps2 > 1.0);
    CHECK(maximum_bank_y_specific_force_mps2 < 1.0e-12);
    CHECK(maximum_bank_z_specific_force_mps2 > 9.0);
    CHECK(maximum_bank_command_rad > 0.05);
    CHECK(maximum_skid_heading_rad > 0.05);
    CHECK(maximum_bank_heading_rad > 0.05);
    CHECK(std::abs(maximum_skid_heading_rad - maximum_bank_heading_rad) < 0.1);
    CHECK(maximum_skid_altitude_error_m < 10.0);
    CHECK(maximum_bank_altitude_error_m < 10.0);
}

TEST_CASE("Dutch-roll calibration follows a sustained coordinated front-view U path")
{
    CalibrationTrajectoryConfig config{};
    config.profile.duration_s = 60.0;
    config.profile.rate = RationalRate{.samples = 100U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.profile.guidance_command_filter.specific_force_time_constant_b_s = Vec3::Constant(1.0);
    config.profile.guidance_command_filter.bank_time_constant_s = 1.0;
    config.maneuver = CalibrationManeuver::DutchRoll;
    config.speed_mps = 100.0;
    config.horizontal_amplitude_rad = 0.18;
    config.vertical_amplitude_rad = 0.05;
    config.period_s = 20.0;
    config.velocity_error_gain_n_1ps = Vec3{0.5, 0.5, 0.25};
    config.altitude_error_p_gain_1ps2 = 0.1;
    config.altitude_error_d_gain_1ps = 0.1;
    config.bank_to_turn_enabled = true;
    config.body_y_specific_force_enabled = false;

    const TruthTrajectory truth = calibration_trajectory(config);
    REQUIRE_FALSE(truth.empty());
    const core::Scalar_t initial_altitude_m =
        core::frames::ecef_m_to_lla_deg_m(config.profile.p_e_m).z();
    std::vector<core::Scalar_t> lateral_displacement_m{};
    std::vector<core::Scalar_t> altitude_m{};
    std::vector<core::Scalar_t> roll_rate_radps{};
    std::vector<core::Scalar_t> yaw_rate_radps{};
    core::Scalar_t early_yaw_rate_energy{};
    core::Scalar_t late_yaw_rate_energy{};
    core::Scalar_t maximum_altitude_error_m{};
    core::Scalar_t maximum_down_velocity_mps{};
    core::Scalar_t maximum_body_y_specific_force_mps2{};
    core::Scalar_t maximum_bank_command_rad{};
    core::Scalar_t maximum_steady_bank_command_rad{};
    for (core::SampleIndex sample_index = 1U; sample_index <= 6000U; ++sample_index) {
        core::Timestamp t{};
        REQUIRE(timestamp_at_sample_index(
            config.profile.t_epoch, config.profile.rate, sample_index, t));
        TruthSample sample{};
        TrajectoryDiagnostics diagnostics{};
        REQUIRE(truth.sample_at(t, sample));
        REQUIRE(truth.diagnostics_at(t, diagnostics));
        maximum_bank_command_rad =
            std::max(maximum_bank_command_rad, std::abs(diagnostics.guidance_bank_command_n_rad));
        maximum_altitude_error_m = std::max(
            maximum_altitude_error_m,
            std::abs(core::frames::ecef_m_to_lla_deg_m(sample.p_e).z() - initial_altitude_m));
        maximum_down_velocity_mps =
            std::max(maximum_down_velocity_mps, std::abs((c_e2n * sample.v_e).z()));
        maximum_body_y_specific_force_mps2 =
            std::max(maximum_body_y_specific_force_mps2,
                     std::abs(diagnostics.vehicle_specific_force_response_b_mps2.y()));
        const core::Time_t elapsed_s = timestamp_seconds(t);
        if (elapsed_s >= 20.0) {
            maximum_steady_bank_command_rad = std::max(
                maximum_steady_bank_command_rad, std::abs(diagnostics.guidance_bank_command_n_rad));
        }
        if (elapsed_s >= 40.0) {
            const core::Vec3 displacement_n_m = c_e2n * (sample.p_e - config.profile.p_e_m);
            lateral_displacement_m.push_back(displacement_n_m.y());
            altitude_m.push_back(core::frames::ecef_m_to_lla_deg_m(sample.p_e).z());
            roll_rate_radps.push_back(sample.w_ib_b_radps.x());
            yaw_rate_radps.push_back(sample.w_ib_b_radps.z());
        }
        if (elapsed_s >= 20.0 && elapsed_s < 40.0) {
            early_yaw_rate_energy += sample.w_ib_b_radps.z() * sample.w_ib_b_radps.z();
        }
        if (elapsed_s >= 40.0) {
            late_yaw_rate_energy += sample.w_ib_b_radps.z() * sample.w_ib_b_radps.z();
        }
    }

    REQUIRE(lateral_displacement_m.size() == altitude_m.size());
    REQUIRE(roll_rate_radps.size() == yaw_rate_radps.size());
    REQUIRE(roll_rate_radps.size() > 700U);
    const core::Scalar_t mean_lateral_displacement_m =
        std::accumulate(lateral_displacement_m.begin(), lateral_displacement_m.end(), 0.0) /
        static_cast<core::Scalar_t>(lateral_displacement_m.size());
    const core::Scalar_t mean_altitude_m =
        std::accumulate(altitude_m.begin(), altitude_m.end(), 0.0) /
        static_cast<core::Scalar_t>(altitude_m.size());
    core::Scalar_t lateral_squared_mean_m2{};
    for (const core::Scalar_t lateral_m : lateral_displacement_m) {
        const core::Scalar_t centered_lateral_m = lateral_m - mean_lateral_displacement_m;
        lateral_squared_mean_m2 += centered_lateral_m * centered_lateral_m;
    }
    lateral_squared_mean_m2 /= static_cast<core::Scalar_t>(lateral_displacement_m.size());

    core::Scalar_t lateral_altitude_cross_energy{};
    core::Scalar_t lateral_altitude_squared_cross_energy{};
    core::Scalar_t lateral_energy{};
    core::Scalar_t lateral_squared_energy{};
    core::Scalar_t altitude_energy{};
    core::Scalar_t minimum_lateral_displacement_m = lateral_displacement_m.front();
    core::Scalar_t maximum_lateral_displacement_m = lateral_displacement_m.front();
    core::Scalar_t minimum_altitude_m = altitude_m.front();
    core::Scalar_t maximum_altitude_m = altitude_m.front();
    for (std::size_t index = 0U; index < lateral_displacement_m.size(); ++index) {
        const core::Scalar_t centered_lateral_m =
            lateral_displacement_m.at(index) - mean_lateral_displacement_m;
        const core::Scalar_t centered_lateral_squared_m2 =
            (centered_lateral_m * centered_lateral_m) - lateral_squared_mean_m2;
        const core::Scalar_t centered_altitude_m = altitude_m.at(index) - mean_altitude_m;
        lateral_altitude_cross_energy += centered_lateral_m * centered_altitude_m;
        lateral_altitude_squared_cross_energy += centered_lateral_squared_m2 * centered_altitude_m;
        lateral_energy += centered_lateral_m * centered_lateral_m;
        lateral_squared_energy += centered_lateral_squared_m2 * centered_lateral_squared_m2;
        altitude_energy += centered_altitude_m * centered_altitude_m;
        minimum_lateral_displacement_m =
            std::min(minimum_lateral_displacement_m, lateral_displacement_m.at(index));
        maximum_lateral_displacement_m =
            std::max(maximum_lateral_displacement_m, lateral_displacement_m.at(index));
        minimum_altitude_m = std::min(minimum_altitude_m, altitude_m.at(index));
        maximum_altitude_m = std::max(maximum_altitude_m, altitude_m.at(index));
    }
    const core::Scalar_t lateral_altitude_correlation =
        lateral_altitude_cross_energy / std::sqrt(lateral_energy * altitude_energy);
    const core::Scalar_t u_shape_correlation =
        lateral_altitude_squared_cross_energy / std::sqrt(lateral_squared_energy * altitude_energy);

    core::Scalar_t roll_energy{};
    core::Scalar_t yaw_energy{};
    core::Scalar_t zero_lag_cross_energy{};
    core::Scalar_t quarter_cycle_cross_energy{};
    core::Scalar_t quarter_cycle_roll_energy{};
    core::Scalar_t quarter_cycle_yaw_energy{};
    constexpr std::size_t quarter_cycle_samples = 500U;
    for (std::size_t index = 0U; index < roll_rate_radps.size(); ++index) {
        roll_energy += roll_rate_radps.at(index) * roll_rate_radps.at(index);
        yaw_energy += yaw_rate_radps.at(index) * yaw_rate_radps.at(index);
        zero_lag_cross_energy += roll_rate_radps.at(index) * yaw_rate_radps.at(index);
        if (index + quarter_cycle_samples < yaw_rate_radps.size()) {
            quarter_cycle_cross_energy +=
                roll_rate_radps.at(index) * yaw_rate_radps.at(index + quarter_cycle_samples);
            quarter_cycle_roll_energy += roll_rate_radps.at(index) * roll_rate_radps.at(index);
            quarter_cycle_yaw_energy += yaw_rate_radps.at(index + quarter_cycle_samples) *
                                        yaw_rate_radps.at(index + quarter_cycle_samples);
        }
    }
    const core::Scalar_t zero_lag_correlation =
        zero_lag_cross_energy / std::sqrt(roll_energy * yaw_energy);
    const core::Scalar_t quarter_cycle_correlation =
        quarter_cycle_cross_energy /
        std::sqrt(quarter_cycle_roll_energy * quarter_cycle_yaw_energy);

    CAPTURE(lateral_altitude_correlation);
    CAPTURE(u_shape_correlation);
    CAPTURE(minimum_lateral_displacement_m);
    CAPTURE(maximum_lateral_displacement_m);
    CAPTURE(minimum_altitude_m);
    CAPTURE(maximum_altitude_m);
    CAPTURE(mean_altitude_m);
    CAPTURE(zero_lag_correlation);
    CAPTURE(quarter_cycle_correlation);
    CAPTURE(maximum_altitude_error_m);
    CAPTURE(maximum_down_velocity_mps);
    CAPTURE(maximum_body_y_specific_force_mps2);
    CHECK(maximum_lateral_displacement_m - minimum_lateral_displacement_m > 25.0);
    CHECK(maximum_altitude_m - minimum_altitude_m > 5.0);
    CHECK(u_shape_correlation > 0.6);
    CHECK(std::abs(lateral_altitude_correlation) < 0.35);
    CHECK(std::abs(mean_altitude_m - initial_altitude_m) < 10.0);
    CHECK(std::sqrt(roll_energy / static_cast<core::Scalar_t>(roll_rate_radps.size())) > 0.01);
    CHECK(std::sqrt(yaw_energy / static_cast<core::Scalar_t>(yaw_rate_radps.size())) > 0.005);
    CHECK(std::abs(zero_lag_correlation) < 0.35);
    CHECK(std::abs(quarter_cycle_correlation) > 0.4);
    CHECK(late_yaw_rate_energy > 0.5 * early_yaw_rate_energy);
    CHECK(maximum_bank_command_rad > 0.05);
    CHECK(maximum_steady_bank_command_rad < 0.95 * config.profile.maximum_bank_angle_rad);
    CHECK(maximum_altitude_error_m < 50.0);
    CHECK(maximum_down_velocity_mps < 20.0);
    CHECK(maximum_body_y_specific_force_mps2 < 1.0e-10);
}

TEST_CASE("Vertical calibration remains in its commanded vertical plane")
{
    CalibrationTrajectoryConfig config{};
    config.profile.duration_s = 60.0;
    config.profile.rate = RationalRate{.samples = 100U, .s = 1U};
    config.profile.guidance_rate = RationalRate{.samples = 50U, .s = 1U};
    config.profile.autopilot_rate = config.profile.rate;
    REQUIRE(core::frames::lla_deg_m_to_ecef_m(Vec3{35.0, -106.0, 1500.0}, config.profile.p_e_m));
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.profile.guidance_command_filter.specific_force_time_constant_b_s = Vec3::Constant(0.2);
    config.profile.guidance_command_filter.bank_time_constant_s = 0.2;
    config.maneuver = CalibrationManeuver::VerticalSTurn;
    config.speed_mps = 100.0;
    config.vertical_amplitude_rad = 10.0 * std::numbers::pi_v<core::Scalar_t> / 180.0;
    config.period_s = 20.0;
    config.velocity_error_gain_n_1ps = Vec3{0.5, 0.5, 0.75};

    const TruthTrajectory truth = calibration_trajectory(config);
    REQUIRE_FALSE(truth.empty());
    core::Scalar_t maximum_cross_track_m{};
    core::Scalar_t maximum_cross_track_velocity_mps{};
    for (const TruthSample& sample : truth.samples()) {
        const core::Vec3 displacement_n_m = c_e2n * (sample.p_e - config.profile.p_e_m);
        const core::Vec3 velocity_n_mps = c_e2n * sample.v_e;
        maximum_cross_track_m = std::max(maximum_cross_track_m, std::abs(displacement_n_m.y()));
        maximum_cross_track_velocity_mps =
            std::max(maximum_cross_track_velocity_mps, std::abs(velocity_n_mps.y()));
    }
    CHECK(maximum_cross_track_m < 5.0);
    CHECK(maximum_cross_track_velocity_mps < 0.25);
}

TEST_CASE("Generated dynamic profiles preserve an explicitly configured initial velocity")
{
    TrajectoryProfileConfig profile{};
    profile.duration_s = 1.0;
    profile.rate = RationalRate{.samples = 20U, .s = 1U};
    profile.guidance_rate = profile.rate;
    profile.autopilot_rate = profile.rate;
    profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(profile.p_e_m, c_e2n));
    profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    profile.v_e_mps = c_e2n.transpose() * (42.0 * Vec3::UnitX());
    profile.initial_velocity_configured = true;

    ConstantAltitudeTrajectoryConfig config{};
    config.profile = profile;
    config.speed_mps = 100.0;
    const TruthTrajectory trajectory = constant_altitude_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    CHECK(trajectory.first().v_e.isApprox(profile.v_e_mps, 1.0e-12));
}

TEST_CASE("Constant-altitude derivative gain damps positive NED Down velocity")
{
    ConstantAltitudeTrajectoryConfig undamped_config{};
    undamped_config.profile.duration_s = 0.1;
    undamped_config.profile.rate = RationalRate{.samples = 100U, .s = 1U};
    undamped_config.profile.guidance_rate = undamped_config.profile.rate;
    undamped_config.profile.autopilot_rate = undamped_config.profile.rate;
    undamped_config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(undamped_config.profile.p_e_m, c_e2n));
    undamped_config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    constexpr core::Scalar_t initial_down_velocity_mps = 5.0;
    undamped_config.profile.v_e_mps =
        c_e2n.transpose() * Vec3{100.0, 0.0, initial_down_velocity_mps};
    undamped_config.profile.initial_velocity_configured = true;
    undamped_config.speed_mps = 100.0;
    undamped_config.velocity_error_gain_n_1ps = Vec3::Zero();
    undamped_config.altitude_error_p_gain_1ps2 = 0.0;
    undamped_config.altitude_error_d_gain_1ps = 0.0;

    ConstantAltitudeTrajectoryConfig damped_config = undamped_config;
    constexpr core::Scalar_t derivative_gain_1ps = 0.2;
    damped_config.altitude_error_d_gain_1ps = derivative_gain_1ps;

    const TruthTrajectory undamped = constant_altitude_trajectory(undamped_config);
    const TruthTrajectory damped = constant_altitude_trajectory(damped_config);
    REQUIRE_FALSE(undamped.empty());
    REQUIRE_FALSE(damped.empty());
    TrajectoryDiagnostics undamped_diagnostics{};
    TrajectoryDiagnostics damped_diagnostics{};
    REQUIRE(undamped.diagnostics_at(undamped.first().t, undamped_diagnostics));
    REQUIRE(damped.diagnostics_at(damped.first().t, damped_diagnostics));

    const core::Scalar_t expected_down_acceleration_difference_mps2 =
        -derivative_gain_1ps * initial_down_velocity_mps;
    CHECK(damped_diagnostics.guidance_acceleration_command_n_mps2.z() -
              undamped_diagnostics.guidance_acceleration_command_n_mps2.z() ==
          doctest::Approx(expected_down_acceleration_difference_mps2).epsilon(1.0e-10));
}

TEST_CASE("Generated trajectory bank limit accepts sixty degrees and rejects larger values")
{
    ConstantAltitudeTrajectoryConfig config{};
    config.profile.duration_s = 0.1;
    config.profile.rate = RationalRate{.samples = 100U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.profile.maximum_bank_angle_rad = std::numbers::pi_v<core::Scalar_t> / 3.0;

    CHECK_FALSE(constant_altitude_trajectory(config).empty());
    config.profile.maximum_bank_angle_rad = std::nextafter(
        config.profile.maximum_bank_angle_rad, std::numeric_limits<core::Scalar_t>::infinity());
    CHECK(constant_altitude_trajectory(config).empty());
}

TEST_CASE("Generated source realizes truth incrementally through planned time")
{
    ConstantAltitudeTrajectoryConfig config{};
    config.profile.duration_s = 2.0;
    config.profile.rate = RationalRate{.samples = 10U, .s = 1U};
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.speed_mps = 100.0;

    std::unique_ptr<TrajectorySource> source = constant_altitude_trajectory_source(config);
    REQUIRE(source);
    CHECK(dynamic_cast<GeneratedTrajectorySource*>(source.get()) != nullptr);

    TruthSample sample{};
    REQUIRE(source->advance_to(Timestamp{.ns = 150'000'000U}));
    REQUIRE(source->query(Timestamp{.ns = 150'000'000U}, sample));
    TrajectoryDiagnostics diagnostics{};
    REQUIRE(source->query_diagnostics(Timestamp{.ns = 150'000'000U}, diagnostics));
    CHECK(diagnostics.velocity_tracking_error_b_mps.allFinite());
    CHECK(diagnostics.acceleration_tracking_error_b_mps2.allFinite());
    CHECK(diagnostics.attitude_tracking_error_b_rad.allFinite());
    CHECK(diagnostics.angular_rate_tracking_error_b_radps.allFinite());
    CHECK(diagnostics.specific_force_tracking_error_b_mps2.allFinite());
    CHECK(diagnostics.guidance_velocity_reference_i_mps.allFinite());
    CHECK(diagnostics.guidance_acceleration_command_i_mps2.allFinite());
    CHECK((diagnostics.angular_rate_tracking_error_b_radps -
           (diagnostics.autopilot_angular_rate_command_b_radps - diagnostics.w_ib_b_radps))
              .norm() < 1.0e-12);
    CHECK(
        (diagnostics.specific_force_tracking_error_b_mps2 -
         (diagnostics.vehicle_specific_force_command_b_mps2 - diagnostics.specific_force_ib_b_mps2))
            .norm() < 1.0e-12);
    CHECK(diagnostics.velocity_tracking_error_b_mps.norm() < 50.0);
    CHECK(diagnostics.attitude_tracking_error_b_rad.norm() < 0.5);
    CHECK_FALSE(source->query(Timestamp{.ns = 200'000'000U}, sample));
    CHECK_FALSE(source->is_complete());

    REQUIRE(source->advance_to(Timestamp{.s = 2U}));
    REQUIRE(source->query(Timestamp{.s = 2U}, sample));
    CHECK(source->is_complete());
}

TEST_CASE("Generated waypoint trajectory turns toward configured local-level waypoints")
{
    WaypointTrajectoryConfig config{};
    config.profile.duration_s = 20.0;
    config.profile.rate = RationalRate{.samples = 20U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.speed_mps = 100.0;
    config.profile.maximum_bank_angle_rad = 0.3;
    config.acceptance_radius_m = 10.0;
    config.waypoint_e_m = {
        Vec3{6378137.0, 1000.0, 0.0},
        Vec3{6378137.0, 1000.0, 1000.0},
    };

    const TruthTrajectory trajectory = waypoint_trajectory(config);
    REQUIRE(trajectory.size() > 2U);
    CHECK_FALSE(trajectory.first().p_e.isApprox(trajectory.last().p_e));
    CHECK(trajectory.last().v_e.norm() == doctest::Approx(config.speed_mps).epsilon(0.03));
}

TEST_CASE("Generated waypoint trajectory continues its terminal leg after final acceptance")
{
    WaypointTrajectoryConfig config{};
    config.profile.duration_s = 30.0;
    config.profile.rate = RationalRate{.samples = 20U, .s = 1U};
    config.profile.guidance_rate = config.profile.rate;
    config.profile.autopilot_rate = config.profile.rate;
    config.profile.p_e_m = Vec3{6378137.0, 0.0, 0.0};
    core::Mat3 c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(config.profile.p_e_m, c_e2n));
    config.profile.q_b2e = Eigen::Quaterniond{c_e2n.transpose()};
    config.speed_mps = 100.0;
    config.profile.maximum_bank_angle_rad = 0.3;
    config.acceptance_radius_m = 20.0;
    config.waypoint_e_m = {config.profile.p_e_m + (c_e2n.transpose() * Vec3{500.0, 0.0, 0.0})};

    const TruthTrajectory trajectory = waypoint_trajectory(config);
    REQUIRE_FALSE(trajectory.empty());
    core::Mat3 final_c_e2n{};
    REQUIRE(core::frames::ecef_to_ned_matrix(trajectory.last().p_e, final_c_e2n));
    const Vec3 final_velocity_n_mps = final_c_e2n * trajectory.last().v_e;
    CHECK(final_velocity_n_mps.x() > 90.0);
    CHECK(std::abs(final_velocity_n_mps.y()) < 10.0);
    CHECK(final_velocity_n_mps.norm() == doctest::Approx(config.speed_mps).epsilon(0.05));
}

} // namespace navkit::sim::test
