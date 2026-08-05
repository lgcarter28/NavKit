// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/RationalRate.hpp"
#include "navkit/sim/autopilot/AutopilotModel.hpp"
#include "navkit/sim/guidance/GuidanceModel.hpp"
#include "navkit/sim/trajectory/TrajectoryIntegration.hpp"
#include "navkit/sim/trajectory/TrajectorySource.hpp"
#include "navkit/sim/trajectory/TruthTrajectory.hpp"
#include "navkit/sim/trajectory/VehicleResponseModel.hpp"

#include <memory>

namespace navkit::sim
{

/** Fixed-frame initial condition retained for launch-pad constraint realization. */
struct TrajectoryInitialCondition
{
    core::Vec3 p_e_m{core::Vec3::Zero()};
    core::Vec3 v_e_mps{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
};

enum class TrajectoryTerminationMode
{
    ConfiguredDuration,
    GroundImpact,
};

/**
 * Incrementally realizes generated profile commands through control and vehicle dynamics.
 *
 * The source owns the profile generator and response models. It generates only enough
 * native-rate truth to answer queries through the latest planned advance_to() request.
 */
class GeneratedTrajectorySource final : public TrajectorySource
{
public:
    GeneratedTrajectorySource(
        core::RationalRate rate,
        core::RationalRate guidance_rate,
        core::RationalRate autopilot_rate,
        core::Timestamp t_start,
        core::Timestamp t_end,
        TranslationalIntegrationMethod integration_method,
        TrajectoryInitialCondition initial_condition,
        std::unique_ptr<GuidanceModel> guidance,
        std::unique_ptr<AutopilotModel> autopilot,
        std::unique_ptr<VehicleResponseModel> vehicle_response,
        TrajectoryTerminationMode termination_mode = TrajectoryTerminationMode::ConfiguredDuration);
    ~GeneratedTrajectorySource() override;

    GeneratedTrajectorySource(const GeneratedTrajectorySource&) = delete;
    GeneratedTrajectorySource& operator=(const GeneratedTrajectorySource&) = delete;
    GeneratedTrajectorySource(GeneratedTrajectorySource&&) noexcept;
    GeneratedTrajectorySource& operator=(GeneratedTrajectorySource&&) noexcept;

    /** Initializes the first native truth state and persistent response-model state. */
    [[nodiscard]] bool initialize();

    [[nodiscard]] bool advance_to(const core::Timestamp& t) override;
    [[nodiscard]] bool query(const core::Timestamp& t, TruthSample& sample) const override;
    [[nodiscard]] bool query_diagnostics(const core::Timestamp& t,
                                         TrajectoryDiagnostics& diagnostics) const override;
    [[nodiscard]] bool set_control_state(const TrajectoryControlState& state) override;
    [[nodiscard]] bool
    observe_imu_increment(const core::estimation::ImuIncrement& increment) override;
    [[nodiscard]] core::Timestamp t_start() const override;
    [[nodiscard]] core::Timestamp t_end() const override;
    [[nodiscard]] bool is_complete() const override;

private:
    class Implementation;
    std::unique_ptr<Implementation> m_impl;
};

} // namespace navkit::sim
