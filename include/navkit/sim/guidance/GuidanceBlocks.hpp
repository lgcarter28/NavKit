// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/sim/guidance/GuidanceAlgorithms.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace navkit::sim
{

/** Output of one primary translation-reference block. */
struct GuidanceReferenceOutput
{
    guidance::KinematicReference kinematics{};
    core::Vec3 reference_position_e_m{core::Vec3::Zero()};
    core::Vec3 additional_feedforward_n_mps2{core::Vec3::Zero()};
    std::size_t reference_index{};
    bool reference_position_valid{false};
};

enum class GuidanceReferenceRole
{
    CurrentState,
    Kinematic,
};

/** Typed primary reference boundary used once by a Guidance state. */
class GuidanceReferenceModel
{
public:
    virtual ~GuidanceReferenceModel() = default;

    [[nodiscard]] virtual GuidanceReferenceRole role() const = 0;
    [[nodiscard]] virtual bool config_is_valid() const = 0;
    [[nodiscard]] virtual bool initialize(const TrajectoryControlState& initial_state,
                                          const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool enter(const TrajectoryControlState& state,
                                     const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool advance(const TrajectoryControlState& state,
                                       const TrajectoryEnvironment& environment,
                                       core::Time_t elapsed_in_state_s,
                                       core::Time_t dt_s,
                                       GuidanceReferenceOutput& output) = 0;
};

/** Current-state reference used by launch-pad, boost, and free-fall states. */
class CurrentStateGuidanceReference final : public GuidanceReferenceModel
{
public:
    [[nodiscard]] GuidanceReferenceRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               GuidanceReferenceOutput& output) override;
};

/** Typed modifier applied after the primary reference and before acceleration feedback. */
class GuidanceReferenceModifier
{
public:
    virtual ~GuidanceReferenceModifier() = default;

    [[nodiscard]] virtual bool config_is_valid() const = 0;
    [[nodiscard]] virtual bool initialize(const TrajectoryControlState& initial_state,
                                          const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool enter(const TrajectoryControlState& state,
                                     const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool apply(const TrajectoryControlState& state,
                                     const TrajectoryEnvironment& environment,
                                     core::Time_t elapsed_in_state_s,
                                     core::Time_t dt_s,
                                     GuidanceReferenceOutput& reference) = 0;
};

enum class GuidanceAccelerationRole
{
    AdditiveVelocityDerivative,
    DirectBodySpecificForce,
};

/** Typed acceleration contribution evaluated after the complete reference is known. */
class GuidanceAccelerationModel
{
public:
    virtual ~GuidanceAccelerationModel() = default;

    [[nodiscard]] virtual GuidanceAccelerationRole role() const = 0;
    [[nodiscard]] virtual bool config_is_valid() const = 0;
    [[nodiscard]] virtual bool initialize(const TrajectoryControlState& initial_state,
                                          const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool enter(const TrajectoryControlState& state,
                                     const TrajectoryEnvironment& environment) = 0;
    [[nodiscard]] virtual bool advance(const TrajectoryControlState& state,
                                       const TrajectoryEnvironment& environment,
                                       const GuidanceReferenceOutput& reference,
                                       core::Time_t elapsed_in_state_s,
                                       core::Time_t dt_s,
                                       core::Vec3& contribution) = 0;
};

/** Typed bank-reference policy, exactly one of which is selected by an active state. */
class GuidanceBankPolicy
{
public:
    virtual ~GuidanceBankPolicy() = default;

    [[nodiscard]] virtual bool config_is_valid() const = 0;
    [[nodiscard]] virtual bool advance(const GuidanceReferenceOutput& reference,
                                       const core::Vec3& acceleration_command_i_mps2,
                                       const TrajectoryEnvironment& environment,
                                       core::Scalar_t& bank_command_n_rad) const = 0;
    [[nodiscard]] virtual bool bank_to_turn_active() const = 0;
};

struct ConstantSpeedGuidanceReferenceConfig
{
    core::Scalar_t speed_mps{};
    core::Scalar_t heading_rad{};
    core::Scalar_t pitch_rad{};
    bool capture_initial_heading{true};
    bool capture_initial_pitch{true};
};

class ConstantSpeedGuidanceReference final : public GuidanceReferenceModel
{
public:
    explicit ConstantSpeedGuidanceReference(ConstantSpeedGuidanceReferenceConfig config);

    [[nodiscard]] GuidanceReferenceRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               GuidanceReferenceOutput& output) override;

private:
    ConstantSpeedGuidanceReferenceConfig m_config{};
    core::Scalar_t m_heading_rad{};
    core::Scalar_t m_pitch_rad{};
    bool m_initialized{false};
};

struct WaypointGuidanceReferenceConfig
{
    std::vector<core::Vec3> waypoint_e_m{};
    core::Scalar_t speed_mps{};
    core::Scalar_t acceptance_radius_m{};
    core::Scalar_t heading_error_gain_mps2_per_rad{};
};

class WaypointGuidanceReference final : public GuidanceReferenceModel
{
public:
    explicit WaypointGuidanceReference(WaypointGuidanceReferenceConfig config);

    [[nodiscard]] GuidanceReferenceRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               GuidanceReferenceOutput& output) override;

private:
    WaypointGuidanceReferenceConfig m_config{};
    std::size_t m_waypoint_index{};
    core::Scalar_t m_terminal_heading_rad{};
    bool m_route_complete{false};
    bool m_initialized{false};
};

struct SinusoidalGuidanceReferenceModifierConfig
{
    core::Scalar_t amplitude_rad{};
    core::Scalar_t frequency_hz{};
    core::Scalar_t phase_rad{};
};

class HorizontalSinusoidalGuidanceReferenceModifier final : public GuidanceReferenceModifier
{
public:
    explicit HorizontalSinusoidalGuidanceReferenceModifier(
        SinusoidalGuidanceReferenceModifierConfig config);

    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool apply(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment,
                             core::Time_t elapsed_in_state_s,
                             core::Time_t dt_s,
                             GuidanceReferenceOutput& reference) override;

private:
    SinusoidalGuidanceReferenceModifierConfig m_config{};
};

class VerticalSinusoidalGuidanceReferenceModifier final : public GuidanceReferenceModifier
{
public:
    explicit VerticalSinusoidalGuidanceReferenceModifier(
        SinusoidalGuidanceReferenceModifierConfig config);

    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool apply(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment,
                             core::Time_t elapsed_in_state_s,
                             core::Time_t dt_s,
                             GuidanceReferenceOutput& reference) override;

private:
    SinusoidalGuidanceReferenceModifierConfig m_config{};
};

struct VelocityTrackingGuidanceAccelerationConfig
{
    core::Vec3 gain_n_1ps{core::Vec3::Zero()};
};

/** Adds the analytic derivative of the configured local flight path. */
class PathFeedforwardGuidanceAcceleration final : public GuidanceAccelerationModel
{
public:
    [[nodiscard]] GuidanceAccelerationRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const GuidanceReferenceOutput& reference,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               core::Vec3& contribution) override;
};

class VelocityTrackingGuidanceAcceleration final : public GuidanceAccelerationModel
{
public:
    explicit VelocityTrackingGuidanceAcceleration(
        VelocityTrackingGuidanceAccelerationConfig config);

    [[nodiscard]] GuidanceAccelerationRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const GuidanceReferenceOutput& reference,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               core::Vec3& contribution) override;

private:
    VelocityTrackingGuidanceAccelerationConfig m_config{};
};

struct AltitudeHoldPdGuidanceAccelerationConfig
{
    core::Scalar_t target_altitude_m{};
    core::Scalar_t proportional_gain_1ps2{};
    core::Scalar_t derivative_gain_1ps{};
    bool capture_initial_altitude{true};
};

class AltitudeHoldPdGuidanceAcceleration final : public GuidanceAccelerationModel
{
public:
    explicit AltitudeHoldPdGuidanceAcceleration(AltitudeHoldPdGuidanceAccelerationConfig config);

    [[nodiscard]] GuidanceAccelerationRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const GuidanceReferenceOutput& reference,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               core::Vec3& contribution) override;

private:
    AltitudeHoldPdGuidanceAccelerationConfig m_config{};
    core::Scalar_t m_target_altitude_m{};
    bool m_initialized{false};
};

struct BodySpecificForceGuidanceAccelerationConfig
{
    core::Vec3 specific_force_ib_b_mps2{core::Vec3::Zero()};
};

class BodySpecificForceGuidanceAcceleration final : public GuidanceAccelerationModel
{
public:
    explicit BodySpecificForceGuidanceAcceleration(
        BodySpecificForceGuidanceAccelerationConfig config);

    [[nodiscard]] GuidanceAccelerationRole role() const override;
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool initialize(const TrajectoryControlState& initial_state,
                                  const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool enter(const TrajectoryControlState& state,
                             const TrajectoryEnvironment& environment) override;
    [[nodiscard]] bool advance(const TrajectoryControlState& state,
                               const TrajectoryEnvironment& environment,
                               const GuidanceReferenceOutput& reference,
                               core::Time_t elapsed_in_state_s,
                               core::Time_t dt_s,
                               core::Vec3& contribution) override;

private:
    BodySpecificForceGuidanceAccelerationConfig m_config{};
};

class ZeroBankGuidancePolicy final : public GuidanceBankPolicy
{
public:
    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool advance(const GuidanceReferenceOutput& reference,
                               const core::Vec3& acceleration_command_i_mps2,
                               const TrajectoryEnvironment& environment,
                               core::Scalar_t& bank_command_n_rad) const override;
    [[nodiscard]] bool bank_to_turn_active() const override;
};

struct CoordinatedBankToTurnGuidancePolicyConfig
{
    core::Scalar_t maximum_bank_angle_rad{};
};

class CoordinatedBankToTurnGuidancePolicy final : public GuidanceBankPolicy
{
public:
    explicit CoordinatedBankToTurnGuidancePolicy(CoordinatedBankToTurnGuidancePolicyConfig config);

    [[nodiscard]] bool config_is_valid() const override;
    [[nodiscard]] bool advance(const GuidanceReferenceOutput& reference,
                               const core::Vec3& acceleration_command_i_mps2,
                               const TrajectoryEnvironment& environment,
                               core::Scalar_t& bank_command_n_rad) const override;
    [[nodiscard]] bool bank_to_turn_active() const override;

private:
    CoordinatedBankToTurnGuidancePolicyConfig m_config{};
};

} // namespace navkit::sim
