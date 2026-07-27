// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalRate.hpp"
#include "navkit/sim/TrajectorySource.hpp"

namespace navkit::sim
{

struct StationaryTrajectoryConfig
{
    core::Time_t duration_s{60.0};
    core::RationalRate rate{.samples = 1U, .s = 1U};
    core::Timestamp t_epoch{};
    core::Vec3 p_e{6378137.0, 0.0, 0.0};
    core::Vec3 v_e{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Vec3 w_ib_b_radps{core::Vec3::Zero()};
};

/** Generates stationary ECEF truth lazily through the requested planned time. */
class StationaryTrajectorySource final : public TrajectorySource
{
public:
    explicit StationaryTrajectorySource(const StationaryTrajectoryConfig& cfg);

    [[nodiscard]] bool advance_to(const core::Timestamp& t) override;
    [[nodiscard]] bool query(const core::Timestamp& t, TruthSample& sample) const override;
    [[nodiscard]] core::Timestamp t_start() const override;
    [[nodiscard]] core::Timestamp t_end() const override;
    [[nodiscard]] bool is_complete() const override;

private:
    [[nodiscard]] bool timestamp_is_in_source_range(const core::Timestamp& t) const;

    StationaryTrajectoryConfig m_cfg{};
    core::Timestamp m_t_available{};
    bool m_valid{false};
    bool m_initialized{false};
    bool m_complete{false};
};

} // namespace navkit::sim
