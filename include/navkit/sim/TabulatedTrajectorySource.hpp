// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/TrajectorySource.hpp"
#include "navkit/sim/TruthTrajectory.hpp"

namespace navkit::sim
{

/** Makes an immutable tabulated truth trajectory available through planned time. */
class TabulatedTrajectorySource final : public TrajectorySource
{
public:
    explicit TabulatedTrajectorySource(TruthTrajectory trajectory);

    [[nodiscard]] bool advance_to(const core::Timestamp& t) override;
    [[nodiscard]] bool query(const core::Timestamp& t, TruthSample& sample) const override;
    [[nodiscard]] core::Timestamp t_start() const override;
    [[nodiscard]] core::Timestamp t_end() const override;
    [[nodiscard]] bool is_complete() const override;

private:
    TruthTrajectory m_trajectory{};
    core::Timestamp m_t_available{};
    bool m_initialized{false};
    bool m_complete{false};
};

} // namespace navkit::sim
