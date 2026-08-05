// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/sim/trajectory/TrajectoryDiagnostics.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

namespace navkit::sim
{

/**
 * Simulation-only streaming truth-source contract.
 *
 * A source owns its native resolution and advances availability through planned
 * time. Queries are bounded to already available truth and must not extrapolate.
 */
class TrajectorySource
{
public:
    virtual ~TrajectorySource() = default;

    /** Makes truth available through the requested planned timestamp. */
    [[nodiscard]] virtual bool advance_to(const core::Timestamp& t) = 0;

    /** Queries an available truth sample at an exact timestamp. */
    [[nodiscard]] virtual bool query(const core::Timestamp& t, TruthSample& sample) const = 0;

    /** Queries optional trajectory dynamics diagnostics at an available timestamp. */
    [[nodiscard]] virtual bool query_diagnostics(const core::Timestamp& t,
                                                 TrajectoryDiagnostics& diagnostics) const = 0;

    /**
     * Supplies the latest source-agnostic state used by Guidance and Autopilot.
     *
     * Non-controlled sources intentionally accept and ignore this input.
     */
    [[nodiscard]] virtual bool set_control_state(const TrajectoryControlState& state)
    {
        static_cast<void>(state);
        return true;
    }

    /**
     * Supplies one actual IMU increment to the Autopilot observation path.
     *
     * Non-controlled sources intentionally accept and ignore this input.
     */
    [[nodiscard]] virtual bool
    observe_imu_increment(const core::estimation::ImuIncrement& increment)
    {
        static_cast<void>(increment);
        return true;
    }

    /** Returns the source's first available timestamp. */
    [[nodiscard]] virtual core::Timestamp t_start() const = 0;

    /** Returns the source's final valid timestamp. */
    [[nodiscard]] virtual core::Timestamp t_end() const = 0;

    /** Returns whether advance_to() has reached the source end. */
    [[nodiscard]] virtual bool is_complete() const = 0;
};

} // namespace navkit::sim
