// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/TruthSample.hpp"

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

    /** Returns the source's first available timestamp. */
    [[nodiscard]] virtual core::Timestamp t_start() const = 0;

    /** Returns the source's final valid timestamp. */
    [[nodiscard]] virtual core::Timestamp t_end() const = 0;

    /** Returns whether advance_to() has reached the source end. */
    [[nodiscard]] virtual bool is_complete() const = 0;
};

} // namespace navkit::sim
