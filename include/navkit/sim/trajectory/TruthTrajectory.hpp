// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/trajectory/TrajectoryDiagnostics.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <cstddef>
#include <vector>

namespace navkit::sim
{

/** A time-ordered ECEF truth trajectory with timestamp-queryable samples. */
class TruthTrajectory
{
public:
    TruthTrajectory() = default;

    /** Takes ownership of source samples, which callers provide in strict timestamp order. */
    explicit TruthTrajectory(std::vector<TruthSample> samples,
                             std::vector<TrajectoryDiagnostics> diagnostics = {});

    /** Returns whether the source contains no truth samples. */
    [[nodiscard]] bool empty() const;

    /** Returns the number of source samples. */
    [[nodiscard]] std::size_t size() const;

    /** Returns the first source sample. The caller must first check `empty()`. */
    [[nodiscard]] const TruthSample& first() const;

    /** Returns the last source sample. The caller must first check `empty()`. */
    [[nodiscard]] const TruthSample& last() const;

    /** Returns the immutable native-rate source samples. */
    [[nodiscard]] const std::vector<TruthSample>& samples() const;

    /** Appends one strictly later truth/diagnostic pair to a streaming trajectory. */
    [[nodiscard]] bool append(const TruthSample& sample, const TrajectoryDiagnostics& diagnostics);

    /** Replaces diagnostics for the latest streaming sample without changing truth. */
    [[nodiscard]] bool update_last_diagnostics(const TrajectoryDiagnostics& diagnostics);

    /** Queries truth at an in-range timestamp using linear state and SLERP attitude interpolation.
     */
    [[nodiscard]] bool sample_at(const core::Timestamp& t, TruthSample& sample) const;

    /** Queries diagnostics using the same interpolation contract as sample_at(). */
    [[nodiscard]] bool diagnostics_at(const core::Timestamp& t,
                                      TrajectoryDiagnostics& diagnostics) const;

private:
    std::vector<TruthSample> m_samples{};
    std::vector<TrajectoryDiagnostics> m_diagnostics{};
};

} // namespace navkit::sim
