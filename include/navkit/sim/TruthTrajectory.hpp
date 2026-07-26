// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/TruthSample.hpp"

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
    explicit TruthTrajectory(std::vector<TruthSample> samples);

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

    /** Queries truth at an in-range timestamp using linear state and SLERP attitude interpolation.
     */
    [[nodiscard]] bool sample_at(const core::Timestamp& t, TruthSample& sample) const;

private:
    std::vector<TruthSample> m_samples{};
};

} // namespace navkit::sim
