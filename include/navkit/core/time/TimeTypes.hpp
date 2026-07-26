// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <cstdint>

namespace navkit::core
{

/** Integral whole-second storage for public time contracts. */
using Seconds = std::uint64_t;

/** Normalized fractional-second storage for public time contracts. */
using Nanoseconds = std::uint32_t;

/** Signed whole seconds reserved for a future explicit SignedDuration contract. */
using SignedSeconds = std::int64_t;

/** Monotonic sample counter used by exact rational-rate scheduling. */
using SampleIndex = std::uint64_t;

/** Numerator storage for a rational sample rate. */
using Samples = std::uint32_t;

} // namespace navkit::core
