// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit
{

template<typename Candidate, typename Model, typename MeasurementSample>
concept NoisePolicy = requires(typename Model::NoiseContext& ctx, const MeasurementSample& meas) {
    { Candidate::update(ctx, meas) } -> std::same_as<void>;
};

} // namespace navkit
