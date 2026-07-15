// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

namespace navkit::io
{

template<typename StateDef, typename Filter>
struct FilterCorrectionLogPayload
{
    core::Time_t time_s{};
    const Filter& filter;
};

} // namespace navkit::io
