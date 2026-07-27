// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"

namespace navkit::app_support
{

/** App-support clock boundary shared by deterministic and wall-clock runs. */
class Clock
{
public:
    virtual ~Clock() = default;

    [[nodiscard]] virtual bool initialize(const core::Timestamp& t_epoch) = 0;
    [[nodiscard]] virtual bool wait_until(const core::Timestamp& t) = 0;
    [[nodiscard]] virtual core::Timestamp now() const = 0;
};

} // namespace navkit::app_support
