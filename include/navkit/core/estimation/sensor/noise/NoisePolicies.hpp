// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::core::estimation
{

struct DefaultNoisePolicy
{
    template<typename ObservationContext, typename Measurement>
    static void update(ObservationContext&, const Measurement&)
    {
        // default: do nothing
    }
};

struct GnssFixedNoisePolicy
{
    template<typename ObservationContext, typename Measurement>
    static void update(ObservationContext&, const Measurement&)
    {
        // GNSS noise is fixed by the model's default ObservationContext in this first draft.
    }
};

} // namespace navkit::core::estimation
