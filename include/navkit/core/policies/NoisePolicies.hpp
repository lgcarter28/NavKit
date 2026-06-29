#pragma once

namespace navkit
{

struct DefaultNoisePolicy
{
    template<typename NoiseContext, typename Measurement>
    static void update(NoiseContext&, const Measurement&)
    {
        // default: do nothing
    }
};

struct GnssFixedNoisePolicy
{
    template<typename NoiseContext, typename Measurement>
    static void update(NoiseContext&, const Measurement&)
    {
        // GNSS noise is fixed by the model's default NoiseContext in this first draft.
    }
};

} // namespace navkit
