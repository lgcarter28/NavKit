// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/RingBuffer.hpp"
#include "navkit/core/profiling/ProfileRecord.hpp"

#include <cstddef>

namespace navkit::core::profiling
{

struct NullProfileSink
{
    template<typename Tick>
    static constexpr void record(ProfileRecord<Tick>) noexcept
    {}
};

template<typename Tick,
         std::size_t Capacity,
         containers::OverflowPolicy Policy = containers::OverflowPolicy::Reject>
class RingBufferProfileSink
{
public:
    using Record = ProfileRecord<Tick>;

    static_assert(Capacity > 0U, "RingBufferProfileSink capacity must be greater than zero");

    static void record(Record record)
    {
        if constexpr (Policy == containers::OverflowPolicy::OverwriteOldest) {
            if (s_records.full()) {
                ++s_dropped_count;
                record.flags = record.flags | ProfileRecordFlags::DroppedBefore;
            }
            static_cast<void>(s_records.push(record));
        }
        else {
            if (!s_records.push(record)) {
                ++s_dropped_count;
            }
        }
    }

    [[nodiscard]] static bool pop(Record& out)
    {
        return s_records.pop(out);
    }

    [[nodiscard]] static bool front(Record& out)
    {
        return s_records.front(out);
    }

    [[nodiscard]] static bool empty()
    {
        return s_records.empty();
    }

    [[nodiscard]] static bool full()
    {
        return s_records.full();
    }

    [[nodiscard]] static std::size_t size()
    {
        return s_records.size();
    }

    [[nodiscard]] static constexpr std::size_t capacity()
    {
        return Capacity;
    }

    [[nodiscard]] static std::size_t dropped_count()
    {
        return s_dropped_count;
    }

    static void reset()
    {
        s_records.clear();
        s_dropped_count = 0U;
    }

private:
    static inline containers::RingBuffer<Record, Capacity, Policy> s_records{};
    static inline std::size_t s_dropped_count{0U};
};

} // namespace navkit::core::profiling
