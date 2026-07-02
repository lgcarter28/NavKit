// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileRecord.hpp"
#include "navkit/io/CsvWriter.hpp"

#include <cstdint>
#include <filesystem>
#include <string_view>

namespace navkit::io
{

inline constexpr std::string_view ProfileExportSchema = "navkit.profile.v1";

[[nodiscard]] constexpr std::string_view
profile_point_name(core::profiling::ProfilePoint point) noexcept
{
    using core::profiling::ProfilePoint;

    switch (point) {
    case ProfilePoint::NavigatorProcessMeasurements:
        return "navigator.process_measurements";
    case ProfilePoint::KalmanObservationUpdate:
        return "kalman.observation_update";
    case ProfilePoint::PropagationUpdate:
        return "propagation.update";
    }

    return "unknown";
}

template<typename Tick>
class ProfileCsvWriter
{
public:
    using Record = core::profiling::ProfileRecord<Tick>;

    explicit ProfileCsvWriter(const std::filesystem::path& path)
        : m_csv(path,
                {"schema",
                 "point_id",
                 "point",
                 "start_tick",
                 "elapsed_ticks",
                 "sequence",
                 "parent_sequence",
                 "depth",
                 "flags"})
    {}

    void write_record(const Record& record)
    {
        m_csv.write_row(ProfileExportSchema,
                        static_cast<std::uint16_t>(record.point),
                        profile_point_name(record.point),
                        record.start_tick,
                        record.elapsed_ticks,
                        record.sequence,
                        record.parent_sequence,
                        record.depth,
                        static_cast<std::uint16_t>(record.flags));
    }

    void flush()
    {
        m_csv.flush();
    }

private:
    CsvWriter m_csv;
};

template<typename Tick, typename Sink>
std::size_t drain_profile_sink_to_csv(const std::filesystem::path& path)
{
    ProfileCsvWriter<Tick> writer(path);
    core::profiling::ProfileRecord<Tick> record{};
    std::size_t count{0U};

    while (Sink::pop(record)) {
        writer.write_record(record);
        ++count;
    }

    writer.flush();
    return count;
}

} // namespace navkit::io
