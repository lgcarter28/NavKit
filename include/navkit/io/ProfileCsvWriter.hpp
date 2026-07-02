// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfileRecord.hpp"
#include "navkit/io/CsvWriter.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <string_view>

namespace navkit::io
{

inline constexpr std::string_view ProfileExportSchema = "navkit.profile.v1";
inline constexpr std::string_view ProfileRunManifestSchema = "navkit.profile_run_manifest.v1";

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

template<typename Tick, typename Sequence = std::uint32_t, typename Depth = std::uint16_t>
class ProfileCsvWriter
{
public:
    using Record = core::profiling::ProfileRecord<Tick, Sequence, Depth>;

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
                        static_cast<std::uint64_t>(record.sequence),
                        static_cast<std::uint64_t>(record.parent_sequence),
                        static_cast<std::uint64_t>(record.depth),
                        static_cast<std::uint16_t>(record.flags));
    }

    void flush()
    {
        m_csv.flush();
    }

private:
    CsvWriter m_csv;
};

struct ProfileRunManifest
{
    std::string run_name;
    std::size_t record_count{};
    std::size_t dropped_record_count{};
    std::string csv_file{"profile.csv"};
};

[[nodiscard]] inline nlohmann::json profile_point_mapping()
{
    using core::profiling::ProfilePoint;

    return nlohmann::json::array(
        {{{"id", static_cast<std::uint16_t>(ProfilePoint::NavigatorProcessMeasurements)},
          {"name", profile_point_name(ProfilePoint::NavigatorProcessMeasurements)}},
         {{"id", static_cast<std::uint16_t>(ProfilePoint::KalmanObservationUpdate)},
          {"name", profile_point_name(ProfilePoint::KalmanObservationUpdate)}},
         {{"id", static_cast<std::uint16_t>(ProfilePoint::PropagationUpdate)},
          {"name", profile_point_name(ProfilePoint::PropagationUpdate)}}});
}

inline void write_profile_run_manifest(const std::filesystem::path& path,
                                       const ProfileRunManifest& manifest)
{
    nlohmann::json document = {
        {"schema", ProfileRunManifestSchema},
        {"profile_schema", ProfileExportSchema},
        {"csv_file", manifest.csv_file},
        {"run_name", manifest.run_name},
        {"record_count", manifest.record_count},
        {"dropped_record_count", manifest.dropped_record_count},
    };

    std::ofstream file(path);
    if (!file) {
        throw std::runtime_error("Failed to open profile run manifest: " + path.string());
    }
    file << document.dump(2) << '\n';
}

template<typename Sink>
std::size_t drain_profile_sink_to_csv(const std::filesystem::path& path)
{
    using Record = typename Sink::Record;
    using Tick = typename Record::Tick_t;
    using Sequence = typename Record::Sequence_t;
    using Depth = typename Record::Depth_t;

    ProfileCsvWriter<Tick, Sequence, Depth> writer(path);
    Record record{};
    std::size_t count{0U};

    while (Sink::pop(record)) {
        writer.write_record(record);
        ++count;
    }

    writer.flush();
    return count;
}

template<typename Tick, typename Sink>
std::size_t drain_profile_sink_to_csv(const std::filesystem::path& path)
{
    return drain_profile_sink_to_csv<Sink>(path);
}

} // namespace navkit::io
