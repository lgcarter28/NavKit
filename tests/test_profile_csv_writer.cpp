// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/profiling/ProfileSinks.hpp"
#include "navkit/io/ProfileCsvWriter.hpp"
#include "test_main.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace navkit::io::test
{

namespace
{

std::string read_file(const std::filesystem::path& path)
{
    std::ifstream file(path);
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

} // namespace

TEST_CASE("ProfileCsvWriter exports profile records with stable schema columns")
{
    const auto path = std::filesystem::temp_directory_path() / "navkit_profile_writer_test.csv";
    std::filesystem::remove(path);

    {
        ProfileCsvWriter<std::uint32_t> writer(path);
        writer.write_record({
            .point = core::profiling::ProfilePoint::KalmanObservationUpdate,
            .start_tick = 10U,
            .elapsed_ticks = 7U,
            .sequence = 3U,
            .parent_sequence = 1U,
            .depth = 2U,
            .flags = core::profiling::ProfileRecordFlags::DroppedBefore,
        });
        writer.flush();
    }

    const std::string contents = read_file(path);
    CHECK(contents.find("schema,point_id,point,start_tick,elapsed_ticks,sequence,parent_sequence,"
                        "depth,flags\n") == 0U);
    CHECK(contents.find("navkit.profile.v1,1,kalman.observation_update,10,7,3,1,2,1\n") !=
          std::string::npos);

    std::filesystem::remove(path);
}

TEST_CASE("Profile CSV drain helper consumes a core ring-buffer sink")
{
    using Sink = core::profiling::RingBufferProfileSink<std::uint32_t, 2U>;

    const auto path = std::filesystem::temp_directory_path() / "navkit_profile_drain_test.csv";
    std::filesystem::remove(path);

    Sink::reset();
    Sink::record({.point = core::profiling::ProfilePoint::NavigatorProcessMeasurements,
                  .start_tick = 4U,
                  .elapsed_ticks = 9U});

    const std::size_t count = drain_profile_sink_to_csv<std::uint32_t, Sink>(path);

    CHECK(count == 1U);
    CHECK(Sink::empty());
    CHECK(
        read_file(path).find("navkit.profile.v1,0,navigator.process_measurements,4,9,0,0,0,0\n") !=
        std::string::npos);

    std::filesystem::remove(path);
}

} // namespace navkit::io::test
