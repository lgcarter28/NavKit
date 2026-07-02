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

TEST_CASE("Profile CSV drain helper supports custom record metadata integer types")
{
    using Sink = core::profiling::RingBufferProfileSink<std::uint32_t,
                                                        2U,
                                                        core::containers::OverflowPolicy::Reject,
                                                        std::uint16_t,
                                                        std::uint8_t>;

    const auto path =
        std::filesystem::temp_directory_path() / "navkit_profile_custom_record_test.csv";
    std::filesystem::remove(path);

    Sink::reset();
    Sink::record({.point = core::profiling::ProfilePoint::PropagationUpdate,
                  .start_tick = 12U,
                  .elapsed_ticks = 5U,
                  .sequence = 9U,
                  .parent_sequence = 3U,
                  .depth = 1U});

    const std::size_t count = drain_profile_sink_to_csv<Sink>(path);

    CHECK(count == 1U);
    CHECK(read_file(path).find("navkit.profile.v1,2,propagation.update,12,5,9,3,1,0\n") !=
          std::string::npos);

    std::filesystem::remove(path);
}

TEST_CASE("Profile run manifest writer records runtime profile output metadata")
{
    const auto path =
        std::filesystem::temp_directory_path() / "navkit_profile_run_manifest_test.json";
    std::filesystem::remove(path);

    write_profile_run_manifest(
        path, {.run_name = "test_run", .record_count = 3U, .dropped_record_count = 1U});

    const std::string contents = read_file(path);
    CHECK(contents.find("\"schema\": \"navkit.profile_run_manifest.v1\"") != std::string::npos);
    CHECK(contents.find("\"run_name\": \"test_run\"") != std::string::npos);
    CHECK(contents.find("\"record_count\": 3") != std::string::npos);
    CHECK(contents.find("\"dropped_record_count\": 1") != std::string::npos);

    std::filesystem::remove(path);
}

} // namespace navkit::io::test
