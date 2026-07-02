// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/containers/RingBuffer.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/io/ProfileCsvWriter.hpp"

#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>
#include <type_traits>

namespace navkit::app_support
{

template<typename Config, typename = void>
struct ConfigProfiler
{
    using type = core::profiling::NullProfiler;
};

template<typename Config>
struct ConfigProfiler<Config, std::void_t<typename Config::Profiler>>
{
    using type = typename Config::Profiler;
};

template<typename Config, typename = void>
inline constexpr bool has_profile_export_v = false;

template<typename Config>
inline constexpr bool
    has_profile_export_v<Config,
                         std::void_t<typename Config::ProfileTick, typename Config::ProfileSink>> =
        true;

template<typename Config>
void reset_profile_sink_if_configured()
{
    if constexpr (has_profile_export_v<Config>) {
        using Sink = typename Config::ProfileSink;
        Sink::reset();
    }
}

template<typename Config>
void export_profile_if_configured(const std::filesystem::path& output_dir,
                                  const std::string& run_name)
{
    if constexpr (has_profile_export_v<Config>) {
        using Sink = typename Config::ProfileSink;

        const auto profile_path = output_dir / "profile.csv";
        const std::size_t record_count = io::drain_profile_sink_to_csv<Sink>(profile_path);
        const auto manifest_path = output_dir / "profile_run_manifest.json";

        io::write_profile_run_manifest(manifest_path,
                                       {.run_name = run_name,
                                        .record_count = record_count,
                                        .dropped_record_count = Sink::dropped_count()});

        std::printf("Wrote NavKit profile log to: %s (%zu records, %zu dropped)\n",
                    profile_path.string().c_str(),
                    record_count,
                    Sink::dropped_count());
    }
}

template<typename Sink>
[[nodiscard]] constexpr std::string_view profile_overflow_policy_name()
{
    using core::containers::OverflowPolicy;

    if constexpr (Sink::overflow_policy() == OverflowPolicy::Reject) {
        return "reject";
    }
    else if constexpr (Sink::overflow_policy() == OverflowPolicy::OverwriteOldest) {
        return "overwrite_oldest";
    }
    else {
        return "unknown";
    }
}

} // namespace navkit::app_support
