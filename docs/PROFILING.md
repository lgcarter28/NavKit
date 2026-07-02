# NavKit Profiling

NavKit profiling is split into two layers:

- `navkit::core` owns the embedded-facing record format, clock policy,
  profiler policy, and fixed-capacity sink policies.
- `navkit::io` and Python tooling own desktop export, file formats, summaries,
  and visualization conversion.

This keeps the hot path deterministic while still making profiling data easy to
inspect on a workstation.

## Core record flow

`ScopedProfiler<Clock, Sink>` records a `ProfileRecord<Tick>` when the scope
ends. The record contains only fixed-size values: profile point ID, start tick,
elapsed ticks, optional nesting/sequence metadata, and flags.

The first concrete sinks are intentionally simple:

- `NullProfileSink` accepts records and discards them.
- `RingBufferProfileSink<Tick, Capacity, Policy>` stores records in fixed
  static storage using `navkit::core::containers::RingBuffer`.

`RingBufferProfileSink` is single-producer and non-thread-safe by design. Its
overflow behavior is explicit:

- `OverflowPolicy::Reject` keeps the oldest queued records and increments the
  dropped-record count when full.
- `OverflowPolicy::OverwriteOldest` keeps the newest fixed-capacity window,
  increments the dropped-record count, and marks the inserted record with
  `ProfileRecordFlags::DroppedBefore`.

RTOS, ISR-safe, multi-producer, lock-free, or atomic sinks should be added later
as separate sink policies when a concrete target requires them. They should not
be hidden inside the baseline sink.

## Export path

Core sinks do not write files or strings. Desktop export adapters live in
`navkit::io`.

The initial C++ export adapter is `ProfileCsvWriter<Tick>`, which writes the
logical `navkit.profile.v1` schema:

```text
schema,point_id,point,start_tick,elapsed_ticks,sequence,parent_sequence,depth,flags
```

`drain_profile_sink_to_csv<Tick, Sink>(path)` drains a fixed-capacity core sink
into that CSV format.

Complete run metadata should live beside the exported records, not inside the
core hot path. Useful metadata includes schema version, profile-point mapping,
clock source, tick frequency or tick-to-time conversion, build/config identity,
run name, dropped-record count, and record flags.

Binary dumps and a formal binary ICD are intentionally deferred until real
algorithm records and metadata have been exercised through at least one
integration path.

## Analysis and visualization

The easiest end-to-end profiling demo is the profiled stationary GNSS config:

```bash
python tools/build.py --build-type Debug --skip-conan \
  --navkit-config navkit_sim/ProfiledStationaryGnss.hpp

python tools/run_first_sim.py --build-type Debug
```

That writes:

- `data/logs/stationary_gnss_demo/profile.csv`
- `data/logs/stationary_gnss_demo/profile.trace.json`

`run_first_sim.py` reads the selected compile-time config from the build
manifest written by `tools/build.py`; the compile-time config does not need to
be repeated when running the executable. The runner prints the profile summary
and writes the trace JSON automatically when `profile.csv` exists. Use
`--no-profile-report` or `--no-profile-trace` for quieter runs.

`--build-type` still matters because Debug and Release builds can coexist. If
you keep multiple selected configs in separate build trees, pass `--build-dir`
to select the tree to run.

Use `tools/profile_report.py` to inspect exported profile CSVs:

```bash
python tools/profile_report.py data/logs/<run_name>/profile.csv
```

To generate Chrome Trace / Perfetto-compatible JSON:

```bash
python tools/profile_report.py data/logs/<run_name>/profile.csv \
  --chrome-trace data/logs/<run_name>/profile.trace.json
```

If one clock tick is not one microsecond, provide the conversion:

```bash
python tools/profile_report.py data/logs/<run_name>/profile.csv \
  --tick-period-us 0.5 \
  --chrome-trace data/logs/<run_name>/profile.trace.json
```

The JSON can be loaded into Chrome's tracing viewer or Perfetto for timeline
inspection. More advanced NavKit-native visualization can be added later if the
standard trace viewers are not enough.
