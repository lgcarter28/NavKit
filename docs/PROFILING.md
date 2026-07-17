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

## Hot-path resource expectations

Profiling is designed to be zero-overhead by default and fixed-resource when
enabled.

- `NullProfiler` is the default profiler. It returns an empty
  `NullProfileScope`, owns no state, and is expected to compile away in optimized
  builds.
- Active core profiling records only fixed-size values. It does not allocate,
  own strings, write files, format JSON/CSV, or use runtime polymorphism.
- Fixed-capacity sinks make storage costs explicit at compile time. Overflow
  behavior is part of the sink type, not hidden runtime policy.
- Desktop timing, trace, and binary-size artifacts are trend signals. They are
  useful for catching regressions early, but they are not a substitute for
  target-specific resource evidence.

Current tests check the contract shape that can be verified portably: the no-op
profiler path is empty/trivial/noexcept, profile records are standard-layout
fixed-size values, and ring-buffer sinks expose fixed capacity and overflow
policy. Hard embedded claims require target profiles: selected compiler,
optimization flags, clock/counter source, memory map, and concurrency model.
Assembly inspection, active-vs-no-op cycle counting, allocation/high-water
instrumentation, and hardware microbenchmarks belong in those target-specific
passes.

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

Compile-time profiling metadata belongs to the build, not the run. The selected
compile-time config owns the C++ metadata constants. After the executable is
built, `tools/build.py` asks the executable to describe the selected config
through `navkit::app_support` helpers and writes that derived metadata into
`build/<type>/navkit_build_manifest.json`, including clock source, tick-to-time
conversion, sink capacity, overflow policy, and profile-point mapping. The
runtime application uses app-support profile-export helpers to write only
run-specific profile metadata in `profile_run_manifest.json`: run name, CSV
file, record count, and dropped-record count.

Binary dumps and a formal binary ICD are intentionally deferred until real
algorithm records and metadata have been exercised through at least one
integration path.

## Configuration knobs

Profiling behavior is selected at compile time by the concrete config. The main
knobs are:

- Clock source and resolution: desktop steady-clock microseconds today; future
  embedded configs may use hardware timers, cycle counters, or RTOS clocks.
- Tick type and range: `std::uint64_t` is comfortable for desktop; embedded
  targets may prefer narrower counters and explicit wraparound handling.
- Sink capacity: larger or denser instrumentation needs more fixed storage,
  streaming, or overwrite behavior.
- Overflow policy: reject overflow to preserve earliest records, or overwrite
  oldest to preserve the most recent window.
- Instrumentation density: profile-point placement usually matters more than
  changing microseconds to nanoseconds.
- Export metadata: tick scale, clock source, selected config, sink capacity,
  and profile-point mapping belong in `navkit_build_manifest.json`; record
  count and dropped count belong in `profile_run_manifest.json`.

## Analysis and visualization

The easiest end-to-end profiling demo is the profiled stationary GNSS config:

```bash
python tools/build.py --build-type Debug --skip-conan \
  --navkit-config apps/navkit_sim/ProfiledEcefInsGnss.hpp

python tools/run_sim.py --build-type Debug --navkit-config apps/navkit_sim/ProfiledEcefInsGnss.hpp
```

That writes:

- `output/logs/ecef_ins_gnss_demo/profile.csv`
- `output/logs/ecef_ins_gnss_demo/profile_run_manifest.json`
- `output/logs/ecef_ins_gnss_demo/profile.trace.json`

`run_sim.py --navkit-config` uses the compile-time config path to locate the
matching build tree and executable; it does not switch the compiled executable
at runtime. The runner prints the profile summary and writes the trace JSON
automatically when `profile.csv` exists. Use `--no-profile-report` or
`--no-profile-trace` for quieter runs.

`--build-type` still matters because Debug and Release builds can coexist. If
you keep multiple selected configs in separate build trees, pass `--build-dir`
to select the tree to run.

Use `tools/profile_report.py` to inspect exported profile CSVs:

```bash
python tools/profile_report.py output/logs/<run_name>/profile.csv
```

To generate Chrome Trace / Perfetto-compatible JSON from an existing profile:

```bash
python tools/profile_report.py output/logs/<run_name>/profile.csv \
  --build-manifest build/debug/apps/navkit_sim/ProfiledEcefInsGnss/navkit_build_manifest.json \
  --chrome-trace output/logs/<run_name>/profile.trace.json
```

Trace conversion requires both the runtime profile manifest and the build
manifest so compile-time and runtime facts remain separate:

```bash
python tools/profile_report.py output/logs/<run_name>/profile.csv \
  --profile-run-manifest output/logs/<run_name>/profile_run_manifest.json \
  --build-manifest build/debug/apps/navkit_sim/ProfiledEcefInsGnss/navkit_build_manifest.json \
  --chrome-trace output/logs/<run_name>/profile.trace.json
```

Use `--tick-period-us` only as an explicit diagnostic override:

```bash
python tools/profile_report.py output/logs/<run_name>/profile.csv \
  --tick-period-us 0.5 \
  --chrome-trace output/logs/<run_name>/profile.trace.json
```

The JSON can be loaded into Chrome's tracing viewer or Perfetto for timeline
inspection. More advanced NavKit-native visualization can be added later if the
standard trace viewers are not enough.
