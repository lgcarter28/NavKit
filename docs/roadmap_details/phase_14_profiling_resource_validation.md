# Phase 14 - Profiling and Resource Validation

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

## Pass 14.1: profiling evidence

- [ ] Improve Chrome Trace / Perfetto export readability with process/thread metadata, clearer display names, and stable category naming.
- [ ] Use profile record `sequence`, `parent_sequence`, and `depth` to represent cleaner nested timing once nested paths justify it.
- [ ] Add target-specific hot-path evidence: optimized assembly inspection, active-vs-no-op profiling cycle counts, and hardware/compiler-specific microbenchmarks once target clocks and toolchains exist.

## Pass 14.2: resource and allocation validation

- [ ] Extend allocation/resource checks to sensor queues, estimator update operations, propagation, mechanization, and logging seams.
- [ ] Add runtime and memory summaries appropriate for simulator qualification runs.
- [ ] Add runtime and memory budgets to CI or target qualification tests.
- [ ] Evaluate whether Perfetto/Chrome trace remains sufficient before building NavKit-native timeline or flame-style visualization.
- [ ] Consider an optional binary log/profile backend only after CSV/JSON throughput becomes a demonstrated bottleneck.
