# Phase 14 - Profiling and Resource Validation

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

## Pass 14.1: profiling evidence

- [ ] Improve Chrome Trace / Perfetto export readability with process/thread metadata, clearer display names, and stable category naming.
- [ ] Use profile record `sequence`, `parent_sequence`, and `depth` to represent cleaner nested timing once nested paths justify it.
- [ ] Add target-specific hot-path evidence: optimized assembly inspection, active-vs-no-op profiling cycle counts, and hardware/compiler-specific microbenchmarks once target clocks and toolchains exist.

## Pass 14.2: resource and allocation validation

- [ ] Add a quick embedded-core smoke-size target that links only `navkit::core`, instantiates the selected embedded-facing navigator/filter/sensor graph, avoids `navkit::sim`, `navkit::io`, JSON, filesystem, and app-support dependencies, and reports the Release executable/library size as an early embedded footprint baseline.
- [ ] Keep the first embedded smoke target intentionally small: force representative template instantiation and, if practical, run one no-IO update path with compile-time config only. Use it to expose accidental core-to-desktop dependency leaks before adding stricter tooling.
- [ ] Add a rigorous embedded resource evidence pass after the quick smoke target: emit map files, report `.text`, `.rdata`/read-only data, `.data`, `.bss`, static object footprint, and obvious large symbols; document the selected compiler flags, target/toolchain assumptions, and link-time dead-stripping behavior.
- [ ] Add package/install-tree smoke validation for the embedded-core target once install rules mature, so resource evidence can be collected from packaged artifacts rather than only the developer build tree.
- [ ] Extend allocation/resource checks to sensor queues, estimator update operations, propagation, mechanization, and logging seams.
- [ ] Add runtime and memory summaries appropriate for simulator qualification runs.
- [ ] Add runtime and memory budgets to CI or target qualification tests.
- [ ] Evaluate whether Perfetto/Chrome trace remains sufficient before building NavKit-native timeline or flame-style visualization.
- [ ] Consider an optional binary log/profile backend only after CSV/JSON throughput becomes a demonstrated bottleneck.
