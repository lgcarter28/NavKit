# Phase 15 - Profiling, Resource Validation, and Remaining Embedded Hardening

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

The Phase 12 MVP pulls forward only the minimum evidence needed to run real
hardware. This phase completes target profiling, resource qualification,
packaging, API hygiene, documentation, and production telemetry after a concrete
target and measured workload exist.

## Pass 15.1: profiling evidence

- [ ] Improve Chrome Trace / Perfetto export readability with process/thread
  metadata, clearer display names, and stable category naming.
- [ ] Use profile record sequence, parent sequence, and depth for nested timing
  once actual target paths justify it.
- [ ] Add target-specific hot-path assembly inspection, active-versus-no-op
  profiling cycle counts, compiler/toolchain comparisons, and measured deadline
  distributions.

## Pass 15.2: full resource and allocation qualification

- [ ] Extend the Phase 12 smoke evidence into map-file and symbol-level resource
  qualification for queues, estimator updates, propagation, mechanization,
  telemetry, and all supported target profiles.
- [ ] Add stack high-water, static storage, allocation, CPU, deadline, queue,
  telemetry-throughput, and drop budgets to target qualification and CI where
  the runner can produce meaningful evidence.
- [ ] Add package/install-tree smoke validation so evidence is collected from
  distributable artifacts rather than only a developer tree.

## Pass 15.3: embedded deployment profiles

- [ ] Define allocation, exception, RTTI, logging, and timing constraints for supported embedded profiles.
- [ ] Audit large fixed-size members and buffers, especially `RingBuffer` instances and state/history buffers, for stack-growth risk. Decide where large storage should remain embedded inline, move to static/global ownership, or be held behind pointers/references supplied by the application or target platform.
- [ ] Add embedded toolchain profiles and a hardware abstraction boundary when a target is selected.

## Pass 15.4: type and frame safety

- [ ] Extend existing unit/frame types based on observed misuse risks.
- [ ] Add compile-time DCM composition/result-frame checks where the added type machinery improves safety without obscuring Eigen interoperability.
- [ ] Add tested unit conversions and arithmetic only where they protect real boundaries.
- [ ] Keep frame/type abstractions zero-overhead and verify generated/runtime behavior where important.

## Pass 15.5: explicit type and API hygiene

- [ ] Continue the repository-wide `auto` audit using the strengthened AGENTS rule. Replace nontrivial `auto` in math, Eigen, state, frame/unit, simulator, and logging code with explicit types unless it falls into the narrow allowed cases.
- [ ] Keep the audit separate from functional navigation/simulation passes so style-only churn does not obscure numerical or architecture changes.

## Pass 15.6: documentation and API teaching material

- [ ] Define repository documentation-comment guidelines and update `AGENTS.md` so new C++ functionality eventually requires Doxygen comments on public and internal functions, classes, structs, namespaces, aliases, concepts, and policy boundaries; evaluate whether the same standard should apply to Python modules/functions.
- [ ] Add Doxygen/API documentation where it helps users understand policy contracts and math boundaries.
- [ ] Add documentation build profiles for different audiences/scopes, such as embedded/product-core API only, external user API docs, and full internal developer docs.
- [ ] Establish a reproducible publication-figure toolchain. Use Matplotlib for
  numerical/data products, TikZ for two-dimensional mathematical, architecture,
  state-machine, and signal-flow diagrams, Asymptote for genuine three-
  dimensional frame/rotation/geometry figures, and Ipe only when a GUI-authored
  academic vector figure is the clearer source of truth. Reserve Manim for
  animations rather than treating it as the default static-figure renderer.
- [ ] Add a shared NavKit publication API and style contract for Python figures:
  journal column widths, typography, mathematical text, line weights,
  colorblind-safe palettes, units, vector PDF/SVG output, and high-resolution
  PNG inspection previews. Reuse the same domain preparation beneath
  interactive Plotly, publication Matplotlib, and future paper/report products
  instead of duplicating numerical derivations.
- [ ] Keep document-owned figure source beside the LaTeX document that owns it,
  for example `docs/algorithms/<document>/figures/<figure>.asy`, `.tex`, `.py`,
  or `.ipe`. Keep only genuinely shared styles, reusable drawing primitives,
  and cross-document figures under a central documentation support area such
  as `docs/figure_support/`. Emit PDFs, SVGs, PNG previews, and other generated
  artifacts under `build/docs/<document>/` rather than mixing generated output
  with authored document sources. Treat the document-local source as canonical;
  do not create an SVG/GUI round-trip that competes with `.asy`, `.tex`, or
  `.py` source. A GUI-native `.ipe` figure is canonical only when manual visual
  layout is deliberately the owning workflow.
- [ ] Add one figure-build entry point that discovers or explicitly registers
  document figures, renders canonical vector output, rasterizes inspection
  previews, and builds a proof page at final publication scale. Fail on LaTeX
  or renderer errors, missing fonts, invalid/empty bounds, and stale products;
  enable an agent to inspect the rendered preview and iterate before the owning
  document is accepted.
- [ ] Define technical-figure review requirements: no clipped or overlapping
  labels, readable final-size typography, minimum line weights, grayscale and
  color-vision legibility, embedded fonts, explicit frames and units, and
  geometry consistent with the documented active/passive rotation and
  start-to-end frame conventions. Where figures are generated from
  mathematical parameters, verify relevant invariants such as normalized axes,
  orthogonal direction-cosine matrices, determinant `+1`, and the declared
  Euler rotation order from the same inputs used to draw the figure.
- [ ] Document and validate the figure-tool dependency contract in setup and
  CI: a supported TeX distribution, `latexmk`, PGF/TikZ and `tikz-3dplot`,
  Asymptote, and a PDF rasterizer such as `pdftocairo`/`pdftoppm`. Keep Ipe,
  Inkscape, and Manim optional unless a checked-in figure or animation actually
  selects one of them as its source workflow.
- [ ] Grow the navigation theory/reference manual alongside implemented equations.
- [ ] Add measurement-model and mechanization tutorials.
- [ ] Add end-to-end scenario walkthroughs and a developer architecture guide.
- [ ] Revisit ADR-001 through ADR-003 and either accept them, revise them, or keep them Proposed with explicit unresolved questions.
- [ ] Keep README, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, `docs/CONFIGURATION.md`, and the active roadmap reconciled after ADR decisions or workflow changes.

## Pass 15.7: CI and release workflow hygiene

- [ ] Confirm hosted GitHub Actions runs pass on the supported platforms and document any local/CI differences.
- [ ] Add install/package validation: build, install, and run a minimal smoke scenario or executable from the install tree so packaged artifacts are tested separately from the developer build tree.
- [ ] Document the expected `build/`, `install/`, `output/`, and CI artifact layout for users and release workflows.
- [ ] Keep the owner-controlled release/tag/archive/backup workflow documented outside normal engineering tasks if it needs future maintenance.

## Pass 15.8: binary logging and telemetry contract

- [ ] Define a versioned, endian-explicit binary record envelope suitable for target logging and live telemetry: stream/schema identifier, record type, payload length, timestamp/clock domain, sequence number, and integrity field as appropriate for the selected transport.
- [ ] Define the stable wire representation for scalar, fixed-size vector/matrix, quaternion, state, covariance, IMU increment, observation, estimator-health, and profiling payloads. Make frame, units, state-definition/schema identity, and covariance layout explicit in metadata rather than relying on host-native layout or implicit CSV-header conventions.
- [ ] Define telemetry channel/record registration, compatibility, forward/backward handling, unknown-record skipping, alignment/packing, fragmentation, bounded-record-size, and corruption/recovery rules. Keep the initial contract allocation-free and deterministic on embedded targets.
- [ ] Separate the portable binary data contract from transport adapters. The same records must support a target file/ring-buffer recorder, serial/CAN/UDP-style telemetry adapter, and desktop capture/replay without putting filesystem, sockets, JSON, or simulation dependencies into `navkit::core`.
- [ ] Document the intended division of responsibility: embedded targets emit compact NavKit binary records; desktop Python tooling validates, decodes, and repackages them into HDF5/other analysis bundles. Direct C++ HDF5 logging remains out of scope.

## Pass 15.9: binary recorder, decoder, and qualification tooling

- [ ] Implement a fixed-capacity binary recorder/telemetry sink behind the Phase 15.6 contract, with explicit overflow/backpressure/drop accounting and no hidden allocation on the embedded-facing path.
- [ ] Add desktop capture/replay and Python decoding/packaging utilities that ingest the binary records into the same shared analysis-data interface used by CSV and HDF5 inputs; preserve source schema, stream metadata, and loss/corruption diagnostics.
- [ ] Add explicit analysis-bundle retention profiles for full raw tables, selected raw tables, and cached-derived/aggregate-only artifacts. Preserve provenance in every profile and benchmark packaging/reload/storage tradeoffs at 100-, 500-, and 1,000-run campaign scale before choosing defaults.
- [ ] Add golden-byte compatibility tests, malformed/truncated/corrupt-record tests, endian/layout checks, decoder compatibility tests, and record round-trip coverage for the primary navigation, sensor, covariance, and profiling payloads.
- [ ] Benchmark binary logging and telemetry throughput, storage footprint, CPU cost, and drop behavior against the existing desktop CSV/JSON path. Use that evidence to choose default logging paths per target profile rather than replacing CSV prematurely.
