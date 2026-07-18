# Phase 15 - Embedded Readiness, API Hygiene, and Documentation

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase contains the remaining embedded/product-hardening work after status/error handling and profiling/resource validation have their own earlier phases.

## Pass 15.1: embedded deployment profile

- [ ] Define allocation, exception, RTTI, logging, and timing constraints for supported embedded profiles.
- [ ] Audit large fixed-size members and buffers, especially `RingBuffer` instances and state/history buffers, for stack-growth risk. Decide where large storage should remain embedded inline, move to static/global ownership, or be held behind pointers/references supplied by the application or target platform.
- [ ] Add embedded toolchain profiles and a hardware abstraction boundary when a target is selected.

## Pass 15.2: type and frame safety

- [ ] Extend existing unit/frame types based on observed misuse risks.
- [ ] Add compile-time DCM composition/result-frame checks where the added type machinery improves safety without obscuring Eigen interoperability.
- [ ] Add tested unit conversions and arithmetic only where they protect real boundaries.
- [ ] Keep frame/type abstractions zero-overhead and verify generated/runtime behavior where important.

## Pass 15.3: explicit type and API hygiene

- [ ] Continue the repository-wide `auto` audit using the strengthened AGENTS rule. Replace nontrivial `auto` in math, Eigen, state, frame/unit, simulator, and logging code with explicit types unless it falls into the narrow allowed cases.
- [ ] Keep the audit separate from functional navigation/simulation passes so style-only churn does not obscure numerical or architecture changes.

## Pass 15.4: documentation and API teaching material

- [ ] Define repository documentation-comment guidelines and update `AGENTS.md` so new C++ functionality eventually requires Doxygen comments on public and internal functions, classes, structs, namespaces, aliases, concepts, and policy boundaries; evaluate whether the same standard should apply to Python modules/functions.
- [ ] Add Doxygen/API documentation where it helps users understand policy contracts and math boundaries.
- [ ] Add documentation build profiles for different audiences/scopes, such as embedded/product-core API only, external user API docs, and full internal developer docs.
- [ ] Grow the navigation theory/reference manual alongside implemented equations.
- [ ] Add measurement-model and mechanization tutorials.
- [ ] Add end-to-end scenario walkthroughs and a developer architecture guide.
- [ ] Revisit ADR-001 through ADR-003 and either accept them, revise them, or keep them Proposed with explicit unresolved questions.
- [ ] Keep README, `docs/SETUP.md`, `docs/ARCHITECTURE.md`, `docs/CONFIGURATION.md`, and the active roadmap reconciled after ADR decisions or workflow changes.

## Pass 15.5: CI and release workflow hygiene

- [ ] Confirm hosted GitHub Actions runs pass on the supported platforms and document any local/CI differences.
- [ ] Add install/package validation: build, install, and run a minimal smoke scenario or executable from the install tree so packaged artifacts are tested separately from the developer build tree.
- [ ] Document the expected `build/`, `install/`, `output/`, and CI artifact layout for users and release workflows.
- [ ] Keep the owner-controlled release/tag/archive/backup workflow documented outside normal engineering tasks if it needs future maintenance.
