# Phase 19 - Simulation Platform and Qualification Infrastructure

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase turns the simulator and analysis tooling into a broader platform after the core navigation, validation, embedded, and advanced-aiding capabilities are stable.

## Pass 19.1: multi-vehicle simulation

- [ ] Add multi-vehicle simulation support when scenario management, logging, and analysis outputs can represent multiple truth and estimate streams clearly.
- [ ] Define inter-vehicle timing, relative measurements, shared environment assumptions, and output naming conventions before implementation.

## Pass 19.2: hardware-in-the-loop integration

- [ ] Add hardware-in-the-loop interfaces only after embedded status/error handling, timing, allocation, and packaging expectations are mature.
- [ ] Define transport, clocking, data contracts, failure modes, and qualification evidence required for HIL runs.

## Pass 19.3: production scenario management and qualification reports

- [ ] Add production-grade scenario management for large scenario suites, parameter sweeps, and qualification campaigns.
- [ ] Generate automatic qualification reports that combine scenario manifests, config versions, binary/build metadata, validation metrics, plots, timing, and resource evidence.
- [ ] Keep qualification artifacts reproducible from committed configs, seeds, source revision, and tool versions.
