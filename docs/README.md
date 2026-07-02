# NavKit Documentation

This index identifies each document's role and authority.

## Start here

- [`SETUP.md`](SETUP.md): prerequisites, environment setup, build/test commands, simulation, analysis, and the intended developer workflow.
- [`ARCHITECTURE.md`](ARCHITECTURE.md): current source layout, CMake target boundaries, namespaces, and data flow.
- [`CONFIGURATION.md`](CONFIGURATION.md): compile-time configuration mental model, domain config concepts, example config contracts, and selected-config workflow.
- [`TESTING.md`](TESTING.md): testing layers, design-intent standards, expected-failure coverage, coverage posture, and runtime timing/resource artifacts.
- [`PROFILING.md`](PROFILING.md): embedded profiling policies, fixed-capacity sinks, CSV export, and trace visualization workflow.
- [`ROADMAP.md`](ROADMAP.md): canonical current-state handoff, working roadmap, and dependency order.
- [`NAMING_CONVENTIONS.md`](NAMING_CONVENTIONS.md): navigation variable, frame, and unit naming.
- [`FOUNDING.md`](FOUNDING.md): stable mission and design values.

## Architecture decisions

The LaTeX ADRs under [`adr/`](adr/) describe proposed compile-time architecture:

- ADR-001: compile-time policy architecture;
- ADR-002: environment policy architecture;
- ADR-003: constrained policy template parameters.

All three are currently marked **Proposed**. They guide discussion but do not override the implementation as a description of current behavior. Their status should be updated deliberately when decisions are accepted or revised.

## Reference material

- [`navigation_reference/`](navigation_reference/) is the mathematical navigation reference in development.

## Source-of-truth rule

The checked-in implementation, active build configuration, and configured tests define current behavior. Accepted ADRs define settled architecture. The master roadmap records the verified handoff and planned sequencing.
