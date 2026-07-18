# Phase 17 - Additional Mechanizations and Environments

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase captures long-range navigation/environment expansion after the default ECEF INS/GNSS path, validation pipeline, sensor cleanup, latency handling, transfer alignment, profiling, embedded hardening, and advanced aiding phases are stable.

## Pass 17.1: additional mechanizations

- [ ] Add PCI/ECI mechanization support with a complete algorithm document before implementation.
- [ ] Add local-level and wander-azimuth mechanizations once the ECEF implementation and validation infrastructure are mature enough to compare behavior cleanly.
- [ ] Keep mechanization state definitions, frame conventions, and propagation-policy contracts explicit rather than hiding them behind vague runtime switches.

## Pass 17.2: expanded environment models

- [ ] Add atmosphere, magnetic-field, Earth-orientation, geoid, terrain, and aero/vehicle-dynamics policies driven by concrete use cases.
- [ ] Reuse existing planet, gravity, frames, and units infrastructure where it remains clear and zero-overhead.
- [ ] Add validation scenarios for each environment model before treating it as a supported product capability.

## Pass 17.3: multi-planet scenarios

- [ ] Extend multi-planet scenarios using the existing planet-policy direction.
- [ ] Keep planet-specific constants, gravity, rotation, frames, and scenario assumptions explicit in configs and documentation.
