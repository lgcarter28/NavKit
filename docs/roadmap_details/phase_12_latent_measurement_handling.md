# Phase 12 - Latent Measurement Handling and Buffering

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Latency and buffering come before transfer alignment/stationary modes because those startup algorithms benefit from batch/history access and delayed/asynchronous measurement handling.

## Pass 12.1: measurement context snapshots

- [ ] When latent/replay measurement handling is implemented, carry per-measurement model context alongside buffered measurements. Current GNSS velocity lever-arm support updates sensor context immediately, which is sufficient for the current no-latency simulation loop but should not become the delayed-measurement architecture.
- [ ] Define the ownership boundary between Navigator buffering, measurement timestamps, observation context snapshots, and model-specific context such as GNSS antenna lever-arm state before adding delayed measurement processing.
- [ ] Add tests showing that delayed/replayed measurements use the context snapshot from the measurement time rather than mutable current sensor context.

## Pass 12.2: state/history buffering

- [ ] Add state history and delayed-measurement correction only after the context snapshot contract is explicit.
- [ ] Define buffer capacities, ownership, memory allocation rules, and timestamp lookup behavior for embedded-facing history buffers.

## Pass 12.3: smoothing and replay extensions

- [ ] Add RTS smoothing only after forward propagation/history interfaces stabilize.
- [ ] Keep smoothing/replay optional and clearly separated from the default real-time embedded navigation path.
