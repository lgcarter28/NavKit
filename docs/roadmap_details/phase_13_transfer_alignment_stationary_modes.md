# Phase 13 - Transfer Alignment and Stationary Modes

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

Transfer alignment and stationary aiding are scheduled after latent measurement handling because these algorithms benefit from history windows, batch processing, and explicit timing/context ownership.

## Pass 13.1: transfer alignment runtime model

- [ ] Implement the first useful transfer-alignment provider as timestamped aiding data rather than hidden direct state writes.
- [ ] Define PVA, angular-rate, and specific-force aiding sample semantics for transfer alignment.
- [ ] Treat transfer alignment as a runtime-loop participant from the simulation app's perspective. It should behave more like a scheduled general emulator/startup-aiding producer of timestamped aiding data than a one-shot initialization helper, while keeping the embedded Navigator API observation-driven.
- [ ] Add runtime scheduling windows for emulators and startup-aiding producers so a source can be active only over selected trajectory intervals, such as transfer alignment during the first 10 seconds.
- [ ] Define the app-loop behavior for transfer-alignment windows explicitly, such as `if transfer_alignment_valid(time) { generate/process transfer-alignment aiding } else { run the normal emulator and Navigator update loop }`, without baking that scenario timing into embedded product-core code.
- [ ] Add transfer-alignment runtime examples and tests that show construction, required PVA initialization, optional alignment aiding, and normal update processing remain distinct.

## Pass 13.2: alignment and stationary aiding modes

- [ ] Add coarse alignment for gravity-vector tilt estimation during stationary startup.
- [ ] Add fine alignment / gyrocompassing support for yaw observability when stationary and Earth-rate observability is meaningful for the configured IMU grade and scenario.
- [ ] Add zero-velocity updates (ZUPTs) to constrain velocity error/covariance growth during stationary intervals and improve IMU bias observability during initialization.
- [ ] Define how startup/alignment/ZUPT event sequencing is configured. Prefer runtime sequencing for scenario timing and mode activation, while keeping the compiled algorithm set and embedded policy graph compile-time selected.
- [ ] Add stationary startup scenarios that exercise PVA initialization, optional transfer alignment, coarse alignment, fine alignment, ZUPTs, and transition into normal ECEF INS/GNSS operation.
