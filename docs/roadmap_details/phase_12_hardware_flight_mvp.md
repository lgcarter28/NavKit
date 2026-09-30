# Phase 12 - Hardware and Flight Minimum Viable Product

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

The milestone is one selected NavKit product running through SWIL, recorded-data
replay, mixed in-process HWIL, external-flight-computer HWIL, and live sensor
ingestion without changing estimator code. The
first sensor target is the already-owned capstone stack documented in Phase 10;
purchasing, custom avionics, and flight-control authority follow evidence rather
than lead it. The Arduino Mega and STM32F103 Blue Pills are useful sensor-bridge
and toolchain-smoke candidates, but their small memories are not assumed to fit
the complete current C++23/Eigen product. A measured map-file/resource result
decides that question.

## Existing-hardware opportunity and expected bottlenecks

The owned capstone hardware is sufficient to begin meaningful hardware work
without buying a new navigation sensor suite. It can support:

- raw, timestamped single-sensor capture through the Arduino Mega or an
  STM32F103 Blue Pill acting as a sensor/transport bridge;
- decoder, unit, axis, calibration, timestamp, logging, replay, and fault-path
  development for the MPU9250-family IMU/magnetometer, BNO055, BMP280-class
  barometer, MPXV7002DP-class differential-pressure sensor, and NEO-6M GNSS;
- host-executed live NavKit runs in which a bridge publishes measurements but
  the current workstation owns the estimator and analysis;
- long static IMU and environmental captures, Allan-deviation analysis,
  barometer/pitot characterization, loose-coupled GNSS proof, and early HWIL
  transport exercises.

The expected constraints are equally important:

- The Arduino Mega's memory is far below the expected requirement for the full
  fixed-size Eigen Navigator; use it as a proven legacy bridge, not as the
  product target.
- The STM32F103 Blue Pill is valuable for cross-compilation, map-file evidence,
  interrupt/timestamp experiments, and a deliberately minimal smoke product,
  but its memory is also likely too small for the complete selected product.
  Measure this rather than weakening the architecture to force a fit.
- Shared time, deterministic timestamps, transport latency, buffering, and
  replay equivalence are likely to expose more important architectural issues
  than the absolute accuracy of the old sensors.
- The NEO-6M is adequate for a low-rate loose-coupled position/velocity and
  transport proof, but it is not the reference receiver for future raw-
  observable, tightly coupled, multi-constellation, or disciplined-PPS work.
- Unknown board revisions, aged wiring, uncertain calibration, old batteries,
  and undocumented motor/ESC condition require current-limited electrical
  inspection before use. Retire and recycle the old 9 V batteries; do not use
  them as a flight-power source.
- The owned stack does not provide a high-grade truth reference. Early tests
  qualify acquisition, timing, replay, estimator stability, and relative
  behavior; they do not establish absolute navigation accuracy.

This makes the first hardware milestone deliberately modest but valuable:
capture real bytes with trustworthy timestamps, replay them through the same
adapter path, run NavKit live on the host, and prove deterministic behavior
before selecting new compute or sensors.

## Pass 12.1: Linux, WSL, and reproducible development environment

- [ ] Establish native Linux build/test parity, beginning with WSL 2 on the
  current Windows workstation and retaining direct Windows support while the
  workflow matures. Keep source and build trees in a location that avoids
  cross-filesystem performance penalties, and document line-ending, path,
  compiler, debugger, and Python/Conan behavior.
- [ ] Add a versioned development-container definition that can bootstrap,
  build, test, run deterministic simulations, and execute Python analysis from
  a clean host. Keep the container an optional reproducibility tool rather than
  a prerequisite for embedded targets or a place to hide undeclared host
  dependencies.
- [ ] Prove one USB/serial debug path from WSL 2 using `usbipd-win`, including
  device attach/detach, permissions/udev, reconnect behavior, and debugger or
  serial-port access. Retain a native-host adapter path when USB passthrough is
  unreliable; estimator and payload contracts must remain identical.
- [ ] Reconcile the container with CI and a future native-Linux workstation so
  the same pinned dependencies and commands work locally, in automation, and on
  a replacement machine. Measure the overhead on the current constrained laptop
  before making containerized analysis the default.

## Pass 12.2: target-independent transport and external-target seams

- [ ] Build on the Phase 8 target-neutral `MissionRuntime`, planned-time host,
  and `MissionAdapter` lifecycle rather than creating a second mission graph or
  host loop. Generated SWIL, replay, mixed HWIL, live bench, flight, and
  third-party hosts must use the same Navigation-phase contract without forcing
  synthetic truth into the embedded product.
- [ ] Extend the adapter boundaries only where concrete transports require new
  capabilities for clock synchronization, sensor acquisition, timestamping,
  publication, solution/telemetry output, health/status, shutdown, and
  target-specific phase-event mapping.
- [ ] Keep vendor protocols, operating-system APIs, serial/I2C/SPI/USB/CAN/UDP,
  and ArduPilot integration outside `navkit::core`. Adapters translate to stable
  timestamped NavKit payloads.

## Pass 12.3: embedded-core smoke target and resource baseline

- [ ] Add a small embedded-facing target that links only `navkit::core`,
  instantiates the selected Navigator/filter/sensor graph, runs a representative
  no-IO update, and rejects dependencies on simulation, JSON, filesystem, or
  desktop logging.
- [ ] Exercise the STM32F103C8T6 Blue Pill as the first no-cost cross-toolchain,
  sensor-bridge, and minimal-core smoke target; use the Arduino Mega only for
  legacy sensor/transport bring-up. Do not contort the product architecture to
  fit either board. In parallel, identify the cheapest modern MCU/dev board with
  sufficient headroom for the selected full Navigator product. Produce map
  files and report
  `.text`, read-only data, `.data`, `.bss`, stack/static objects, large symbols,
  exceptions, RTTI, allocation, compiler flags, and dead stripping.
- [ ] Define a minimal embedded profile for allocation, stack, exceptions,
  logging, timing, queue capacity, and failure behavior before running live
  hardware.

## Pass 12.4: replay and capstone-hardware bench integration

- [ ] Implement recorded-byte capture and replay through the exact same
  decoder, timestamp, measurement-event, and Navigator ingestion boundaries used
  by live hardware. Preserve raw data and conversion diagnostics.
- [ ] Bring up the inventoried IMU/magnetometer, barometer, pitot sensor, and
  u-blox receiver incrementally. Begin with static bench capture, one sensor at
  a time, then synchronized multi-sensor operation. Treat ultrasonic sensors,
  servos, motors, propellers, and reported ESCs as optional platform/actuator
  assets rather than dependencies of the navigation vertical slice.
- [ ] Add deterministic fault injection for dropout, latency, reordering,
  duplicate messages, timestamp jumps, saturation, checksum corruption, and
  transport interruption. Verify explicit degraded/failed behavior and replay
  equivalence.
- [ ] Demonstrate the first vertical slice in ascending risk: IMU-only static
  capture and replay; IMU plus magnetometer/barometer; GNSS plus PPS or the best
  available receiver time mark; pitot/differential-pressure characterization;
  and finally synchronized multi-sensor capture. Record which board, breakout,
  protocol, axis map, unit conversion, rate, timestamp source, calibration, and
  firmware revision produced every dataset.

## Pass 12.5: mixed in-process HWIL vertical slice

- [ ] Implement `navkit_hwil` as the mixed development-bench application. It
  continues to run Navigator in process while composing each sensor channel
  from exactly one emulator, real device/transport, or externally stimulated
  receiver. It may link selected reusable `navkit::sim` components but must not
  import `navkit_swil` or its concrete adapter.
- [ ] Run NavKit at planned real-time cadence against hardware or hardware-like
  transports while simulated trajectory, environment, and plant models supply
  controlled truth/stimulus where practical. Record deadline misses, queue
  depth, data age, drops, CPU, memory, estimator health, and solution error.
- [ ] Demonstrate the same committed scenario and selected product through
  simulated execution, recorded replay, and HWIL with target-specific adapters
  as the only architectural substitution.
- [ ] Demonstrate mixed source ownership explicitly, beginning with one real
  sensor and the remaining channels emulated. Validate exactly-one-producer
  bindings, target clock requirements, timestamp correlation, duplicate/stale
  publication rejection, transport interruption, and deterministic evidence.
- [ ] Define repeatable bench qualification procedures and machine-readable
  evidence rather than accepting visual plausibility as the HWIL milestone.

## Pass 12.6: external-flight-computer HWIL orchestration

- [ ] Add `navkit_hwil_orchestrator` when a concrete flight-computer target is
  available. The host owns real-time scenario/mission control, truth,
  environment and plant integration, emulators, external stimulus equipment,
  transport gateways, fault injection, monitoring, and evidence collection;
  it does not own the target Navigator update.
- [ ] Run `navkit_flight`, or the owning vehicle's equivalent production
  executable, on the real flight computer. The target must use its normal
  drivers, scheduler, telemetry, watchdog, persistence, and GNC interfaces and
  must not know that the external world is HWIL.
- [ ] Close the loop through normal sensor/stimulus inputs and normal
  telemetry/actuator outputs. Define deterministic startup, reset, arming,
  shutdown, missed-deadline, reconnect, and fault-containment behavior across
  host and target.
- [ ] Establish authoritative time correlation across the host, flight
  computer, devices, and stimulus equipment. Record clock offsets, jitter,
  latency, stale data, missed frames, commands, stimuli, measurements,
  telemetry, actuator outputs, binary/build identity, and target firmware in
  machine-readable evidence.
- [ ] Treat GNSS RF simulation (for example, a future Spirent-class source),
  analog/discrete IO, CAN, serial, Ethernet, PPS/PTP, and other equipment as
  focused orchestrator adapters added only when real hardware requires them.

## Pass 12.7: budget prototype trade study

- [ ] After legacy-hardware bring-up reveals the actual bottlenecks, perform a
  documented, budget-constrained trade study across IMU grade, magnetometer,
  barometer, differential pressure/air-data, GNSS receiver, MCU/SoC, storage,
  interfaces, clock/PPS support, power, availability, development ecosystem,
  licensing, and total prototype cost. Target a few-hundred-dollar incremental
  prototype budget, require an explicit decision before any single purchase or
  combined upgrade exceeds approximately $500, and keep workstation replacement
  as a separate higher-priority capital decision rather than silently consuming
  it through sensor experiments.
- [ ] Compare at least a minimum-cost stack, a balanced development stack, and
  a higher-confidence reference stack. Score timing observability, raw-data
  access, calibration support, documentation, supply risk, compute/memory
  headroom, and integration effort, not datasheet accuracy alone.
- [ ] Select one reference sensor and compute platform only after explicit
  mission, budget, update-rate, environmental, and flight-safety requirements
  are recorded.

The expected incremental purchase ladder is evidence-gated rather than one
large order:

1. **Bench visibility and electrical safety (approximately $40-$100 if not
   already owned):** a genuine supported SWD debugger such as an ST-LINK, a
   reliable 3.3 V/5 V USB-UART adapter, an inexpensive logic analyzer, and the
   required multimeter/current-limited supply, powered USB hub, connectors, and
   strain relief. These purchases improve every later integration and should
   precede speculative sensor upgrades.
2. **Representative embedded compute (approximately $30-$70):** prefer an
   STM32H7-class Nucleo, provisionally an STM32H743-class board, as the first
   full-product target with useful RAM/flash/FPU/timer/interface headroom. A
   Teensy 4.1 is a viable rapid-prototyping alternative, but the trade must
   account for ecosystem, debugger, build-system, and eventual flight-computer
   relevance rather than clock speed alone.
3. **Timing and power infrastructure (approximately $30-$80):** add clean
   regulated power, level shifting or isolation where required, robust
   connectors, and GNSS timing/PPS access once the first captures identify the
   actual timing limitation.
4. **Reference flight platform (approximately $200-$400, separately approved):**
   select a proven H7 ArduPilot/PX4-compatible autopilot and multicopter or
   companion-capable platform. Its initial role is safety controller and
   comparison reference while NavKit runs in shadow/companion mode.
5. **Sensor upgrades only after measured need (likely $100-$300 per meaningful
   step):** prioritize a modern multi-constellation receiver with raw
   observations and PPS if GNSS timing/observables are the limiting seam. Do
   not buy a nicer IMU merely because the legacy MEMS data are noisy; first use
   that noise to validate calibration, Allan analysis, stochastic models, and
   estimator robustness.

Before legacy bring-up is complete, target no more than roughly $100-$150 in
bench tooling and one modern MCU board. Preserve the remainder of the hardware
budget until resource maps, timestamp evidence, and real captures identify the
next bottleneck. A replacement Linux-capable workstation is a separate capital
priority and should not be disguised as sensor-program spending.

## Pass 12.8: graduated flight demonstration

- [ ] Progress through static bench, outdoor static, carried/vehicle ground
  test, restrained or captive test where applicable, and controlled flight.
  Define go/no-go criteria and evidence at each rung.
- [ ] Prefer a proven ArduPilot/PX4-compatible multicopter and autopilot
  ecosystem for the first airborne demonstration: it reaches controlled flight
  quickly, supports hover and diverse trajectories, and provides a mature
  reference solution and safety envelope. Integrate NavKit initially as a
  logging/shadow navigation payload or companion process, compare against the
  host solution, and avoid flight-critical control authority until evidence
  supports it. Do not let reuse of the old foam glider, A2212 motors, propellers,
  or unknown ESCs delay the first navigation flight.
- [ ] Add an ArduPilot/PX4-style adapter only at the transport/mode boundary;
  the external framework continues to own safety-critical flight control during
  the first demonstrations.
- [ ] Defer a custom UAV, rocket, custom PCB, or fully integrated flight-control
  computer until the bought-platform demonstration identifies requirements that
  cannot be met with development hardware. Treat rockets as a later, separately
  safety-reviewed test domain rather than the first flight target.

## Pass 12.9: reusable post-MVP hardware qualification gate

- [ ] Turn the replay/HWIL vertical slice into a reusable qualification harness
  for later capabilities. Preserve target, firmware, product/config, sensor,
  calibration, clock, transport, raw-capture, environmental, and test-procedure
  identities in machine-readable evidence so a result is reproducible rather
  than merely visually plausible.
- [ ] Define the inherited evidence ladder for hardware-relevant work:
  numerical/unit verification, deterministic simulation, stochastic analysis
  where applicable, recorded real-data replay, HWIL, and safe field/flight
  evidence. Each later pass selects the applicable tiers and declares its
  quantitative acceptance metrics before testing.
- [ ] Require replay and HWIL for later capabilities whenever owned or selected
  hardware can exercise them. When receiver observables, stimulus equipment,
  environment, budget, or safety make a tier infeasible, record the exact
  limitation, complete the highest feasible tier, and retain a traceable
  deferred acceptance item rather than silently waiving hardware validation.
- [ ] Establish regression baselines for the hardware exercised by this phase
  so later sensor models, alignment modes, GNSS algorithms, timing changes, and
  embedded targets can reuse the same capture/replay/HWIL evidence path.
