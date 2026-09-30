# Phase 10 - Flight-Relevant Sensor Models and Hardware-Aiding Cleanup

**Status:** future backlog detail. Current active ownership is `docs/ROADMAP.md`.

This phase prepares the existing loosely coupled INS and the owner's available
capstone hardware for real ingestion work. It deliberately prioritizes the
already-owned low-grade MEMS IMU/magnetometer, barometer, pitot tube, and
u-blox-class GNSS receiver before purchasing a replacement prototype stack.
Sensor algorithms, calibration, units, validity, timestamps, and simulation
models are established here; target transports and live execution belong to
Phase 12.

The planning baseline below was reconciled against the surviving 2018-2019
capstone BOM, final report, Arduino firmware, MATLAB analysis, recorded test
data, calibration workbooks, and current photographs. It records evidence of
ownership and historical use, not present-day electrical health:

- The strongest historically integrated stack was an Arduino Mega 2560 with an
  MPU9250-family IMU/magnetometer, BMP280-class barometer, microSD logging, and
  SG92R servos. The archive contains integrated controller firmware, recorded
  ground/flight data, and flight-test discussion.
- The differential-pressure assembly is a DIYDrones Pressure Board v2.0 using
  an MPXV7002DP-class sensor and pitot probe. Standalone acquisition code and
  calibration workbooks exist, but integration into the final flight stack is
  not established.
- A photographed GY-GPS6MV2 board contains a u-blox NEO-6M-0-001 receiver and
  patch antenna. The BOM also lists an Adafruit Ultimate GPS, and the surviving
  standalone GPS sketch uses Adafruit/PMTK commands; these are distinct receiver
  paths and must not be conflated during adapter work.
- Current photographs also identify an Adafruit BNO055 breakout, an
  MPU-9250/9255-family breakout, at least one BMP/BME280-family breakout,
  STM32F103C8T6 Blue Pill boards, an Arduino Mega, microSD modules/cards and a
  USB reader, HC-SR04 and US-015 ultrasonic modules, SG92R servos, and four
  A2212/13T 1000 KV motors with propellers. The owner's reported Uno, Micro,
  ESCs, and additional duplicate parts remain inventory claims until found and
  photographed.
- The archived BOM records one Mega, two STM32F103C8T6 boards, two MPU9250s,
  two BMP280s, one NEO-6M, one Adafruit GPS, five microSD modules, two microSD
  cards, eight SG92R servos, five HC-SR04s, one US-015, one pitot/pressure
  assembly, and three foam gliders. These quantities are acquisition evidence,
  not a current physical count.
- The second-semester archive contains offline complementary-filter, attitude
  Kalman-filter, and position/velocity Kalman-filter work over recorded 20 Hz
  logs. These artifacts are useful replay and interface references, but they are
  not accepted navigation algorithms or numerical baselines for NavKit.
- The old 9 V batteries are consumables, not reusable flight power. Inspect and
  recycle them as appropriate; begin bring-up from current-limited USB/bench
  power and select a regulated rechargeable flight-power system later.

## Pass 10.1: legacy-hardware inventory and data-contract capture

- [ ] Inventory the available capstone hardware from source, schematics,
  datasheets, firmware, logs, photos, and part markings. Record exact part
  numbers, board revisions, interfaces, message formats, rates, clocks,
  calibration storage, voltage/logic requirements, and known failure history;
  do not infer a device solely from an old project label. Reconcile physical
  quantities against the archived BOM and explicitly classify each item as
  found, photographed, electrically healthy, communicating, calibrated, or
  historically documented only.
- [ ] Create a concise hardware manifest and raw-data capture plan. Preserve
  original bytes plus host receive time and device time where available, so
  later decoder or timestamp changes can be replayed without reacquiring data.
  Preserve selected legacy logs as provenance-labeled import/replay fixtures
  only after their column meanings, units, timestamps, and ownership are
  documented; do not copy historical algorithms into the product core.
- [ ] Define target-independent timestamped input payloads for the available
  IMU/magnetometer, barometer, pitot/air-data, and receiver-level GNSS outputs.
  Keep serial/I2C/SPI/vendor parsing outside measurement models and outside the
  embedded estimator core.
- [ ] Establish bench acceptance tests for power-up, communications, sustained
  capture, monotonic timestamps, rate/jitter, saturation, dropout, and
  stationary plausibility before treating any legacy sensor as trustworthy.
  Start with visual inspection, continuity/voltage checks, and one device at a
  time. Treat seven-year storage, unknown ESD/over-voltage exposure, crashed
  airframe history, clone-board variation, and unbalanced or damaged rotating
  hardware as explicit risks.

## Pass 10.2: loosely coupled GNSS cleanup and u-blox-class receiver support

- [ ] Split loosely coupled GNSS aiding into its own complete LaTeX algorithm
  document before major refactors. Cover receiver-level position/velocity
  observations, lever arms, valid flags, covariance frame transforms, combined
  position/velocity updates, Jacobian structure, timing, logging/validation
  expectations, and the intended default configuration.
- [ ] Add a combined GNSS position/velocity emulator output and measurement
  model as the preferred default path. Reuse existing machinery, but process a
  full valid observation in one Kalman update so cross-coupling is not erased by
  an artificial reset between same-epoch position and velocity updates.
- [ ] Support position-only, velocity-only, and combined valid subsets by
  selecting the valid residual/Jacobian/covariance rows. Do not retain padded
  invalid rows or zero them and assume the statistics remain equivalent.
- [ ] Add a receiver adapter and replay fixture for the identified legacy
  u-blox NEO-6M output. Keep UBX/NMEA decoding separate from both the generic
  GNSS observation contract and the archived Adafruit/PMTK experiment; retain
  raw messages for decoder regression. Use the NEO-6M for serial, timestamp,
  replay, and loosely coupled proof-of-concept work, not as the long-term raw
  observable receiver for tightly coupled GNSS.
- [ ] Add multi-receiver examples and tests with a nonzero lever arm in at
  least one default qualification scenario.

## Pass 10.3: magnetometer aiding and calibration

- [ ] Add a complete LaTeX algorithm document covering magnetic-field model
  requirements, body-frame mounting, hard/soft iron calibration, misalignment,
  covariance/error models, disturbance detection, innovation acceptance, and
  yaw-observability limits.
- [ ] Add magnetometer simulator/emulator output and runtime configuration,
  including both available paths: the MPU9250-family embedded magnetometer and
  the separately photographed Adafruit BNO055. Verify which exact IMU variant
  is present and do not treat the BNO055's fused orientation output as raw
  magnetometer truth or as an independent NavKit attitude solution.
- [ ] Add the magnetometer measurement model and Jacobian for attitude/yaw
  aiding where the selected field model and scenario make it meaningful.
- [ ] Add bench calibration and validation workflows: stationary orientation
  sweeps, hard/soft iron fitting, residual plots, repeatability, local magnetic
  disturbance detection, and recorded-data replay.

## Pass 10.4: atmosphere, barometer, and pressure-altitude aiding

- [ ] Add shared atmosphere-model requirements before pressure-derived sensor
  implementation. Define pressure, density, temperature, altitude, frame,
  units, reference pressure, weather assumptions, and runtime configuration for
  both barometric altitude and pitot/air-data models.
- [ ] Add a complete LaTeX algorithm document for static pressure and
  barometric altitude aiding.
- [ ] Implement the barometer simulator and replace the placeholder altitude
  model with a physically meaningful ECEF/local-vertical measurement model and
  Jacobian.
- [ ] Add an adapter, static bench characterization, temperature/zero-offset
  evaluation, and replay fixtures for the BMP280-class capstone barometer.
  Resolve the physical breakout's BMP-versus-BME part identity during inventory
  rather than relying on interchangeable marketplace board markings.

## Pass 10.5: pitot tube and air-data support

- [ ] Add a complete LaTeX algorithm document for pitot/static pressure,
  dynamic pressure, airspeed, angle of attack, sideslip, wind, and relative-air
  modeling before implementation.
- [ ] Reuse the Phase 10.4 atmosphere contract and add a pitot/air-data model
  with clear frame, unit, covariance, noise, validity, saturation, and plumbing
  assumptions.
- [ ] Add optional wind-state estimation and measurement Jacobians with respect
  to navigation velocity, attitude, calibration terms, and selected wind
  states only when the configured observations can support them.
- [ ] Add adapter and bench-flow characterization for the capstone pressure
  transducer/pitot assembly, provisionally identified as an MPXV7002DP-class
  DIYDrones Pressure Board v2.0. Reconcile the archived calibration workbook
  and standalone ADC firmware before defining a new calibration. Clearly
  distinguish sensor verification from aerodynamic calibration that requires a
  known flow source.
- [ ] Add maneuvering validation scenarios with enough excitation to make
  air-data and wind observability meaningful.

## Pass 10.6: IMU output compensation contracts

- [ ] Give the precompensated IMU simulator selection a real numerical
  coning/sculling path with explicit sample grouping, output cadence, and
  terminal partial-group policy; it must not remain a metadata-only promise.
- [ ] Enforce and test exactly one coning/sculling compensation owner
  (simulator or Navigator), including simulator-on/Navigator-off and
  simulator-off/Navigator-on equivalence and deliberate rejection of double or
  missing compensation where the selected contract requires it.

## Pass 10.7: Allan-deviation-derived IMU stochastic models and verification

- [ ] Thoroughly extend the IMU emulator LaTeX reference before implementation.
  Define Allan variance/deviation, averaging time, log-log slope and coefficient
  conventions, units, sample-rate dependence, and the direct mapping from each
  documented Allan term to the simulator process. Cover gyro and accelerometer
  quantization, angle/velocity random walk, bias instability,
  rate/acceleration random walk, and rate ramp; explicitly defer correlated and
  sinusoidal terms.
- [ ] Keep one configuration source of truth. Runtime JSON and C++ simulator
  configuration retain only directly consumed stochastic-process parameters
  with documented Allan meanings. Add focused unit/coefficient conversion
  utilities only where an input boundary needs them; do not maintain parallel
  manufacturer-specification, Allan-fit, and simulator parameter trees.
- [ ] Implement independently selectable, seeded stochastic components with
  simulator-owned random engines, explicit composition order, initialization,
  persistent state, sampling assumptions, and units.
- [ ] Add component-isolation tests for replay, sample-rate scaling,
  zero/bypass behavior, units, axis covariance/coloring, and long-run numerical
  stability. Keep the ES-EKF gyro/accelerometer-bias state definition unchanged
  while allowing deliberately richer unmodeled simulator truth.
- [ ] Build one reusable Python overlapping-Allan analysis path that recovers
  configured coefficients and slopes with declared tolerances and finite-record
  limits; share derivations between automated checks and renderers.
- [ ] Add long-duration seeded verification for every supported IMU config and
  isolated component, producing machine-readable results and
  publication-quality realized-versus-configured plots. Use Monte Carlo only
  when confidence characterization warrants it.
- [ ] Include representative regenerated plots and governing equations in the
  LaTeX reference/report, with sample rate, duration, seed, units, and exact
  regeneration command stated.
- [ ] Add matched and deliberately mismatched navigation cases to quantify when
  bias-only ES-EKF process-noise tuning absorbs richer errors. Add estimator
  states only after evidence demonstrates a material, observable benefit.
