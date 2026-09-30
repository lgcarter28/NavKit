# NavKit Architecture

This document describes the architecture that exists in the repository today.
The ADRs under `docs/adr/` describe proposed direction and design rationale, but
they do not override the checked-in implementation.

## Product boundaries

NavKit is split by product boundary first, then by engineering domain.

| Boundary | CMake target | Header root | Role |
|---|---|---|---|
| Product core | `navkit::core` | `include/navkit/core` | Reusable estimation/navigation framework and domain models |
| Simulation support | `navkit::sim` | `include/navkit/sim` | Desktop simulation infrastructure and generated measurements |
| IO support | `navkit::io` | `include/navkit/io` | Desktop logging, files, CSV, JSON, and run manifests |
| Target-neutral application support | `navkit::app_support_common` | Target-neutral headers under `include/navkit/app_support` | Header-only mission graph/runtime, adapter lifecycle, clocks, and runtime-input helpers that do not depend on simulation |
| SWIL application support | `navkit::swil_support` | `include/navkit/swil` | Header-only selected-config SWIL composition; depends on `navkit::app_support_common` and `navkit::sim` while reusing app-support emulation, initialization, and logging components |
| Applications | app targets under `apps/` | `apps/` | Executable entry points that compose the libraries they need |
| Analysis | Python package under `python/` | `python/navkit_analysis` | Offline CSV/HDF5 analysis, plots, and validation |

The root `CMakeLists.txt` is intentionally an orchestration layer. Header-only
and interface target definitions live under `cmake/targets/`, while compiled
targets live next to the source files they build:

```text
cmake/targets/NavKitCore.cmake   navkit_core / navkit::core
cmake/targets/NavKitIo.cmake     navkit_io / navkit::io
src/sim/CMakeLists.txt           navkit_sim / navkit::sim
src/app_support/CMakeLists.txt   navkit_app_support_common / navkit::app_support_common
                                 navkit_swil_support / navkit::swil_support
```

## Header and source layout

```text
include/navkit/
  core/
    config/
    time/
    containers/
    estimation/
      state/
      measurement/
      filter/
        injection/
        reset/
      sensor/
        noise/
      navigator/
        PVA/TXA state vocabulary
        update/
        propagation/
    environment/
      planet/
      gravity/
    frames/
    math/
    units/
    models/
    profiling/

  sim/
    sensors/
    guidance/
    autopilot/
    trajectory/
    math/
  io/
    log_payloads/
    log_products/
  app_support/
    config/
    emulation/
      concrete/
    execution/
    mission/
    navigation/
    runtime/
    simulation/
    time/
    initialization/
    logging/
    profiling/
    trajectory/

config/
  compiletime/
    navkit/
    apps/
      navkit_swil/
  runtime/
    navkit/
      components/
      scenario/

src/
  sim/
    sensors/
    guidance/
    autopilot/
    trajectory/
    math/
  app_support/
```

`include/navkit/core` is the reusable product core, not a miscellaneous bucket.
Simulation, desktop IO, concrete app/product compile-time configurations, and
executable runtime input bundles are intentionally outside the core boundary.
Future domains such as atmosphere, magnetic-field, geoid, and terrain policies
should be added when concrete implementations land; the repository should avoid
empty placeholder directories for planned architecture.

## Namespaces

Public namespaces mirror the folder structure through the stable domain level.
Deeper leaf folders may organize implementation and policy families without
adding additional namespaces unless that subdomain becomes independently
meaningful. For example, `include/navkit/core/environment/gravity/J2.hpp`
currently contributes `navkit::core::environment::J2`, not
`navkit::core::environment::gravity::J2`.

| CMake target | Primary namespace | Notes |
|---|---|---|
| `navkit::core` | `navkit::core` | Common product-core foundational aliases such as `Scalar_t` and `Time_t` |
| `navkit::core` | `navkit::core::config` | Shared product-core compile-time configuration vocabulary |
| `navkit::core` | `navkit::core::containers` | Product-core containers |
| `navkit::core` | `navkit::core::estimation` | State definitions, measurements, filters, sensors, navigators, and estimator policies |
| `navkit::core` | `navkit::core::environment` | Planet and gravity policies |
| `navkit::core` | `navkit::core::frames` | Frame tags and frame-typed helpers |
| `navkit::core` | `navkit::core::models` | Reusable product-core measurement and process models |
| `navkit::core` | `navkit::core::profiling` | Zero-overhead-by-default profiling vocabulary, policies, and fixed records |
| `navkit::core` | `navkit::core::units` | Unit and frame helper types |
| `navkit::sim` | `navkit::sim` | Simulation support |
| `navkit::io` | `navkit::io` | Logging, CSV, JSON, and run manifests |
| `navkit::app_support_common` | `navkit::app_support` | Target-neutral mission/runtime and adapter-boundary templates that are not product-core API |
| `navkit::swil_support` | `navkit::swil` and `navkit::app_support` | SWIL-selected executable support layered on common application support and simulation |

Shared compile-time product-core configuration vocabulary lives under
`navkit::core::config`. Domain-specific configuration concepts live beside the
domain that consumes them; for example, estimator sensor-buffer and
measurement-statistics configuration concepts currently live under the
`navkit::core::estimation` namespace.

## Time and scheduling boundaries

`core::Time_t` remains the scalar-seconds type used by physical equations,
mechanization, and covariance math. Public time-bearing messages instead use
`core::Timestamp`: a versioned, explicitly scaled absolute time point with
`Seconds s` and normalized `Nanoseconds ns` fields. Its wire
version is the first field so telemetry/transport code can deserialize the
format before interpreting the remaining fields. Timestamp serialization must
write fields explicitly; the C++ struct layout is not a wire format.

`core::Duration` is a non-negative elapsed interval with the same `s`/`ns`
storage convention. Boundaries validate time
version, scale, ordering, and nanoseconds with integer second/nanosecond
subtraction; only the resulting duration is explicitly converted to `Time_t`.
Navigation code must not derive an interval by subtracting scalarized absolute
timestamps. `TimeScale` identifies `Monotonic`, `Utc`, `Gps`, or `Tai`; the
current Navigator requires matching scales and does not silently transform
between them.

Simulation cadence uses `core::RationalRate` and a 64-bit sample index rather
than repeated floating-point additions. Runtime JSON still accepts one of
`rate_hz` or `dt_s`, then canonicalizes it once. `RationalSchedule::due(t)` is
the consumer-side cadence gate, while `RationalTimeline::next(t)` produces the
next exact planned timestamp strictly after the timeline epoch.

`sim::TrajectorySource` is a simulation-only virtual boundary. It makes truth
available through `advance_to(t)` and serves bounded exact-time `query(t,
sample)` calls without extrapolation. `StationaryTrajectorySource` produces
truth lazily; `TabulatedTrajectorySource` wraps generated or CSV-backed
`TruthTrajectory` storage, which retains native samples and uses linear ECEF
state interpolation plus quaternion SLERP. This keeps the reusable product-core
navigation path statically composed while allowing application/simulation code
to select a source at runtime. `GeneratedTrajectorySource` is the controlled
dynamic implementation: it owns a simulation-only `GuidanceModel`,
`AutopilotModel`, `VehicleResponseModel`, and canonical ECI plant state behind
the same truth-source interface.

These collaborators have explicit ownership domains. Reusable Guidance
algorithms, typed command blocks, and the runtime `GuidanceStateMachine` live
under `sim/guidance`; Autopilot attitude/rate tracking lives under
`sim/autopilot`; the generated source, ECI truth integration, and Vehicle/plant
response remain under `sim/trajectory`. The common `GuidanceCommand` payload is
the deliberately narrow data boundary from Guidance to Autopilot: filtered
body-resolved specific force plus NED bank command. State-machine execution
flags and logging/realization diagnostics travel in separate producer-output
structures rather than expanding that consumer contract. Vehicle response is
not an Autopilot implementation detail: Autopilot produces controller
body-rate response, while the Vehicle independently applies plant body-rate
and specific-force response before truth integration.

Guidance consumes `sim::TrajectoryControlState`, not truth objects or Navigator
internals. The generated source adapts that selected state once into the
focused `sim::AutopilotState` needed for attitude/rate tracking: body-to-ECI
attitude, body inertial rate, NED velocity, and the ECI-to-NED DCM. Separate
`AutopilotExecutionState` flags state whether tracking is active or the launch
attitude must be held. The Autopilot does not receive plant position,
acceleration, gravity, or other trajectory-environment fields it does not use.
The SWIL mission adapter selects the controller state source at runtime:
`"navigation_estimate"` is the closed-loop default, while
`"truth_passthrough"` is an explicit analysis/reference override. The choice is
translated at the application boundary; controller implementations never
inspect which source was selected. After `Navigator::update()`, the app
publishes the latest timestamped navigation-control snapshot for the next
applicable controller tick. Current semantics deliberately use the latest
available estimate without delayed-state replay.

The generated source uses exact independent Physics, Autopilot, and Guidance
schedules. Target-neutral `MissionRuntime` owns one authoritative ordered phase
array, stable phase IDs, checked transition indices, and the active phase. Each
phase owns its Navigation, Guidance, and Autopilot selection
inline or through an explicit reference; app support does not join parallel
subsystem phase catalogs. Phase-owned Guidance payloads compose typed reference,
acceleration, and bank blocks, while mission-level GNC configuration and the
simulation model separately own persistent controller tuning and plant behavior.
Trajectory names such as ballistic or waypoint no longer select bespoke C++ Guidance classes. Guidance
emits total inertial acceleration plus a NED bank
intent in a `GuidanceOutput`; the physical plant boundary converts that intent
to a complete minimal `GuidanceCommand`. A persistent Guidance-output filter
independently shapes body-X/Y/Z specific force and bank using global,
state-nominal, or temporary state-entry time constants without resetting its
command state. Runtime diagnostics expose the active mission-phase index rather
than coupling other subsystems to a Guidance-specific state identity.
Autopilot forms attitude and body-rate commands, consuming the actual
configured IMU increments through a fixed-capacity moving-window
`sum(delta_theta) / sum(dt)` observation. Vehicle response owns final body
rate, its two internal specific-force response stages, post-response limits,
and the conversion back to realized ECI acceleration. Only the ECI plant
integrates truth. This runtime-polymorphic simulation boundary is intentionally
outside the statically composed embedded product core.

Sensor-specific Navigation actions live directly in each mission phase, either
inline or through a reference into `components/gnc/navigation`. Those actions do
not enter `GuidanceCommand`, the Autopilot, the Vehicle, or embedded NavKit
policy configuration. `MissionRuntime` validates each adapter-reported phase
transition and applies the destination Navigation action before committing its
active phase. Current actions control per-sensor innovation gates and optional
chi-square acceptance-probability overrides; phase timing remains application
orchestration.

Guidance blocks that author acceleration relative to NED or ECEF apply the
appropriate local-frame transport, Coriolis, and centripetal terms before
publishing the canonical total inertial acceleration in ECI; a DCM-only
rotation is valid only when the source quantity is already an inertial
acceleration. Ballistic coast behavior is explicit in its configured state:
zero nongravitational force plus active Autopilot produces a velocity-aligned
gravity turn, while the same force command with inactive Autopilot permits
free inertial attitude propagation.

Guidance and Autopilot outputs are zero-order held between their exact producer
epochs. Runtime diagnostics sampled at Physics or logging cadence repeat the
held values until the next producer update rather than interpolating commands
or implying extra controller executions.

`MissionApp` is the target-agnostic host loop. It owns the planned master cadence
selected by the runtime `execution_target` component and a matching app-support
`Clock`, while a shallow runtime-polymorphic `MissionAdapter` owns target-specific
pre-deadline preparation, at-deadline publication, and post-navigation feedback.
The `navkit_swil` executable remains a thin selected-config launcher and selects
the fixed `navkit::swil::SwilMissionAdapterFactory` for that host boundary. The
factory's target identity is checked against runtime JSON before SWIL-specific
configuration is parsed, and the compile-time product config does not repeat that
choice. Its configured
execution rate must be an integer multiple of every synthetic producer rate, so
each consumer deadline is visited exactly. At each planned timestamp, the SWIL
adapter advances the source and prepares synthetic emulator updates before the
deadline. The host then calls `wait_until(t)`, synchronizes the adapter-reported
stable mission-phase ID, publishes prepared updates to Navigator-visible queues,
and invokes `Navigator::update()`. The simulated clock adopts planned time
immediately; the real-time clock maps the same API to a steady-clock deadline for
future HWIL use. Synthetic preparation may run ahead of the wall clock, but
phase application, publication, and all real hardware acquisition remain
post-deadline.

`Clock` is intentionally virtual only in app support: `SimulatedClock` adopts
planned time immediately, while `RealtimeClock` waits against a steady-clock
deadline. The selected `"simulated"` or `"realtime"` mode belongs to the runtime
`execution_target`, not an embedded NavKit policy. Exact planned timestamps come from `RationalTimeline`;
consumer-side `RationalSchedule` remains solely the due-time gate for log and
synthetic-emulator cadences.

At each planned application epoch, the source first receives the latest
selected controller-facing state and advances Physics through that exact time.
Synthetic emulators then prepare observations from available truth. After
`Clock::wait_until(t)`, the app publishes prepared sensor data, routes the same
published IMU increment to both Navigator and the source's Autopilot observation
path, invokes `Navigator::update()`, and refreshes the closed-loop control-state
snapshot. Preparation may run ahead of a real-time deadline; publication and
controller observation of sensor data may not.

The time vocabulary is split by dependency: `TimeTypes.hpp`, `Timestamp.hpp`,
`Duration.hpp`, `RationalRate.hpp`, `RationalSchedule.hpp`, and
`RationalTimeline.hpp`. `Time.hpp` is a
convenience umbrella, while production headers include the narrowest dependency
they use. `SignedDuration` remains deferred until a real latency/replay or
timestamp-offset boundary requires signed intervals.

The public config API include boundary is intentionally narrow.
`include/navkit/api/config/ConfigApi.hpp` collects shared product-graph
vocabulary, required contracts, and defaults exposed directly by primary core
template boundaries. Concrete product configs include `ConfigApi.hpp` plus the
specific model, profiler, target, or component headers they select. It should
not become a universal include for every concrete component choice.

See [`CONFIGURATION.md`](CONFIGURATION.md) for the user-facing configuration
mental model, example config contracts, and the selected-config build workflow.

Runtime scenario inputs for executables live outside public headers under
`config/runtime/navkit`. This avoids mixing "what product are we compiling?"
with "what scenario are we running today?" It also avoids overly ceremonial names
such as `navkit::core::environment::planet::Wgs84` until a leaf domain becomes
independently meaningful.

Runtime composition is one explicit object graph. A moderately complex object
can be authored inline or referenced with `{"config": "relative/path.json"}`
and optional explicit `overrides`. Paths resolve relative to the containing
file, references resolve recursively, and cycles or ambiguous mixed
reference/inline objects fail validation. The resolved runtime artifact is
self-contained; app support never performs a hidden join of disconnected
catalogs.

Ownership is deliberately narrow:

- `mission` owns one ordered phase graph, transitions, mission commands, and
  each phase's resolved Navigation, Guidance, and Autopilot selections. It may
  link persistent GNC implementation/tuning objects, but it does not own
  synthetic truth, plant implementation, clocks, or logging.
- `simulation` owns synthetic initial truth, trajectory-source selection,
  environment, dynamics/integration, plant models, and truth-versus-navigation
  feedback wiring. Its `phase_behavior` mapping is keyed by stable mission phase IDs and
  must match the mission exactly; missing and orphaned mappings are errors.
- `execution_target` owns the application adapter, clock, and planned cadence.
  The NavKit-owned `swil` adapter is implemented. `hwil` is recognized but
  fails closed until an application build supplies its real transport/runtime adapter.
- `scenario` is the thin final composition root selecting mission, target,
  simulation, sensor models, estimator initialization, logging, and explicit
  local overrides.

This separation avoids parallel mission and scenario trees: one mission can be
reused by multiple scenarios, simulation models, sensor suites, and execution targets.
Internal trajectory-source, truth-trajectory, and trajectory-analysis types keep
their domain names; `mission` is an application-level phase plan, not a replacement
for trajectory mathematics.

Future mixed HWIL, external-flight-computer orchestration, flight, or
external-framework integrations should be separate thin executables and
`MissionAdapter` implementations rather than conditionals embedded throughout
`MissionApp`. Shared mission, Navigation, filter, initialization, sensor-semantic,
and logging components remain reusable; each scenario root adds only its target's
simulation/emulator or hardware/transport/stimulus graph. A mixed-HWIL adapter may
reuse selected `navkit::sim` models, but it must not depend on `navkit::swil` or the
`navkit_swil` executable. For example, an ArduPilot adapter can map externally owned
mission/mode events to the same Navigation phase contract, feed hardware or
transport-provided measurements to `Navigator`, and return the navigation
solution without constructing synthetic truth or a `TrajectorySource`.
Other external target names remain unsupported runtime values until their
transport and lifecycle contracts are implemented. `hwil` is the one recognized
but unavailable value: validation and adapter construction fail closed until an
application build supplies its concrete transport/runtime implementation.

## Target kinds

Use CMake target kinds honestly:

- `navkit::core` is currently an `INTERFACE` target because the core is
  header-only and template-heavy. It carries include directories, C++23 compile
  features, and the Eigen usage requirement.
- `navkit::io` is currently an `INTERFACE` target because IO support is still
  header-only. It carries the desktop JSON dependency.
- `navkit::sim` is a compiled library because simulator implementation sources
  live under `src/sim/`. Sensor simulators are grouped under `sensors/`, while
  runtime Guidance and Autopilot own top-level `guidance/` and `autopilot/`
  domains. Trajectory truth integration and Vehicle/plant response stay under
  `trajectory/`; shared simulation math lives under `math/`.
- `navkit::app_support_common` is an `INTERFACE` target for target-neutral
  mission/runtime and adapter support. Its exported link interface is limited to
  `navkit::core` and `navkit::io`; a focused common-header compile smoke protects
  that selected neutral boundary from a `navkit::sim` dependency.
- `navkit::swil_support` is an `INTERFACE` SWIL umbrella layered on
  `navkit::app_support_common` and `navkit::sim`. It provides selected-config
  synthetic emulation, trajectory, initialization, logging, and profile-export
  composition. Concrete application entry points remain thin selected-config
  dispatchers.

Do not add dummy `.cpp` files merely to force a static archive. Convert an
`INTERFACE` target to a compiled/static library when the component owns
meaningful `.cpp` implementation.

Target-definition placement follows the same rule. Header-only/interface target
definitions belong under `cmake/targets/` because they do not own local source
files. Compiled targets belong beside their implementation sources, such as
`src/sim/CMakeLists.txt`. The app-support target definition currently lives
under `src/app_support/CMakeLists.txt` to keep that boundary visible; it can
move under `cmake/targets/` if it remains purely header-only.

## Manifest ownership

Python tooling currently orchestrates build commands and writes the outer
`navkit_build_manifest.json` because it knows wrapper-level facts such as build
type, selected build directory, elapsed build time, and resource reports. The
selected-config portion of that manifest comes from C++: after building,
`tools/build.py` asks the executable to describe its compiled config. Runtime
application manifests and log metadata are written by C++ application/IO code.
Application entry points should stay selected-config generic where practical.
The selected app config composes a reusable NavKit library config with app-side
sensor bindings, emulator policies, navigation initialization providers, and
optional transfer-alignment providers. `navkit::app_support_common` owns the
target-neutral mission graph/runtime and adapter lifecycle, while
`navkit::swil` owns the selected SWIL composition layered over it. Focused
subdirectories retain app compile-time config vocabulary, generic emulator binding
and runtime dispatch, concrete emulators, JSON/runtime validation, PVA startup
initialization and transfer-alignment seams, app-side logging adapters, profile
export, and trajectory providers. IO log products and typed payload wrappers live under
`include/navkit/io/log_products` and `include/navkit/io/log_payloads` so the
serialization boundary is explicit without forcing logging into product-core
code. `app_support::RuntimeLogger<NavKit>` centrally defines the desktop
simulation product list and binds NavKit-dependent log-product types.
`navkit::io::RunLogger<...>` remains the small compile-time façade that owns
output-directory setup, product open/flush/close orchestration, per-product
metadata files, and the run manifest, while concrete products own CSV schemas
and payload serialization. Concrete app compile-time configs do not repeat the
desktop logging tuple. Runtime JSON owns whether each known product is enabled
and its cadence; this keeps nonembedded logging selection out of the NavKit
product config.

This boundary avoids checked-in sidecar metadata that can drift from the actual
compiled configuration. If build-manifest writing moves fully into C++ later,
preserve that rule: compiled C++ should be the source of truth for compiled
configuration facts.

## Current data flow

The current simulation supports stationary and dynamically generated ECEF
INS/GNSS scenarios:

```text
selected control state (Navigator estimate by default)
    -> Guidance acceleration/bank command
    -> persistent body-specific-force/bank command filter
    -> Autopilot attitude/body-rate command and response
    -> Vehicle body-rate/specific-force response
    -> ECI truth plant
    -> IMU and GNSS simulators
    -> Navigator buffers and ECEF INS propagation
    -> KalmanFilter covariance/measurement updates
    -> latest closed-loop control-state snapshot

truth + diagnostics + estimates
    -> runtime-selected CSV/JSON products
    -> Python CSV/HDF5 analysis and plots
```

The working filter remains a loosely coupled ECEF INS with GNSS position and
velocity aiding. Later roadmap phases add richer aiding, history/replay,
tightly coupled observables, and higher-fidelity vehicle dynamics.

## Current implementation boundaries

- `KalmanFilter` performs measurement updates, propagates covariance from
  discrete `Phi`/`Qd` inputs, stores optional per-model statistics, and
  delegates injection/reset.
- `Navigator` exposes `update()` as the normal orchestration call and explicit
  stage methods for `process_strapdown_integration()`, `propagate_covariance()`,
  and `process_measurements()`. It owns a fixed-capacity FIFO of timestamped IMU
  increments fed through `push_imu(...)` and composes one pending discrete
  covariance step while draining that buffer. The pending covariance step is
  applied at the selected compile-time medium-rate cadence, and recent applied
  covariance steps are retained in a bounded history buffer for inspection and
  future delayed-measurement support. The selected propagation policy owns state
  propagation plus `F_k`/`G_k`/`Phi_k`/`Q_d` construction; the filter owns
  applying the covariance propagation; and the update policy owns
  measurement update behavior. `NoOpPropagation` remains available for
  measurement-only products, while the selected stationary GNSS products now use
  the first ECEF INS propagation policy.
- `EcefInsPropagation` implements the first single-IMU ECEF propagation path. It
  propagates nominal body-to-ECEF attitude as a unit quaternion, keeps the
  covariance attitude state as a 3D small-angle `Att` perturbation, builds
  first-order `F_k`/`G_k`/`Phi_k`/`Q_d` products, and applies the v1
  PVA+gyro-bias+accelerometer-bias dynamics to the selected
  `InsGyroAccelBiasStateDef` aggregate. Its nominal layout stores attitude in
  `AttQuat`, while its error/covariance layout keeps attitude as a 3D
  `AttRotVec` perturbation.
- Planet and gravity policies are the most complete examples of the intended
  concept -> optional CRTP base -> concrete policy layering.
- Product-core profiling now provides the embedded-facing vocabulary for future
  instrumentation: enum profile points, fixed timing records, optional
  visualization metadata fields, clock/sink/profiler concepts, a null default
  profiler, and a scoped profiler for deterministic clock/sink policies.
  `KalmanFilter::observation_update` and `Navigator::process_measurements` are
  the first coarse algorithm integration points and both default to
  `NullProfiler`. Sequencing/nesting semantics are not owned by the generic
  record type and remain future profiler/sink policy work.
- `sim::ImuSimulator` generates raw timestamped IMU increments from consecutive
  ECEF truth samples. It derives ideal body-frame inertial angular increments
  and specific-force increments, then applies deterministic gyro/accelerometer
  error-model parameters such as bias, bias random walk, white noise, scale
  factor, misalignment, non-orthogonality, and quantization. The selected
  SWIL application validates IMU runtime config, generates these increments, and
  feeds them into the Navigator before GNSS measurement processing.
- Simulation currently contains desktop-oriented support and may use runtime
  polymorphism where practical.
- Python analysis is deliberately outside the embedded-facing C++ product core. It consumes
  portable CSV/JSON run artifacts directly and can package them into versioned HDF5 bundles for
  cached large-campaign analysis; C++ remains responsible only for raw desktop logs.

## Policy concepts and template boundaries

Policy concepts are intended to name stable capability boundaries, not to copy
every member function of the first implementation that happens to satisfy them.
Use a concept when multiple consumers rely on the same interface, when a
configuration-selected type is intentionally swappable, or when diagnostics at a
public template boundary would otherwise be poor.

Context-dependent concepts stay candidate-first and carry only the context they
actually require. For example, a measurement model is validated relative to a
state definition at the filter or product-configuration boundary; the generic
sensor container does not need to know that state definition merely to store and
queue measurements.

When two consumers need different parts of a type's capability, prefer separate
narrow concepts over one over-coupled concept. A future filter cleanup may
distinguish the standalone filter lifecycle contract from the sensor-specific
"this filter can process this sensor" contract so Navigator construction,
initialization, and tuple compatibility checks can each name the dependency they
actually own.

Raw `typename` remains appropriate for private tuple expansion helpers, local
implementation details, and deliberately unconstrained utilities. It is a design
smell on primary policy or product-configuration surfaces when a clear concept
already exists and improves readability or diagnostics.

## Explicitly not implemented yet

- IMU history/replay, delayed-measurement handling, and latency compensation.
- General coordinate conversions and local-vertical altitude modeling.
- Barometer simulator behavior beyond current shells/placeholders.
- Embedded target profiles, resource budgets, and hardware abstraction layers.

Use `docs/ROADMAP.md` for sequencing and status, and update this document when
implemented architecture changes.
