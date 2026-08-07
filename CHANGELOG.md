# Changelog

All notable changes to NavKit will be documented in this file.

The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

This project follows
[Semantic Versioning](https://semver.org/).

---

## [Unreleased] - YYYY-MM-DD

### Added

- Runtime-configured chi-square innovation acceptance for separate GNSS
  position and velocity observations, with measurement-model-derived degrees
  of freedom and thresholds, rejection before persistent filter mutation, and
  versioned diagnostics that log NIS, gate configuration, numerical validity,
  and the acceptance decision.
- Pass 8.1 adds a versioned deterministic truth-reconstruction regression
  runner, compact provenance-rich JSON reports, strict ECEF/SLERP time
  alignment, failure-only artifact retention, and stationary free-inertial,
  stationary truth-GNSS, ballistic, and bank-to-turn acceptance scenarios with
  evidence-calibrated position, velocity, and attitude tolerances plus explicit
  free-inertial versus aided GNSS update-count contracts.
- Runtime JSON now uses degrees for direct angular values and degrees per
  second for direct angular rates while retaining radian-based covariance and
  PSD terms. Trajectory physics cadence is explicitly named
  `dynamics_rate_hz`/`dynamics_dt_s`. A persistent Guidance-output LPF now
  applies independently configured body-X/Y/Z specific-force and bank time
  constants, supports per-channel zero-time-constant bypass, and accepts
  continuous state-machine parameter changes without resetting filter state.
  Optional state-entry windows temporarily select slower time constants on the
  first sample after a Guidance transition, then return automatically to the
  active state's nominal filter constants.
- Pass 7.14 replaces profile-specific Guidance branches with one validated
  runtime JSON state graph composed from typed reference, acceleration, bank,
  transition, plant-constraint, and filter blocks. Guidance, Autopilot, and
  Vehicle/plant ownership now have distinct simulation domains, and trajectory
  diagnostics expose a generic `guidance_state_index` instead of a fixed mode
  enumeration. The Guidance-to-Autopilot command now contains only filtered
  body-specific force and bank, while focused execution, diagnostics, and
  Autopilot-state payloads prevent downstream consumers from depending on the
  trajectory-wide state and environment.
- Trajectory diagnostics now express every acceleration and specific-force
  series in standard gravity units, place independent legends beside their
  corresponding subplots, and enable mouse-wheel zoom plus double-click home
  reset across the shared interactive 2-D and 3-D renderers.
- Generated trajectory Guidance now exposes explicit altitude proportional and
  vertical-speed derivative gains. Coordinated bank is defined in the plane
  normal to the full three-dimensional velocity vector, remains valid at
  nonzero flight-path elevation, wraps on `[-180, 180]` degrees, and enforces
  the common `+/-60` degree command limit.
- The trajectory-generation reference now uses rich frame notation for DCMs
  and quaternions and documents the permanent Guidance command filter,
  altitude P--D contribution, and velocity-normal coordinated-bank
  construction.
- Pass 7.13 trajectory-profile follow-up: a 2--3 minute ballistic reference
  with an approximately five-g powered body-X specific-force boost; paired
  skid-to-turn and bank-to-turn horizontal
  S-turns; altitude-coupled coordinated-bank allocation; optional body-lateral
  specific-force suppression; planar vertical calibration; and a sustained
  acceleration-based Dutch-roll calibration combining horizontal excitation
  at its base frequency with twice-frequency vertical excitation, coordinated
  bank-to-turn, a front-view half-pipe trajectory, and body `p`/`r` excitation
  in quadrature.
- Frame-rigorous local-guidance conversion now includes NED transport, uses
  physical inertial acceleration for coordinated-bank force allocation, and
  resolves plant specific-force commands through the realized physical
  attitude rather than the selected control-state attitude.
- Consolidated trajectory analysis into frame-explicit kinematics, focused
  Guidance and Autopilot figures, one Guidance/Control dashboard, tracking
  errors, and LLA/relative-local 3-D products. Body plots identify
  `w_ib_b` as the body inertial angular rate \(\omega_{ib}^{b}\) explicitly;
  relative-local position uses North/East/Up with `Up = -Down` and a
  data-proportional aspect.
- Pass 7.12 trajectory/estimator correctness follow-up: uniform
  truth-minus-estimate analysis signs, same-epoch sequential GNSS
  injection/reset, measured-rate antenna lever-arm velocity modeling,
  independent deterministic GNSS position/velocity random substreams,
  rotating-Earth centrifugal dynamics, midpoint-attitude delta-velocity
  mechanization, ballistic gravity-turn/impact behavior, smoothed trajectory
  commands with desired-rate feedforward, and expanded nested-loop/body-frame
  diagnostics.
- Source-agnostic low-fidelity Guidance, Autopilot, and Vehicle-response
  boundaries with runtime-selected navigation-estimate or truth-passthrough
  control state, actual simulated-IMU rate feedback, exact subsystem cadences,
  and ECI truth-plant integration.
- Explicit launch-pad, boost, gravity-turn, free-inertial, curved-Earth
  constant-altitude, calibration, and waypoint Guidance modes with
  velocity-alignment guards, two-stage specific-force response, body-rate
  response/limits, and closed-loop navigation selected by default.
- Runtime trajectory Guidance and Autopilot/Vehicle logs plus reusable
  interactive Euler, ECEF/ECI/NED/body kinematic, command/response,
  tracking-error, LLA 3-D, and relative-NED 3-D analysis products.
- Frame-explicit trajectory attitude input for all quaternion, DCM, and
  aerospace 3-2-1 RPY forms across ECEF, ECI, and local NED directions,
  validated and canonicalized to passive scalar-first `q_b2e`.
- Reusable status-returning geodetic/ECEF, local-NED, and uniformly rotating
  fixed/inertial frame transforms with focused round-trip regression coverage.
- A standalone modular trajectory-generation algorithm reference covering the
  implemented Earth-orientation, attitude-input, response-dynamics, ECI
  integration, saturation, and diagnostics contracts.
- Optional frame-explicit trajectory-inspection products for ECEF, ECI, NED,
  body-resolved, and command/control/realized-response data, with independent
  runtime cadences and reusable Plotly/Matplotlib trajectory dashboards.
- Generated runtime trajectory profiles and scenarios for launch-pad ballistic
  boost/coast, curved-Earth constant-altitude flight, three calibration
  maneuvers, and bank-limited waypoint following. All generated non-stationary
  profiles derive body inertial angular rate from their truth attitude and
  reject unsupported direct input fields rather than silently ignoring them.
- Planned-time simulation orchestration with exact rational application
  cadence, streaming stationary/CSV trajectory sources, simulated and
  real-time clock contracts, and two-phase synthetic emulator preparation and
  publication so Navigator-visible measurements are not exposed before their
  planned deadline.
- Runtime-selectable simulated/realtime app-support clocks, explicit planned
  timeline production separate from consumer-side due schedules, and
  owner-specific planned-loop failure reporting.
- Shared `TruthTrajectory` source storage/querying for generated and CSV ECEF
  truth, exact-cadence IMU truth sampling with interpolation, CSV playback
  through the normal simulation app path, and full local-level transport-rate
  handling for degree-based runtime `w_nb_b_degps` trajectory initialization.
- Product-core versioned timestamp, duration, time-scale, and rational-rate
  vocabulary; exact multi-rate simulation/logging schedules; and focused
  600 Hz, duration-borrow, and incommensurate-rate regression coverage.
- Minimal time-vocabulary headers (`TimeTypes`, `Timestamp`, `Duration`,
  `RationalRate`, and `RationalSchedule`), with `Time.hpp` retained only as a
  convenience umbrella and public timestamp fields standardized as `t/s/ns`.
- Product-family variant directories for compile-time NavKit and simulation-app
  selections, plus ownership-based compile-time component directories.
- Reusable `EcefInsGnssLc<...>` product composition with explicit concrete
  `GyroAccelBias` default/profiled selections, plus state-contract-aware
  compile-time and runtime configuration naming.
- Runtime scenario taxonomy under `config/runtime/navkit_sim/scenario`, with
  role-keyed component compositions, resolved replayable run inputs, and
  Monte Carlo restart-initialization scenarios supporting deterministic
  explicit or covariance-colored full error-state estimate errors.
- Matched and deliberately conservative HG1700 Monte Carlo covariance scenarios
  plus initial gyro/accelerometer bias covariance-matching report metrics.
- Uncertainty-normalized Monte Carlo CDF-residual consistency dashboards with
  pointwise finite-campaign sampling uncertainty.
- A canonical current-state handoff and master roadmap.
- Repository-wide agent guidance and documentation indexes.
- Cross-platform environment bootstrap tooling and Linux/Windows GitHub Actions CI.
- Candidate-first `InjectionPolicy` and `ResetPolicy` concepts with positive and negative compile-time tests.
- Candidate-first `MeasurementModelPolicy` concept with positive coverage for GNSS position, GNSS velocity, and barometer models plus negative compile-time tests.
- Candidate-first `NoisePolicy` concept with positive and negative compile-time tests.
- First-pass `FilterPolicy`, `SensorCollectionPolicy`, `UpdatePolicy`, and `NavigatorUpdatePolicy` concepts for Navigator orchestration boundaries.
- Measurement-statistics regression tests for accepted and rejected measurement updates.
- Design-intent testing guide plus focused coverage for ring-buffer overflow policies, sensor FIFO/noise behavior, CSV writer output/failure behavior, and stationary trajectory semantics.
- Linux-oriented coverage reporting through `tools/quality/coverage.py` and a CI coverage artifact.
- Lightweight timing artifacts for stationary simulation and analysis runs, plus coarse Debug/Release executable/library size reports through `tools/profile/resource_report.py`.
- Human-readable timing summaries through `tools/profile/timing_report.py` and documented `navkit.timing.v1` artifact schema fields.
- Stable public tooling under `tools/`, profiling diagnostics under `tools/profile/`, and internal support/verification helpers under `tools/internal/`, including controlled HDF5-backed Monte Carlo analysis-scaling benchmarks.
- Product-core embedded profiling vocabulary with enum profile points, fixed timing records, visualization metadata fields, clock/sink/profiler concepts, `NullProfiler`, `ScopedProfiler`, and deterministic concept/runtime tests.
- Coarse embedded profiling integration points for `KalmanFilter::observation_update` and `Navigator::process_measurements`, both defaulting to `NullProfiler`.
- Runtime-input validation for the selected stationary GNSS app composition, including required scenario sections, unsupported sensor/emulator sections, and numeric/vector shape checks.
- Generic `SimulationApp<Config>` support with app-configured sensor bindings, unsigned sensor IDs, emulator tuples, and tuple-derived runtime validation.
- Public `include/navkit/api/config` contracts for user-facing product config graphs, including `NavKitProductConfigPolicy`.
- App-side navigation initialization and transfer-alignment provider seams, including typed PVA/TXA startup and alignment vocabulary, deterministic and random PVA initialization providers, transfer-alignment sample validity flags, and validation for the selected stationary GNSS app config.
- Focused ECEF navigator v1 algorithm specification under `docs/algorithms/navigator_ecef_v1`, covering the first one-IMU quaternion mechanization, coning/sculling baseline, analytical covariance prediction contract, GNSS position/velocity antenna-lever-arm observations, runtime API contract, and validation gate.
- Focused IMU emulator v1 algorithm specification and first working simulator implementation, including a product-core IMU increment sample, ECEF-truth-derived ideal increments, deterministic gyro/accelerometer error-model parameters, seeded noise, bias random walk, quantization, runtime config parsing helpers, and equation-shaped tests.
- First ECEF INS propagation policy with rotation-vector/quaternion helpers, two-sample coning/sculling compensation, ECEF PVA nominal propagation, first-order `F_k`/`G_k` covariance prediction, symmetry-preserving covariance tests, and stationary truth/IMU closed-loop tests.
- Navigator typed IMU ingestion through `push_imu(...)`, fixed-capacity IMU buffering, propagation success reporting, and explicit stage-method coverage.
- Simulation app IMU runtime validation and app-configured IMU simulator selection for feeding generated IMU increments into the selected Navigator.
- Shared runtime cadence parsing for `rate_hz` or `dt_s`, plus a small IMU runtime helper that keeps IMU simulator initialization/generation details out of the central simulation loop.
- Compile-time medium-rate covariance cadence and bounded covariance-step history configuration for Navigator propagation products.
- Runtime-configurable console, truth, nav-estimate, and measurement-statistics logging cadences so high-rate simulation no longer forces high-rate file or console output.
- A `tools/run_scenario.py` workflow that runs a runtime scenario, can override output location/run name without mutating checked-in JSON, writes an effective runtime config, and then runs the standard analysis plots.
- Compile-time and runtime-selected first-order Gauss-Markov IMU error dynamics for ECEF INS covariance prediction, with matching IMU simulator bias-dynamics support and tests.
- Compile-time and runtime-configurable covariance floors for selected filter error-state layouts, including diagonal and frame-aware INS PVA floor forms with validation and Navigator/filter application tests.
- Split ECEF INS propagation runtime configuration into separately owned process-noise and IMU bias-dynamics payloads, with compile-time defaults, runtime JSON overrides, and focused validation.
- Runtime filter nominal-state restart overrides for selected non-PVA nominal state values under `filter_initialization.nominal_state.non_pva_values`, with validation and initialization tests.
- A seeded Monte Carlo campaign runner with generic JSON-pointer seed derivation, replayable per-run effective configs, campaign/run manifests, and first-pass aggregate error/covariance plots.
- Monte Carlo-focused runtime scenario logging that keeps low-rate truth, navigation estimate, and IMU truth-bias logs while disabling high-volume debug/correction/statistics outputs.
- Monte Carlo aggregate report generation with per-axis RMSE/coverage, state-family NEES, GNSS NIS, run timing, output-size summaries, and a comparison utility for existing campaign reports.
- Plotting controls for selected single-run/Monte Carlo plot groups and post-run time windows, plus gyro-bias plot/report scaling in `deg/hr`.
- Versioned schema compatibility helpers for Monte Carlo campaign/run/report artifacts and future analysis inputs.
- Optional HDF5 analysis-bundle packaging for raw CSV runs and Monte Carlo campaigns, including source metadata, per-run logs, cached truth-error products, and cached aggregate Monte Carlo series.
- Shared CSV/HDF5 analysis-source loading plus renderer-neutral plot specifications with Matplotlib static and Plotly interactive renderers.
- `tools/package_analysis.py` for HDF5 packaging, `tools/plot_field.py` for quick named-field inspection, and HDF5/Plotly support in `tools/plot_monte_carlo.py`.
- Interactive Monte Carlo controls that keep individual-run histories visible, toggle the unified hover panel on demand, and retain one shared Unicode legend per figure.
- Plotly interactive aggregate figures and HDF5 analysis-bundle packaging as the default Monte Carlo campaign analysis outputs, with Matplotlib retained as an explicit static-renderer choice.
- Cached joint Monte Carlo NEES/NIS histories in HDF5, with full-INS/PVA/navigation/IMU-bias consistency products, observation-family NIS, interactive time-linked PDF/CDF/QQ dashboards, and machine-readable consistency reports.
- Per-state X/Y/Z marginal normalized-squared-error consistency dashboards, with heatmap-hover epoch selection, explicit click/type pinning, a control to resume hover-following, and browser-side snapshot export.
- Empirical-CDF heatmaps in raw NEES/NIS space and dimension-independent PIT-space CDF-residual heatmaps, organized with the existing occurrence-density dashboards under a generated consistency-figure index.
- Analysis-performance evidence and cache-safe desktop scaling: explicit full/derived-only HDF5 bundle modes, configurable chunked compression, package/figure/dashboard fingerprints, campaign performance reports, selected artifact controls, and opt-in parallel Plotly rendering with serial/parallel equivalence verification.
- Demand-driven HDF5 consistency-cache preparation through `tools/prepare_analysis.py`, named cache-stage timing, shared Monte Carlo NED transforms, and empirical-CDF reference strips in interactive consistency dashboards.

### Changed

- Moved applied-correction cycle ownership from `KalmanFilter` to `Navigator`:
  filter injection now returns a filter-domain correction value, Navigator
  composes sequential sensor corrections in injection order, and correction
  logging consumes the completed Navigator-owned result without filter-side
  logging bookkeeping.
- GNSS position and velocity consistency figures now show the runtime gate in
  both equivalent forms: the configured chi-square NIS acceptance threshold
  and its upper-tail p-value rejection threshold.
- Preserved configured physical mission references independently from the
  runtime-selected control-state feedback source, so navigation-estimate
  initialization errors no longer redefine commanded altitude, heading, or
  launch-rail attitude.
- Moved non-embedded simulation logging composition out of compile-time product
  configs into runtime logger selection, split trajectory diagnostics into
  Guidance and Autopilot/Vehicle products, and changed GNSS availability
  configuration from outage windows to explicit active windows.
- Aligned the default compile-time IMU-bias initial covariance with the
  moderate conservative runtime default instead of the previous oversized
  gyro-bias covariance.
- Consistency-dashboard colorbars now derive their placement from the Plotly
  subplot domains and use a narrower vertical title to avoid overlapping the
  heatmaps or selected-epoch distributions.

- Renamed the single-run plotting tools to `plot_run.py` and `plot_field.py`, renamed the Monte Carlo per-run plotting option to `output.plot_individual_runs`, and advanced the campaign schema to `navkit.monte_carlo_campaign.v2`.
- Reconciled README and setup documentation with the current implementation and build configuration.
- Formatted console status timestamps as `HH:MM:SS.sss` while preserving raw `time_s` in CSV logs.
- Updated the configured C++ language standard from C++20 to C++23.
- Registered the StateDef policy tests in the configured test executable.
- Ordered source mutation/checks before build and test verification.
- Refined estimator policy concepts by splitting standalone filter, filter-correction, and sensor-filter compatibility contracts; renamed measurement-model vocabulary to `MeasurementModelPolicy` and `Sensor::MeasurementModel_t`.
- Consolidated superseded TODO lists and early core design notes into the canonical roadmap before removing them.
- Constrained `KalmanFilter` on `StateDefPolicy`, injection policy, and reset policy boundaries.
- Constrained `KalmanFilter` observation-update and measurement-statistics methods on measurement-model policy compatibility.
- Constrained `Sensor<Id, Model, BufferSize, NoisePolicy>` on noise-policy compatibility while preserving fixed-capacity buffering.
- Completed the Phase 2 estimator-boundary refactor scope and explicitly deferred `SensorPolicy` until a Navigator-facing capability boundary exists.
- Constrained `Navigator` on current filter, sensor-collection, and update-policy capabilities.
- Clarified ADR-003 and agent guidance around valid C++ concept-definition syntax versus constrained template-parameter syntax.
- Reorganized public headers from the generic flat `core` bucket into structured product-core domain folders.
- Reorganized public headers under `include/navkit/core` as the reusable product-core boundary, with estimation domains under `core/estimation`, environment under `core/environment`, and simulation/IO kept outside core.
- Split the monolithic CMake library into `navkit_core`/`navkit::core` for reusable product-core code, `navkit_sim`/`navkit::sim` for simulator support, and `navkit_io`/`navkit::io` for desktop logging/file/JSON support, while keeping runnable executables under `apps/`.
- Split root CMake orchestration from product-boundary target definitions, moved header-only/interface target definitions under `cmake/targets`, kept compiled simulator target metadata beside simulator sources, and removed the dummy source file by modeling header-only core code as an `INTERFACE` target.
- Updated the documented development workflow to include changelog updates and README/SETUP reconciliation for user-facing behavior, layout, tooling, or workflow changes.
- Added a current architecture document and moved detailed target-boundary, namespace, source-layout, and target-kind rationale out of setup-oriented documentation.
- Added a dedicated configuration guide covering domain config concepts, concrete config slices, example config contracts, static-assert wiring, runtime-input separation, and the `NAVKIT_CONFIG` selection model.
- Aligned public namespaces with the product-core folder structure through the stable domain level: `navkit::core::estimation`, `navkit::core::environment`, `navkit::core::frames`, `navkit::core::models`, `navkit::core::units`, and `navkit::core::containers`.
- Elevated compile-time configuration cleanup, Release/Debug compiler-flag hardening, static-analysis posture, runtime profiling/resource evidence, and intentional coverage strategy into the next immediate roadmap phase.
- Clarified the roadmap distinction between product-core compile-time configuration and runtime app input bundles such as `config/runtime/navkit_sim/...` scenario files.
- Replaced vague `core/common` configuration with explicit `core/config` headers for foundational types, narrow configuration capability concepts, and default configuration slices.
- Moved estimator-specific configuration concepts for sensor buffer capacity and measurement-statistics availability beside the estimation domain while keeping `core/config` focused on shared scalar/time configuration vocabulary.
- Moved concrete app/product compile-time configuration examples out of public NavKit headers and into `config/compiletime`.
- Added `NAVKIT_CONFIG` CMake selection with a generated `navkit/SelectedConfig.hpp` alias and `tools/build.py --navkit-config` forwarding.
- Added `tools/build.py --build-dir`, selected-config CMake presets, and stricter `NAVKIT_CONFIG` validation for multi-config development.
- Added centralized NavKit-owned target warning profiles, CI warnings-as-errors, embedded-oriented Release optimization settings, Release CI build verification, and `tools/build.py` compile-check switches.
- Added Linux Debug `clang-tidy` static analysis to CI and made the local tidy wrapper require a valid compilation database instead of silently running without build flags.
- Clarified that clang-tidy is intentionally a CI gate and not part of the normal local agentic development loop.
- Preserved stationary GNSS timing and resource reports as CI artifacts without making wall-clock timing a brittle pass/fail gate.
- Default-enabled timing artifact updates for build and test wrappers, with opt-out flags for quiet or artifact-free commands.
- Made build, test, simulation, and analysis wrappers print concise timing summaries by default after updating `timing.json`.
- Made build and resource-report wrappers display coarse executable/library size summaries by default after writing resource artifacts.
- Moved `navkit_sim` runtime JSON inputs from `apps/navkit_sim/configs` to `config/runtime/navkit_sim`.
- Removed stale root example placeholder directories and documented that future architecture domains should not be represented by empty folders.
- Split compile-time configs into reusable NavKit library configs under `config/compiletime/navkit` and app composition configs under `config/compiletime/apps`, with a generic selected-app launcher for `navkit_sim`.
- Moved reusable NavKit product configs under `config/compiletime/navkit/products` with product-local namespaces and role-based internal type names.
- Expanded `ConfigApi.hpp` into the shared product-config include for common core graph machinery.
- Clarified that same-named NavKit and app compile-time config files are expected when separated by ownership directories, and documented how runtime JSON links to the selected app/NavKit composition.
- Replaced the bespoke stationary GNSS app runner with the generic simulation app loop while preserving stationary GNSS log/profile behavior.
- Moved runnable NavKit product graph aliases into reusable NavKit configs and collapsed app configs to `NavKit` plus explicit `EmulatorBindings`.
- Renamed the profiled reusable NavKit GNSS config to `EcefInsGnssLcGyroAccelBiasProfiled.hpp` to match the app-level selected config name.
- Replaced app-facing sensor-index wiring with configured `Sensor::Id` values, emulator-owned stream IDs, explicit `(Emulator, Sensor)` bindings, and tuple helpers for ID-based lookup.
- Replaced derived `MeasurementModels` config aliases with explicit `MeasurementStatisticsTuple` aliases keyed by configured sensor types.
- Split app-support policy concepts into standalone headers and constrained simulation app configs, emulator bindings, emulator runtime plumbing, runtime validation, measurement models, and Navigator sensor processing on their real concept boundaries.
- Reorganized `include/navkit/app_support` into ownership-oriented subdirectories for app config, emulation, runtime input, initialization, logging, profiling, and trajectory support.
- Split concrete IO log products and reusable log payload wrappers into focused `log_products` and `log_payloads` headers, and removed the unused `RunLogProducts.hpp` umbrella.
- Changed the default build directory convention so Python wrappers and presets derive build trees from the selected compile-time config header, preventing different `NAVKIT_CONFIG` builds from sharing one generated selected-config tree.
- Added a `KalmanFilter::MeasurementStatisticsTuple_t` class-level alias for consistency with the other filter type aliases.
- Split profiling, sensor-tuple, emulator-binding, product-config, and runtime-config-validation headers so public contracts stay separate from helper/trait machinery.
- Removed unused profiling and sensor-tuple umbrella headers after replacing internal users with narrower includes.
- Documented the config API include boundary in agent and architecture guidance, keeping `ConfigApi.hpp` focused on shared product graph vocabulary and exposed defaults.
- Tightened `SensorCollectionPolicy` around real NavKit sensors and moved ID/tuple lookup helpers out of public config headers.
- Moved Navigator policy compatibility checks to a dedicated header and simplified KalmanFilter measurement-statistics storage naming.
- Replaced Navigator's update-policy template-template parameter with an explicit concrete `NavigatorUpdate` policy alias in reusable NavKit configs.
- Moved app-side emulator binding vocabulary to `EmulatorBinding.hpp`, added focused trajectory-provider and measurement-statistics logging helpers, and slimmed `SimulationApp` orchestration.
- Extracted run settings, filter initialization, and emulator runtime processing out of `SimulationApp`, replaced dummy-object statistics dispatch with type-level logging, and collapsed one-field GNSS buffer config wrappers in reusable product configs.
- Renamed compile-time config constants such as GNSS sensor IDs and buffer sizes to snake_case while keeping type aliases in PascalCase.
- Refactored stationary simulation logging so `RunLogger` coordinates composable log-product adapters while app compile-time configs explicitly select the logger type.
- Added payload-specific log-product concepts and CSV schema helpers so logging adapters expose explicit serialization boundaries.
- Reworked `RunLogger` into a compile-time log-product tuple facade with typed payload dispatch, selected-product metadata emission, and generic app-support measurement-statistics logging.
- Moved logger composition into app compile-time configs, added logger lifecycle/payload/product-access concepts, and made `RunLogger<...>` the generic log-product tuple facade.
- Removed public `MeasurementStatisticsTuple` aliases from reusable product configs; `KalmanFilter` now derives filter-owned diagnostics storage from the configured `Sensors` tuple and exposes `measurement_statistics_available<Sensor>()`.
- Added sensor diagnostics configuration and disabled-statistics coverage while keeping measurement statistics keyed by configured sensor type.
- Installed Ninja through bootstrap as a local compile-database convenience for optional Windows clang-tidy diagnostics.
- Exposed sensor diagnostics aliases in reusable product configs and moved profiling clock metadata onto the selected clock type to avoid config metadata drift.
- Made Ninja the default generator for Python build/test/sim/tidy wrappers and rooted default build directories by generator, build type, and selected compile-time config.
- Parameterized the GNSS position-update log product on its selected measurement-statistics stream so matrix dimensions and metadata come from the configured payload instead of hard-coded schema constants.
- Split the Navigator propagation seam into explicit strapdown-integration and covariance-prediction hooks, added `Navigator::update()` as the normal orchestration call, and moved the simulation app loop to that API while preserving current GNSS-only `NoOpPropagation` behavior.
- Corrected the truth-log metadata convention for `q_eb` so it matches the ECEF-to-body attitude used by trajectory truth and IMU derivation.
- Tightened the IMU emulator API and truth schema: `TruthSample` now carries only trajectory truth, truth logs no longer include IMU-derived acceleration/angular-rate fields, reusable quaternion/skew helpers live under core math, and IMU generation uses explicit `bool`/out-parameter failure handling instead of exceptions or `std::optional`.
- Selected stationary GNSS product configs now use the first ECEF INS propagation policy instead of `NoOpPropagation`, while `NoOpPropagation` remains available for measurement-only products.
- Split the selected ECEF INS state-space definition into explicit nominal and error layouts: `InsGyroAccelBiasStateDef` now aggregates `InsGyroAccelBiasNominalStateDef` with `AttQuat` and `InsGyroAccelBiasErrorStateDef` with `AttRotVec`, so propagation, covariance, Jacobians, and injection use the correct vector dimensions at their boundaries.
- Extended MSVC NavKit-owned compile options with `/bigobj` so template-heavy selected-config application translation units compile reliably on Windows.
- Corrected the propagation/filter ownership boundary: propagation policies now operate on configured state definitions, build discrete covariance inputs, and leave `KalmanFilter` responsible for applying covariance propagation.
- Reduced the v1 INS state definitions to bias-only PVA/IMU-error states by removing stale gyro and accelerometer scale-factor segments.
- Moved reusable coning/sculling, planet-rate, gravity-gradient, quaternion/RPY, and simulation random-draw helpers toward their owning domains instead of keeping them inside the first ECEF INS and IMU simulator implementations.
- Moved filter initial covariance into a selected immutable NavKit product config value using `InitialCovariance<StateDef>` and `diagonal_initial_covariance<StateDef>()`, tuned the current examples to variance values corresponding to 50 milli-deg/s gyro bias and 100 micro-g accelerometer bias, swept app-support runtime parsing to remove hidden scenario fallback defaults, and added runtime JSON override validation for configured initial covariance, including raw full-state forms and a frame-aware PVA plus remaining-error-state diagonal form.
- Renamed the repo-level simulation wrapper from `tools/run_first_sim.py` to `tools/run_sim.py`, clarified that `--navkit-config` locates a compile-time-configured build tree rather than changing runtime behavior, and factored runtime JSON merge/output override handling into reusable tool helpers.
- Moved IMU cumulative increment ownership from the nominal CSV log product into the full-rate IMU runtime path so decimated IMU log rows contain run cumulative snapshots rather than sums of only logged rows.
- Removed stale app-support/core unused-parameter breadcrumbs around filter initialization and runtime JSON validation while preserving intentional no-op parameters at required concept/API seams.
- Tightened runtime `filter_initialization` validation so unknown keys are rejected instead of silently expanding the startup contract.
- Exposed small reusable Python analysis helpers so Monte Carlo covariance plots share single-run scaling conventions while using a narrow aggregate-analysis loader.
- Optimized Monte Carlo aggregate plotting to use a narrow run loader, optional plot decimation, aggregate plot timing, and CLI overrides for run count, start index, parallel jobs, and maximum plot points.

### Removed

- Removed the public `SensorGraphConfigPolicy` helper and the `MeasurementModelsFromSensors_t` derivation path.

- Placeholder `imu_gnss_straight_line.json` runtime config until the corresponding simulation path is real and validated.
- Removed the obsolete `python/navkit_analysis/run_sim.py` package helper; simulation launching now belongs to repo-level tools.

### Fixed

- Corrected ballistic launch behavior so pad support force is not retained as
  actuator state and the configured speed guard preserves the launch program
  through the low-speed gravity-turn singularity before powered
  velocity-alignment begins.
- Corrected waypoint bank-to-turn behavior so turns are produced by bounded
  lateral acceleration while speed follows the current course, and final
  waypoint acceptance transitions to a stable terminal continuation rather
  than repeated point pursuit and reversals.
- Removed stale feature-status claims and resolved the C++20/C++23 documentation mismatch.

---

## [0.1.0] - 2026-XX-XX

Initial pre-employment release establishing NavKit as an independently
developed software platform.

### Added

#### Repository

- Initial repository structure
- CMake build system
- Conan package management
- Cross-platform Python tooling
- VS Code development environment
- Clang-format configuration
- Clang-tidy configuration

#### Core

- Generic error-state Kalman filter
- Compile-time StateDef architecture
- Segment abstraction
- Ring buffer
- Generic measurement framework
- Generic sensor abstraction
- Policy-based filter architecture

#### Navigation

- Earth model
- Gravity model
- Coordinate frame utilities
- Unit framework (initial)

#### Simulation

- Trajectory generator
- GNSS simulator
- Truth generation

#### Analysis

- CSV logging framework
- Run logger
- Measurement statistics
- Covariance plots
- Innovation plots
- NIS analysis
- p-value analysis
- Innovation histograms

#### Testing

- Initial unit test framework
- Ring buffer tests
- Segment tests
- Navigation compile tests
- Measurement model tests

#### Documentation

- README
- SETUP
- Naming conventions
- Repository organization

### Changed

- Numerous architectural refinements during initial development.

### Fixed

- Build system compatibility
- Conan integration
- Template metaprogramming issues
- Logging architecture
- Plotting infrastructure

