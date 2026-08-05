# NavKit

> A modular estimation and navigation development platform for embedded aerospace systems.

![C++](https://img.shields.io/badge/C%2B%2B-23-blue)
![CMake](https://img.shields.io/badge/CMake-3.23%2B-brightgreen)
![Conan](https://img.shields.io/badge/Conan-2.x-orange)
![Python](https://img.shields.io/badge/Python-3.10%2B-yellow)
![Status](https://img.shields.io/badge/status-active%20development-blue)

NavKit is a C++ estimation and navigation framework aimed at embedded aerospace systems. It combines fixed-size estimation infrastructure with simulation, logging, and offline analysis so navigation algorithms can be developed and validated in one repository.

The current working demonstration is a stationary ECEF INS/GNSS-position estimator: ideal or configured IMU increments drive the first strapdown propagation policy, and GNSS position measurements provide aiding. The longer-term objective is a reusable, embedded-ready INS/GNSS platform with additional mechanizations, aiding sources, environments, and validation workflows.

## Current status

| Capability | Status |
|---|---|
| Fixed-size error-state Kalman filter | Implemented |
| Compile-time state definitions and segments | Implemented |
| Measurement models, sensor queues, and update statistics | Implemented |
| Planet, gravity, and frame policies | Implemented |
| Stationary ECEF INS/GNSS-position simulation and logging | Implemented |
| Covariance, innovation, NIS, p-value, and histogram analysis | Implemented |
| Fully constrained estimator policy architecture | In progress |
| GNSS velocity and barometer integration | Planned; model classes exist |
| Propagation policy and first strapdown INS mechanization | Implemented first pass |
| IMU simulation | Implemented first pass |
| Barometer simulation | Planned; shell exists |
| Monte Carlo and automatic validation reports | Planned |

NavKit requires C++23.

## Architecture direction

NavKit favors compile-time capability checks, policy-based composition, fixed-size Eigen types, and runtime algorithms that focus on orchestration. The proposed architectural pattern is:

```text
concept -> optional CRTP base -> concrete policy -> runtime algorithm
```

Today, the environment policies most fully realize this pattern. The estimator is partway through the same refactor, and Navigator has its first propagation/mechanization policy wired into the stationary app.

Current data flow:

```text
stationary truth -> IMU simulator -> Navigator IMU buffer
    -> ECEF INS propagation -> GNSS simulator -> Sensor queue
    -> KalmanFilter measurement update -> measurement statistics
    -> CSV/JSON logs -> Python analysis
```

Target data flow adds richer aiding, multi-rate sensor simulation, replay/latency handling, and broader validation; it should not be mistaken for current functionality.

## Repository layout

```text
include/navkit/core/  Reusable product-core public headers
include/navkit/api/   User-facing compile-time configuration API contracts
include/navkit/sim/   Simulation support public headers, grouped into sensors,
                       trajectory/guidance/control, and shared simulation math
include/navkit/io/    Desktop logging/file/JSON public headers, including log
                       products and payload boundaries
include/navkit/app_support/
                       Header-only executable support helpers organized by app
                       config, emulation, runtime input, logging, profiling,
                       trajectory, and initialization boundaries
config/               Compile-time configurations and runtime input bundles
cmake/targets/        Header-only/interface CMake target definitions
src/sim/              Compiled simulator implementation with matching domain
                       subdirectories
apps/                 Simulation and replay applications
tests/                Doctest unit and compile-time tests
python/               Offline analysis package
tools/                Cross-platform developer commands
docs/                 Setup, architecture, ADRs, roadmap, and reference material
output/               Generated logs and datasets
```

The reusable product core is currently header-only/template-heavy and is modeled
as the `navkit::core` CMake `INTERFACE` target. Simulator implementation is
compiled into `navkit::sim`, while desktop logging/file/JSON support is exposed
through `navkit::io`.

## Namespaces and targets

| CMake target | Namespace | Role |
|---|---|---|
| `navkit::core` | `navkit::core` | Reusable product-core common API and foundational types |
| `navkit::core` | `navkit::core::config` | Shared product-core compile-time configuration vocabulary |
| `navkit::core` | `navkit::core::containers` | Product-core containers |
| `navkit::core` | `navkit::core::estimation` | State definitions, measurements, filters, sensors, navigators, and estimator policies |
| `navkit::core` | `navkit::core::environment` | Planet and gravity policies |
| `navkit::core` | `navkit::core::frames` | Frame tags and frame-typed helpers |
| `navkit::core` | `navkit::core::models` | Reusable product-core measurement and process models |
| `navkit::core` | `navkit::core::units` | Unit and frame helper types |
| `navkit::core` | `navkit::api::config` | Public compile-time config contracts for product graph authors |
| `navkit::sim` | `navkit::sim` | Simulation support |
| `navkit::io` | `navkit::io` | Desktop logging, file, CSV, and JSON support |
| `navkit::app_support` | `navkit::app_support` | Selected-config app runner, JSON input, config description, and profile export helpers |

Public namespaces mirror the folder structure through the stable domain level.
Deeper leaf folders may organize implementation and policy families without
adding additional namespaces unless that subdomain becomes independently
meaningful.

Configuration concepts follow the domain that consumes them. Shared scalar/time
configuration vocabulary lives in `navkit::core::config`; estimator-specific
configuration concepts, such as sensor buffer capacity and measurement
statistics availability, live with the estimation domain.

Concrete compile-time configurations live under `config/compiletime` and are
selected per build tree with `NAVKIT_CONFIG`. Reusable NavKit library configs
live under `config/compiletime/navkit`; executable composition configs live
under `config/compiletime/apps`. Runtime scenario inputs live under
`config/runtime`. App-support code validates runtime inputs against the selected
compile-time app composition before running. Reusable NavKit configs expose the
product graph (`StateDef`, `Sensors`, `Profiler`, `Filter`, and `Navigator`);
app configs connect emulators to selected NavKit sensors with stable unsigned
sensor IDs plus explicit sensor aliases, so duplicate sensors of the same model
type remain representable without raw tuple indices in app configs.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for target-boundary,
namespace, and source-layout rationale.

## Getting started

Follow [docs/SETUP.md](docs/SETUP.md) for prerequisites and initial setup. The repository's Python wrappers are the primary developer interface.

Fresh local or cloud environment:

```text
python tools/bootstrap.py
```

First or clean Debug build:

```text
python tools/build.py --build-type Debug --clean
```

Normal edit/build/test cycle:

```text
python tools/quality/copyright.py --write
python tools/quality/format.py
python tools/quality/copyright.py --check
python tools/quality/format.py --check
python tools/build.py --build-type Debug --build-only
python tools/run_tests.py --build-type Debug
```

For changes worth tracking, update `CHANGELOG.md`. If behavior, layout,
tooling, or workflow changes, also reconcile `README.md` and `docs/SETUP.md`
before the final checks.

Run the working demonstration and analysis in one command:

```text
python tools/run_scenario.py --build-type Debug --show
```

Run a specific runtime input and put all logs/figures under a chosen output
folder:

```text
python tools/run_scenario.py --build-type Release --config config/runtime/navkit_sim/scenario/ecef_ins_gnss_lc_gyro_accel_bias_stationary_nominal.json --output-dir output/logs/my_case
```

## Documentation

- [Documentation index](docs/README.md)
- [Current architecture](docs/ARCHITECTURE.md)
- [Configuration model](docs/CONFIGURATION.md)
- [Testing strategy](docs/TESTING.md)
- [Development setup and workflow](docs/SETUP.md)
- [Master roadmap and current-state handoff](docs/ROADMAP.md)
- [Naming conventions](docs/NAMING_CONVENTIONS.md)
- [Founding principles](docs/FOUNDING.md)
- [Architecture decision records](docs/adr/)
- [Navigation reference](docs/navigation_reference/README.md)

## License

NavKit is proprietary software. See [LICENSE](LICENSE) and [COPYRIGHT.md](COPYRIGHT.md).
