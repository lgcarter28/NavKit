# NavKit

NavKit is an early-stage C++ navigation and sensor-fusion framework prototype. This first draft focuses on a minimal vertical slice:

```text
stationary truth trajectory
    -> GNSS position measurement simulator
    -> buffered sensor abstraction
    -> compile-time configured Navigator + KalmanFilter
    -> CSV time-history logs + JSON manifests
    -> Python plotting helpers
```

This repository intentionally does **not** include fixed-lag smoothing, embedded HALs, or hardware targets yet.

## Prerequisites

Required:

- C++20 compiler
  - Windows: MSVC 2022 or recent Clang/GCC through MSYS2/MinGW
  - Linux/macOS: GCC 11+, Clang 14+
- CMake 3.23+
- Python 3.10+
- Conan 2.x

Recommended:

- VS Code with the suggested extensions in `.vscode/extensions.json`
- `clang-tidy` and `clang-format`

## Dependencies

C++ dependencies are managed by Conan:

- Eigen 3.4
- nlohmann_json 3.11
- doctest 2.4

Python-side analysis dependencies are listed in `python/pyproject.toml`:

- numpy
- pandas
- matplotlib

## Configure and build

From the repository root:

```bash
python tools/build.py
```

This runs:

```bash
conan install . --output-folder=build --build=missing -s build_type=Release
cmake --preset conan-release
cmake --build --preset conan-release
```

For Debug:

```bash
python tools/build.py --build-type Debug
```

## Run tests

```bash
python tools/run_tests.py
```

## Run the first simulation

```bash
python tools/run_first_sim.py
```

Or run directly after building:

```bash
./build/Release/apps/navkit_sim/navkit_sim apps/navkit_sim/configs/stationary_gnss.json
```

On Windows, the executable path may be:

```text
build/Release/apps/navkit_sim/Release/navkit_sim.exe
```

## Output logs

The first simulation writes CSV time-history logs and JSON metadata under `data/logs/<run_name>/`:

```text
truth.csv
truth.meta.json
gnss.csv
gnss.meta.json
nav.csv
nav.meta.json
run_manifest.json
```

CSV is used for time histories. JSON is used for configuration, per-log metadata, and the hierarchical run manifest.

## Plotting

After running the first simulation:

```bash
python -m navkit_analysis.plots data/logs/stationary_gnss_demo
```

or:

```bash
python python/navkit_analysis/plots.py data/logs/stationary_gnss_demo
```

## Naming convention

Kinematic variables use Groves-style notation. See `NamingConventions.md`.

## Current limitations

- The current filter slice only demonstrates a GNSS position measurement update.
- The nominal INS mechanization and IMU process model are placeholders.
- Attitude injection is additive in this first draft and should be replaced by a proper quaternion-based nominal-state correction when the mechanization is added.
- No RTS/fixed-lag smoothing yet.
- No embedded HALs yet.
