# NavKit Development Environment Setup

This guide walks through setting up a complete development environment for NavKit.

NavKit is developed using modern C++, CMake, Conan, Python, and VS Code. The build system is intended to remain cross-platform across Windows, Linux, and eventually embedded toolchains.

---

## Prerequisites

NavKit currently expects:

- C++23-capable compiler
- CMake 3.23 or newer
- Conan 2.x
- Python 3.10 or newer
- VS Code
- MSVC on Windows, or GCC/Clang on Linux/macOS
- LLVM tooling for formatting

Recommended editor:

- VS Code with the repository-provided recommended extensions in `.vscode/extensions.json`

---

## Install Visual Studio 2026 on Windows

Install **Visual Studio 2026 Community**.

During installation, select the following workload:

- **Desktop Development with C++**

Under optional components, ensure the following are installed:

- Latest MSVC C++ Toolset
- Latest Windows SDK
- C++ CMake Tools for Windows
- Ninja Build
- Git for Windows, optional if already installed separately

Although development will primarily occur in **VS Code**, Visual Studio provides the Microsoft C++ compiler, linker, debugger, Windows SDK, and build tools required by CMake and Conan.

After installation, verify the compiler is available from a Developer PowerShell or Developer Command Prompt:

```powershell
cl
```

Verify CMake is available:

```powershell
cmake --version
```

NavKit requires **C++23**, and the active CMake configuration sets `CMAKE_CXX_STANDARD` to 23.

---

## Install VS Code

Install Visual Studio Code.

Recommended extensions:

- C/C++
- CMake Tools
- CMake
- Conan
- clangd
- Python
- GitLens
- Even Better TOML
- CodeLLDB, optional

The repository also provides recommended extensions through:

```text
.vscode/extensions.json
```

---

## Install Python

Install Python 3.10 or newer. Python 3.12+ is recommended.

Verify installation:

```bash
python --version
```

On Linux/macOS, this may be:

```bash
python3 --version
```

---

## Create a Python Virtual Environment

A dedicated virtual environment is recommended to isolate Conan and Python analysis dependencies. The preferred setup is the idempotent repository bootstrap command:

```bash
python tools/bootstrap.py
```

It creates `.venv`, upgrades pip, installs Conan and the local analysis package with its declared dependencies, and detects a default Conan profile when one does not exist. This is also the setup command for Codex cloud environments and CI.

The equivalent manual steps follow for troubleshooting.

From the repository root:

```bash
python -m venv .venv
```

Activate it.

Windows:

```powershell
.venv\Scripts\activate
```

Linux/macOS:

```bash
source .venv/bin/activate
```

Upgrade pip:

```bash
python -m pip install --upgrade pip
```

---

## Install Conan

Inside the virtual environment:

```bash
pip install conan
```

Verify:

```bash
conan --version
```

---

## Detect Your Compiler

Run once:

```bash
conan profile detect
```

This generates your default Conan profile.

Future versions of NavKit may provide repository-specific profiles such as:

```text
profiles/
    windows-msvc-debug
    windows-msvc-release
    linux-gcc-debug
    linux-gcc-release
    stm32f4-gcc
    stm32h7-gcc
```

---

## Install CMake

Install CMake 3.23 or newer.

Verify:

```bash
cmake --version
```

On Windows, CMake may already be installed with Visual Studio if the CMake tools component was selected.

---

## Install LLVM for Formatting

NavKit uses repository-wide formatting configuration stored in:

```text
.clang-format
.editorconfig
```

These files live at the repository root and are automatically used by VS Code and `clang-format`.

### Windows

Install LLVM:

```powershell
winget install LLVM.LLVM
```

Restart VS Code or your terminal, then verify:

```powershell
clang-format --version
```

If the commands are not found, add the LLVM `bin` directory to your `PATH`, typically:

```text
C:\Program Files\LLVM\bin
```

### Linux

Install Clang tooling:

```bash
sudo apt update
sudo apt install clang-format
```

Verify:

```bash
clang-format --version
```

---

## Install Python Analysis Dependencies

The bootstrap script installs the analysis package and its declared dependencies from `python/pyproject.toml`. To do so manually:

```bash
pip install -e python
```

---

## Repository Tooling

NavKit provides a collection of Python utilities under `tools/` that serve as the primary developer interface to the project.

These scripts provide a consistent cross-platform workflow and abstract away platform-specific differences between Windows and Linux.

| Script | Purpose |
|---------|---------|
| `build.py` | Configure, build, and rebuild NavKit using Conan and CMake |
| `run_tests.py` | Execute the complete unit test suite |
| `run_first_sim.py` | Run the default stationary GNSS simulation |
| `run_analysis.py` | Generate plots and analysis from simulation logs |
| `timing_report.py` | Summarize a complete `navkit.timing.v1` timing artifact |
| `resource_report.py` | Write and display coarse executable/library size reports for a build tree |
| `format.py` | Run clang-format; also exposes clang-tidy for CI diagnostics |
| `coverage.py` | Generate Linux/GCC-style coverage reports with gcovr |
| `copyright.py` | Insert or verify copyright headers |

For normal development, prefer these Python wrappers over invoking Conan, CMake, or CTest directly.

The raw commands are still documented throughout this guide for debugging and advanced workflows.

---

## Build NavKit with the Python Wrapper

The recommended interface is the Python build wrapper.

Clean configure and build:

```bash
python tools/build.py --build-type Debug --clean
```

Incremental configure and build while skipping Conan dependency installation:

```bash
python tools/build.py --build-type Debug --skip-conan
```

Fast rebuild only, with no Conan install and no CMake configure:

```bash
python tools/build.py --build-type Debug --build-only
```

Release build:

```bash
python tools/build.py --build-type Release --clean
```

Release compile verification without test targets:

```bash
python tools/build.py --build-type Release --clean --without-tests
```

In day-to-day development, `--build-only` is typically sufficient unless CMake configuration, Conan dependencies, or build-system files have changed.

Select a compile-time configuration with `--navkit-config`. The value is
relative to `config/compiletime`, and defaults to
`apps/navkit_sim/StationaryGnss.hpp`:

```bash
python tools/build.py --build-type Debug --skip-conan --navkit-config apps/navkit_sim/StationaryGnss.hpp
```

That app-level selection composes:

```text
config/compiletime/apps/navkit_sim/StationaryGnss.hpp
    -> config/compiletime/navkit/products/StationaryGnss.hpp
```

The app config is the executable composition selected by CMake. The NavKit
config is the reusable library configuration consumed by that app. App configs
also declare the simulator/emulator tuple and bind each emulator to a NavKit
sensor with a stable unsigned sensor ID.

To run the same stationary GNSS scenario with the embedded-style profiling
configuration:

```bash
python tools/build.py --build-type Debug --skip-conan --navkit-config apps/navkit_sim/ProfiledStationaryGnss.hpp
python tools/run_first_sim.py --build-type Debug
```

That writes `profile.csv` and `profile.trace.json` under
`data/logs/stationary_gnss_demo/`. The run wrapper reads the selected
compile-time config from the build manifest written by `build.py`; it does not
reselect compile-time configuration at run time. Use `--no-profile-trace` when
you want only the compact CSV profile export.

Use a separate build directory for each selected compile-time configuration:

```bash
python tools/build.py --build-type Debug --build-dir build/debug-stationary-gnss --navkit-config apps/navkit_sim/StationaryGnss.hpp
```

This keeps each generated `navkit/SelectedConfig.hpp` isolated. Debug/Release
and `NAVKIT_CONFIG` are independent: build type chooses compiler mode, while
`NAVKIT_CONFIG` chooses the top-level compile-time build configuration.

Run wrappers use `--build-type` to choose the Debug or Release executable from
the default build tree. When you intentionally keep multiple build trees for
different selected configs, pass the matching `--build-dir` to the run wrapper.

`CMakePresets.json` also contains example selected-config presets such as
`debug-stationary-gnss`, `debug-profiled-stationary-gnss`,
`release-stationary-gnss`, and `release-profiled-stationary-gnss`. As with any
Conan-backed CMake preset, install dependencies into the preset binary directory
before configuring if the toolchain file does not exist yet.

---

## CMake Target Layout

The root `CMakeLists.txt` is intentionally an orchestration layer. Header-only/interface target definitions live under `cmake/targets/`, while compiled source targets stay with their sources:

```text
cmake/targets/NavKitCore.cmake   navkit_core / navkit::core
cmake/targets/NavKitIo.cmake     navkit_io / navkit::io
src/sim/CMakeLists.txt           navkit_sim / navkit::sim
src/app_support/CMakeLists.txt   navkit_app_support / navkit::app_support
```

Applications should link only the product-boundary targets they need. For
example, `apps/navkit_sim` links `navkit::app_support` for selected-config
description, JSON-input, runtime-validation, estimator-alias, and profile-export
helpers, while its `main.cpp` remains a thin selected-config entry point. The
generic `SimulationApp<Config>` loop lives in app support and is selected by the
app compile-time config.
`apps/navkit_replay` currently links only `navkit::core`.

For target boundaries, namespaces, and the header-only versus compiled-library
rationale, see [`ARCHITECTURE.md`](ARCHITECTURE.md).

For the compile-time configuration model, domain-specific config concepts,
example config contracts, and the `NAVKIT_CONFIG` build selection flow, see
[`CONFIGURATION.md`](CONFIGURATION.md).

---

## Build NavKit Manually with Conan and CMake

The Python wrapper is preferred, but the raw commands are useful for debugging.

From the repository root, install dependencies with Conan:

```bash
conan install . --output-folder build/Debug --build=missing -s build_type=Debug
```

Conan 2 usually writes the generated CMake toolchain file to:

```text
build/Debug/build/generators/conan_toolchain.cmake
```

Configure CMake manually:

```bash
cmake -S . -B build/Debug -DCMAKE_TOOLCHAIN_FILE=build/Debug/build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
```

To select a compile-time config manually, add `-DNAVKIT_CONFIG=...`:

```bash
cmake -S . -B build/Debug -DCMAKE_TOOLCHAIN_FILE=build/Debug/build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug -DNAVKIT_CONFIG=apps/navkit_sim/StationaryGnss.hpp
```

Build manually:

```bash
cmake --build build/Debug --config Debug
```

For Release, replace `Debug` with `Release`:

```bash
conan install . --output-folder build/Release --build=missing -s build_type=Release
cmake -S . -B build/Release -DCMAKE_TOOLCHAIN_FILE=build/Release/build/generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build/Release --config Release
```

Notes:

- `--config Debug` or `--config Release` is required for multi-config generators such as Visual Studio.
- `CMAKE_BUILD_TYPE` is primarily used by single-config generators such as Ninja or Unix Makefiles.
- The Python wrapper handles these differences and is preferred for normal development.

---

## Run Unit Tests

Run all unit tests:

```bash
python tools/run_tests.py --build-type Debug
```

If the project has already been built, this runs CTest against the selected build directory.

Equivalent raw CTest command:

```bash
ctest --test-dir build/Debug --output-on-failure -C Debug
```

For Release:

```bash
ctest --test-dir build/Release --output-on-failure -C Release
```

---

## Run the First Simulation

Run the first GNSS-only stationary simulation:

```bash
python tools/run_first_sim.py --build-type Debug
```

Or manually, after building:

Windows Debug:

```powershell
build\Debug\apps\navkit_sim\Debug\navkit_sim.exe config\runtime\navkit_sim\stationary_gnss.json
```

Windows Release:

```powershell
build\Release\apps\navkit_sim\Release\navkit_sim.exe config\runtime\navkit_sim\stationary_gnss.json
```

Linux Debug:

```bash
build/Debug/apps/navkit_sim/navkit_sim config/runtime/navkit_sim/stationary_gnss.json
```

The simulation generates logs under:

```text
data/logs/<run_name>/
```

Expected outputs:

```text
truth.csv
truth.meta.json

gnss.csv
gnss.meta.json

nav.csv
nav.meta.json

run_manifest.json
timing.json
```

CSV is used for time-history logs. JSON is used for runtime input bundles,
per-log metadata, the hierarchical run manifest, and lightweight workflow
timing artifacts. Runtime input bundles such as
`config/runtime/navkit_sim/stationary_gnss.json` are executable inputs, not
NavKit library compile-time configuration. The selected app validates the
runtime JSON before running; for the stationary GNSS app, that means required
`trajectory` and `gnss` sections must exist, unsupported `imu` or `baro`
sections are rejected, and common vector/numeric fields must have the expected
shape.

---

## Plot Results

Example:

```bash
python python/navkit_analysis/plots.py data/logs/stationary_gnss_demo
```

Or, to display an interactive browser of the figures:
```bash
python python/navkit_analysis/plots.py data/logs/stationary_gnss_demo --show
```

Future plotting utilities will include:

- Truth vs estimate
- Position error
- 3-sigma covariance bounds
- Innovation history
- NEES
- NIS
- Monte Carlo statistics

The analysis package is intentionally separated from the embedded navigation library to allow richer offline validation, visualization, Monte Carlo analysis, and future report generation without impacting embedded flight software.

---

## Collect Timing and Resource Artifacts

The simulation and analysis wrappers automatically update:

```text
data/logs/<run_name>/timing.json
```

For the default stationary GNSS workflow:

```bash
python tools/run_first_sim.py --build-type Debug
python tools/run_analysis.py data/logs/stationary_gnss_demo
```

Both commands update `timing.json` and print a compact timing summary for the
operation they just ran. Add `--no-timing-report` to either command when script
output needs to stay quiet.

To view the timing artifact as a compact terminal report:

```bash
python tools/timing_report.py data/logs/stationary_gnss_demo/timing.json
```

Build, test, simulation, and analysis wrappers update the default timing
artifact during normal use:

```bash
python tools/build.py --build-type Debug --skip-conan
python tools/run_tests.py --build-type Debug
python tools/run_first_sim.py --build-type Debug
python tools/run_analysis.py data/logs/stationary_gnss_demo
```

Use `--timing-output <path>` on `build.py` or `run_tests.py` to write to a
different timing artifact. Use `--no-timing` on those wrappers to disable timing
updates for a single command.

`build.py` and `run_tests.py` also print concise timing summaries by default.
Use `--no-timing-report` when the timing artifact should still update but the
terminal output should stay quiet.

To write a coarse executable/library size report for a build tree:

```bash
python tools/resource_report.py --build-type Debug --output data/logs/stationary_gnss_demo/resources-debug-local.json
```

`build.py` writes and displays the same coarse artifact-size report by default
after a successful build. Use `--resource-output <path>` to change where that
report is written, or `--no-resource-report` to skip it for a single build.

For a Release size snapshot:

```bash
python tools/build.py --build-type Release --clean --without-tests
python tools/resource_report.py --build-type Release --output data/logs/stationary_gnss_demo/resources-release-local.json
```

These files are intended for trend review and future Monte Carlo summaries.
They are deliberately not pass/fail gates because wall-clock timing and binary
layout vary across local machines, compilers, and CI runners.

---

## Format, Lint, and Copyright

NavKit uses repository-wide formatting and copyright tools.

Insert or update copyright headers:

```bash
python tools/copyright.py --write
```

Verify all source files contain the required copyright header:

```bash
python tools/copyright.py --check
```

Format all source files:

```bash
python tools/format.py
```

Check formatting without modifying files:

```bash
python tools/format.py --check
```

Formatting behavior is controlled by:

```text
.clang-format
```

Static-analysis rules are controlled by:

```text
.clang-tidy
```

Basic editor whitespace behavior is controlled by:

```text
.editorconfig
```

---

## Compiler Diagnostics and Build Profiles

NavKit applies warning and Release optimization settings through
`cmake/NavKitWarnings.cmake` for NavKit-owned targets.

Warnings are enabled by default:

- MSVC: `/W4`, `/permissive-`, and `/Zc:__cplusplus`
- GCC/Clang: `-Wall`, `-Wextra`, `-Wpedantic`, `-Wconversion`,
  `-Wsign-conversion`, and `-Wshadow`

Warnings-as-errors are intentionally opt-in while the architecture is still
moving quickly for local development, but CI enables them for NavKit-owned
targets:

```bash
python tools/build.py --build-type Debug --skip-conan --warnings-as-errors
```

Debug builds also enable extra supported diagnostics:

- MSVC Debug: `/sdl`
- GCC/Clang Debug: `-fno-omit-frame-pointer`

Release builds use an embedded-oriented optimization profile that favors a
portable, conservative speed baseline plus dead-code elimination support:

- MSVC Release: `/O2`, `/Ob2`, `/Gy`, `/Gw`, `/OPT:REF`, and `/OPT:ICF`
- GCC/Clang Release: `-O2`, `-ffunction-sections`, `-fdata-sections`, and
  linker garbage collection (`--gc-sections`, or `dead_strip` on Apple)

CI builds and tests Debug, then also verifies that Release compiles. Run a
Release build locally when changing performance-sensitive code or compiler
configuration. For a compile-only optimization-profile check, omit test targets:

```bash
python tools/build.py --build-type Release --clean --without-tests --warnings-as-errors
```

CI runs clang-tidy on Linux as the canonical static-analysis gate. Local
development does not require clang-tidy; run it locally only when explicitly
debugging the CI static-analysis lane.

CI also generates a Linux coverage artifact with `tools/coverage.py`. Local
development does not require coverage reporting; use it only when reviewing
coverage gaps or debugging the coverage lane.

Build, test, simulation, and analysis wrappers write a lightweight
`timing.json` artifact under `data/logs/<run_name>/` during normal use. CI
uploads those logs along with Debug and Release resource-size reports produced by
`tools/resource_report.py`. These artifacts are trend evidence only; wall-clock
timing and hosted-runner binary sizes are intentionally not pass/fail gates.

---

## VS Code Debugging

NavKit provides VS Code launch configurations for debugging.

Recommended workflow:

1. Build a Debug configuration manually:

```bash
python tools/build.py --build-type Debug --build-only
```

If this is the first Debug build, run:

```bash
python tools/build.py --build-type Debug --clean
```

2. Open the repository root in VS Code.

3. Press **F5** and select one of the launch configurations:

- **Windows Debug navkit_sim**
- **Windows Debug navkit_tests**
- **Linux Debug navkit_sim**
- **Linux Debug navkit_tests**

The launch configurations intentionally assume a Debug build already exists and do not invoke the build system automatically. This keeps debugger startup fast and avoids issues related to Python virtual environments, Conan, and shell activation.

Useful first breakpoints:

```text
apps/navkit_sim/main.cpp

include/navkit/core/estimation/navigator/Navigator.hpp
include/navkit/core/estimation/filter/KalmanFilter.hpp
include/navkit/core/estimation/sensor/Sensor.hpp
include/navkit/core/models/GnssPosModel.hpp
include/navkit/core/estimation/filter/injection/InjectionPolicies.hpp
```

Useful call path for the first simulation:

```text
main.cpp
    -> navigator.process_measurements()
    -> Navigator::process_one_sensor()
    -> KalmanFilter::process_sensor()
    -> KalmanFilter::observation_update()
    -> GnssPosModel::obs_impl()
    -> KalmanFilter::covariance_update()
    -> UpdatePostFilter::filter_update()
    -> KalmanFilter::inject()
    -> InsInjectionPolicy::apply()
    -> KalmanFilter::reset()
```

Because NavKit uses templates and Eigen, stepping line-by-line can enter a lot of Eigen internals. Prefer setting breakpoints directly inside NavKit functions and using **Step Over** aggressively.

---

## Recommended Development Workflow

A typical NavKit development cycle is:

```text
git pull

activate virtual environment

python tools/copyright.py --write
python tools/format.py

python tools/copyright.py --check
python tools/format.py --check

python tools/build.py --build-type Debug --build-only

python tools/run_tests.py --build-type Debug

python tools/run_first_sim.py --build-type Debug

python tools/run_analysis.py data/logs/stationary_gnss_demo --show

update CHANGELOG.md for changes worth tracking

reconcile README.md and docs/SETUP.md when user-facing behavior, layout,
tooling, or workflow changes

git status
git add ...
git commit
```

All source-mutating copyright and formatting steps occur before the final build and tests. Changelog and documentation reconciliation should happen before the final checks so the reviewed change includes code, build metadata, and user-facing guidance together. CI runs only the check forms, then builds and tests the exact checked source.

---

## Codex Cloud and CI

For a fresh Codex cloud environment, use this setup command:

```bash
python tools/bootstrap.py
```

Do not rely on dependencies persisting between independent cloud tasks. The bootstrap command is safe to rerun, while CI caches pip downloads and the Conan cache to reduce cold-start time.

The GitHub Actions workflow in `.github/workflows/ci.yml` enforces this order:

1. Copyright and formatting checks on Linux.
2. C++23 Debug builds with warnings-as-errors on Linux and Windows.
3. `clang-tidy` static analysis on the Linux Debug compilation database.
4. Unit tests on both platforms.
5. Stationary simulation and headless analysis smoke tests on both platforms.
6. Release compile checks with warnings-as-errors on both platforms.
7. Linux coverage report generation as an uploaded artifact.

The build/test jobs wait for source checks, ensuring CI never tests code that would subsequently be changed by formatting.

### Common Build Scenarios

Incremental rebuild:

```bash
python tools/build.py --build-type Debug --build-only
```

Reconfigure CMake without reinstalling Conan dependencies:

```bash
python tools/build.py --build-type Debug --skip-conan
```

Full clean rebuild:

```bash
python tools/build.py --build-type Debug --clean
```

Release build:

```bash
python tools/build.py --build-type Release --clean
```

---
