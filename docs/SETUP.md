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
- LLVM tooling for formatting and static analysis

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

A dedicated virtual environment is recommended to isolate Conan and Python analysis dependencies.

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

## Install LLVM for Formatting and Linting

NavKit uses repository-wide formatting and static-analysis configuration stored in:

```text
.clang-format
.clang-tidy
.editorconfig
```

These files live at the repository root and are automatically used by VS Code, `clang-format`, and `clang-tidy`.

### Windows

Install LLVM:

```powershell
winget install LLVM.LLVM
```

Restart VS Code or your terminal, then verify:

```powershell
clang-format --version
clang-tidy --version
```

If the commands are not found, add the LLVM `bin` directory to your `PATH`, typically:

```text
C:\Program Files\LLVM\bin
```

### Linux

Install Clang tooling:

```bash
sudo apt update
sudo apt install clang-format clang-tidy
```

Verify:

```bash
clang-format --version
clang-tidy --version
```

---

## Install Python Analysis Dependencies

Install the Python analysis packages:

```bash
pip install numpy pandas matplotlib scipy plotly pyarrow
```

Eventually these may be managed directly from:

```text
python/pyproject.toml
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
| `format.py` | Run clang-format and optional clang-tidy |
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

In day-to-day development, `--build-only` is typically sufficient unless CMake configuration, Conan dependencies, or build-system files have changed.

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
build\Debug\apps\navkit_sim\Debug\navkit_sim.exe apps\navkit_sim\configs\stationary_gnss.json
```

Windows Release:

```powershell
build\Release\apps\navkit_sim\Release\navkit_sim.exe apps\navkit_sim\configs\stationary_gnss.json
```

Linux Debug:

```bash
build/Debug/apps/navkit_sim/navkit_sim apps/navkit_sim/configs/stationary_gnss.json
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
```

CSV is used for time-history logs. JSON is used for configuration, per-log metadata, and the hierarchical run manifest.

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
- 3σ covariance bounds
- Innovation history
- NEES
- NIS
- Monte Carlo statistics

The analysis package is intentionally separated from the embedded navigation library to allow richer offline validation, visualization, Monte Carlo analysis, and future report generation without impacting embedded flight software.

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

Run static analysis:

```bash
python tools/format.py --tidy
```

Apply available clang-tidy fixes:

```bash
python tools/format.py --tidy --fix
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

include/navkit/core/Navigator.hpp
include/navkit/core/KalmanFilter.hpp
include/navkit/core/Sensor.hpp
include/navkit/models/GnssPosModel.hpp
include/navkit/core/policies/InjectionPolicies.hpp
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

python tools/build.py --build-type Debug --build-only

python tools/run_tests.py --build-type Debug

python tools/run_first_sim.py --build-type Debug

python tools/run_analysis.py data/logs/stationary_gnss_demo --show

git status
git add ...
git commit
```

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
