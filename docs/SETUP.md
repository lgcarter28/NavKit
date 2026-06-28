# NavKit Development Environment Setup

This guide walks through setting up a complete development environment for NavKit.

---

# Prerequisites

NavKit is developed using:

- C++23
- CMake
- Conan 2.x
- Python
- VS Code
- MSVC (Windows) or GCC/Clang (Linux/macOS)

The build system is intentionally cross-platform.

---

# 1. Install Visual Studio 2026 (Windows)

Install **Visual Studio 2026 Community**.

During installation, select the following workload:

- **Desktop Development with C++**

Under the optional components, ensure the following are installed:

- Latest MSVC C++ Toolset
- Latest Windows SDK
- C++ CMake Tools for Windows
- Ninja Build
- Git for Windows (optional, if not already installed)

Although development will primarily occur in **VS Code**, Visual Studio provides the Microsoft C++ compiler (MSVC), linker, debugger, Windows SDK, and build tools required by CMake and Conan.

After installation, verify the compiler is available:

```powershell
cl
```

You should also verify CMake is installed:

```powershell
cmake --version
```

NavKit targets **C++23**, and the latest Visual Studio 2026 MSVC toolset fully supports the language features used throughout the project.

---

# 2. Install VS Code

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
- CodeLLDB (optional)

The repository also provides recommended extensions through `.vscode/extensions.json`.

---

# 3. Install Python

Install the latest stable version of Python (3.12+ recommended).

Verify installation:

```bash
python --version
```

---

# 4. Create a Python Virtual Environment

A dedicated virtual environment is recommended to isolate Conan and Python dependencies.

From the repository root:

```bash
python -m venv .venv
```

Activate it.

Windows:

```bash
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

# 5. Install Conan

Inside the virtual environment:

```bash
pip install conan
```

Verify:

```bash
conan --version
```

---

# 6. Detect Your Compiler

Run once:

```bash
conan profile detect
```

This generates your default Conan profile.

Future versions of NavKit will provide repository-specific profiles such as:

```
profiles/
    windows-msvc-debug
    windows-msvc-release
    stm32f4-gcc
    stm32h7-gcc
```

---

# 7. Install CMake

Install CMake 3.23 or newer.

Verify:

```bash
cmake --version
```

---

# 8. Install Conan Dependencies

From the repository root:

```bash
conan install . --output-folder=build --build=missing -s build_type=Debug
```

The current dependencies are:

- Eigen
- nlohmann_json
- doctest

---

# 9. Build NavKit

The recommended method is via the Python wrapper:

```bash
python tools/build.py
```

Debug build:

```bash
python tools/build.py --build-type Debug
```

Release build:

```bash
python tools/build.py --build-type Release
```

Internally this executes:

```bash
conan install ...

cmake --preset ...

cmake --build ...
```

Developers should rarely need to invoke CMake manually.

---

# 10. Running Tests

Run all unit tests:

```bash
python tools/run_tests.py
```

This script builds (if necessary), executes CTest, and summarizes results.

---

# 11. Running the First Simulation

Execute:

```bash
python tools/run_first_sim.py
```

or manually:

```bash
build/.../navkit_sim apps/navkit_sim/configs/stationary_gnss.json
```

The simulation generates:

```
truth.csv
truth.meta.json

gnss.csv
gnss.meta.json

nav.csv
nav.meta.json

run_manifest.json
```

---

# 12. Python Analysis Environment

Install the analysis packages:

```bash
pip install numpy pandas matplotlib scipy plotly pyarrow
```

Eventually these will be managed directly from `pyproject.toml`.

---

# 13. Plotting Results

Example:

```bash
python python/navkit_analysis/plots.py data/logs/stationary_demo
```

Future plotting utilities will include:

- Truth vs Estimate
- Error
- 3σ Covariance Bounds
- Innovation History
- NEES
- NIS
- Monte Carlo Statistics

---

# 14. Formatting

Install LLVM (clang-format / clang-tidy).

Run:

```bash
python tools/format.py
```

This will execute:

- clang-format
- clang-tidy

across the repository.

---

# 15. Recommended Daily Workflow

```text
git pull

activate virtual environment

python tools/build.py

python tools/run_tests.py

python tools/run_first_sim.py

python python/navkit_analysis/plots.py ...

git commit
```

---

# Repository Philosophy

NavKit is intended to be:

- Modern C++23
- Cross-platform
- Compile-time configurable
- Deterministic
- Embedded-friendly
- Header-heavy for generic framework components
- Source-based for simulations, utilities, and applications

The initial development focus is on correctness, testing, and clean architecture before embedded deployment, hardware abstraction layers, and smoothing algorithms are introduced.