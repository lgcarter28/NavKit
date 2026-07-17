# NavKit Testing Strategy

NavKit tests should explain design intent as much as they verify behavior. The
goal is not raw test count; the goal is a suite that makes extension contracts,
failure behavior, and numerical expectations hard to misunderstand.

## Test layers

| Layer | Purpose | Current examples |
|---|---|---|
| Compile-time concept tests | Prove public policy boundaries accept valid types and reject invalid ones without intentionally uncompilable targets. | `test_config_policy.cpp`, `test_injection_reset_policy.cpp`, `test_measurement_model_policy.cpp`, `test_noise_policy.cpp`, `test_navigator_policy.cpp`, `test_profiling_policy.cpp`, `test_state_def_policy.cpp` |
| Core behavior tests | Verify small deterministic product-core contracts. | ring buffers, segments, frame/unit helpers, sensor FIFO behavior |
| Numerical estimator tests | Verify update math, statistics, accepted/rejected behavior, and future propagation behavior. | GNSS position update, measurement statistics |
| Environment/model tests | Verify policy capabilities and physics/model semantics at a stable tolerance. | planet/gravity policy tests |
| Simulation and IO tests | Verify executable support code produces expected data contracts. | trajectory generation, CSV writer behavior |
| End-to-end smoke tests | Verify the configured app/demo still runs and analysis can consume its logs. | `tools/run_sim.py`, `tools/run_analysis.py`, `tools/run_scenario.py` |

## Standards for new tests

- Add every new test source to `tests/CMakeLists.txt`; files in `tests/` are
  not discovered automatically.
- Prefer positive and negative `static_assert` coverage for concepts. Negative
  cases should compile and assert `!Concept<Bad>`, not rely on intentionally
  broken build targets.
- Runtime tests should cover both normal behavior and stable expected failures,
  such as rejected overflow, invalid output paths, rejected measurements, or
  unsupported configurations.
- Use deterministic seeds and fixed tolerances for numerical tests. Avoid
  stochastic pass/fail gates from a single noisy run.
- Keep tests domain-focused. A test should make one contract clear rather than
  exercise half the stack accidentally.
- When a test encodes a temporary limitation, say so in the test name or nearby
  documentation so future implementation can deliberately replace it.

## Coverage reporting

Coverage is useful only after the configured test target represents the intended
suite. The near-term priority is meaningful domain coverage and design-intent
tests. Line/branch coverage reporting should be added after this baseline is
stable, and coverage gaps should be reviewed by engineering domain rather than
treated as a blind percentage chase.

Linux CI generates a coverage artifact using GCC/Clang-style coverage
instrumentation and `gcovr`. Local Windows development does not need to run
coverage. To reproduce the CI coverage path on a machine with compatible
tooling:

```bash
python tools/coverage.py --html
```

Coverage reports are written under `build/coverage/coverage/`. Treat the report
as a review aid for finding meaningful gaps, not as a standalone quality score.

## Local and CI workflow

The normal local agentic workflow runs formatting checks, Debug build, and the
configured doctest executable. Simulation and analysis smoke tests are added
when behavior affects logs, navigation results, or runtime app wiring.

Clang-tidy is intentionally a CI static-analysis gate, not part of the normal
local workflow. The Python build wrappers use Ninja for the official default
layout, which produces the `compile_commands.json` needed by tidy in the
selected config build directory. If you override the generator locally with an
explicit build directory, choose one that emits a compilation database before
running tidy. Coverage reporting is also CI-oriented; run it
locally only when reviewing coverage gaps or debugging the CI coverage lane.

## Timing and resource artifacts

Simulation and analysis smoke tests write lightweight timing data to
`output/logs/<run_name>/timing.json`. CI also preserves coarse Debug and Release
executable/library size reports from `tools/resource_report.py` with the
stationary GNSS logs. These artifacts are intended for trend review and future
Monte Carlo summaries; they are not pass/fail tests because local machines and
hosted runners vary too much for wall-clock thresholds to be meaningful yet.

For a local timing smoke pass:

```bash
python tools/build.py --build-type Debug --skip-conan
python tools/run_tests.py --build-type Debug
python tools/run_scenario.py --build-type Debug --no-plot
python tools/run_analysis.py output/logs/ecef_ins_gnss_demo
python tools/timing_report.py output/logs/ecef_ins_gnss_demo/timing.json
python tools/resource_report.py --build-type Debug --output output/logs/ecef_ins_gnss_demo/resources-debug-local.json
```

Build, test, simulation, and analysis wrappers update the default timing
artifact during normal use. Each wrapper prints a concise timing summary for the
operation it just ran; `timing_report.py` prints the accumulated artifact. Use
`--no-timing-report` when a quieter wrapper run is needed. Use `--no-timing` on
`build.py` or `run_tests.py` when a command should not update that artifact.
`build.py` also writes and displays a coarse executable/library size report by
default after successful builds.

See `docs/SETUP.md` for the fuller timing/resource workflow and Release size
snapshot commands.

Profiling records use a separate embedded-facing path from coarse wrapper
timing. See `docs/PROFILING.md` for `ProfileRecord`, fixed-capacity sink, CSV
export, and Chrome Trace / Perfetto conversion details.

### `navkit.timing.v1` schema

Timing artifacts use the lightweight `navkit.timing.v1` JSON schema. The schema
identifier is defined in `tools/perf_artifacts.py` and recorded in each artifact
so future tooling can reject incompatible files instead of silently producing
misleading reports.

Required top-level fields:

- `schema`: currently `navkit.timing.v1`.
- `run_name`: logical run name, such as `ecef_ins_gnss_demo`.
- `created_utc`: timestamp for first artifact creation.
- `updated_utc`: timestamp for the most recent record update.
- `environment`: object with at least `platform` and `python`.
- `build`: latest build metadata object that may contain `build_type` and
  `navkit_config`.
- `commands`: object keyed by stable command names.

Each command record contains:

- `command`: argv-style command list.
- `tool_version`: wrapper/tool identity, currently the script name.
- `build_type` and `navkit_config`: command-local build metadata when known.
- `start_utc` and `end_utc`: command timing timestamps.
- `elapsed_s`: wall-clock elapsed seconds.
- `return_code`: process or wrapper return code.

The schema is intentionally small and trend-oriented. It is suitable for local
inspection, CI artifact preservation, and future Monte Carlo aggregation, but it
does not define pass/fail thresholds.
