# NavKit Testing Strategy

NavKit tests should explain design intent as much as they verify behavior. The
goal is not raw test count; the goal is a suite that makes extension contracts,
failure behavior, and numerical expectations hard to misunderstand.

## Test layers

| Layer | Purpose | Current examples |
|---|---|---|
| Compile-time concept tests | Prove public policy boundaries accept valid types and reject invalid ones without intentionally uncompilable targets. | `test_config_policy.cpp`, `test_injection_reset_policy.cpp`, `test_measurement_policy.cpp`, `test_noise_policy.cpp`, `test_navigator_policy.cpp`, `test_state_def_policy.cpp` |
| Core behavior tests | Verify small deterministic product-core contracts. | ring buffers, segments, frame/unit helpers, sensor FIFO behavior |
| Numerical estimator tests | Verify update math, statistics, accepted/rejected behavior, and future propagation behavior. | GNSS position update, measurement statistics |
| Environment/model tests | Verify policy capabilities and physics/model semantics at a stable tolerance. | planet/gravity policy tests |
| Simulation and IO tests | Verify executable support code produces expected data contracts. | trajectory generation, CSV writer behavior |
| End-to-end smoke tests | Verify the configured app/demo still runs and analysis can consume its logs. | `tools/run_first_sim.py`, `tools/run_analysis.py` |

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
local workflow. Coverage reporting is also CI-oriented; run it locally only when
reviewing coverage gaps or debugging the CI coverage lane.
