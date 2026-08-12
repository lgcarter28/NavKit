# Runtime Inputs

This directory contains runtime inputs for applications, simulation, validation,
and analysis workflows. Runtime inputs answer "what are we running today?";
compile-time configuration answers "what product or application are we
building?"

The NavKit-owned runtime graph lives under `navkit/`:

```text
navkit/
  components/
    mission/              target-independent phase plans
    gnc/
      guidance/           reusable Guidance configuration
      navigation/         reusable Navigation phase actions and initialization
      autopilot/          reusable Autopilot configuration
      filters/            reusable GNC command and estimator-filter configuration
    simulation/           SWIL truth, plant, environment, and sensor models
    execution/            target adapter, clock, and planned cadence
  scenario/               complete runnable composition roots
```

Scenarios use explicit named fields rather than a generic `components` merge
table. Any moderately complex object may be written inline or linked with the
same reference form:

```json
{
  "mission": {
    "config": "../components/mission/stationary_ecef.json",
    "overrides": {
      "duration_s": 120.0
    }
  }
}
```

`config` paths are resolved relative to the file containing the reference, not
the process working directory or the top-level scenario. `overrides` is
optional and is deep-merged after the referenced file resolves. A reference
object may contain only `config` and `overrides`. Nested objects merge
recursively, while arrays and scalar values are replaced wholesale; ambiguous mixtures of linked
and inline fields, missing files, and reference cycles are validation errors.
The resolved `effective_runtime_config.json` written beside each run is the
self-contained replay artifact.

One `mission.phases` array is the authoritative ordered phase graph. A phase
owns its Navigation, Guidance, and Autopilot selections inline or through
explicit references. There are no parallel Navigation, Guidance, Autopilot, or
plant phase catalogs for the application to join implicitly. Synthetic plant
behavior belongs to `simulation` and is keyed by the stable mission phase ID;
validation requires an exact mapping with no missing or orphaned simulation
phase behavior.

The scenario is a thin composition root that selects mission intent, execution
target, simulation model, sensor models, estimator initialization, logging, and
local overrides. SWIL owns synthetic truth and plant behavior. Future HWIL,
flight, and external-framework adapters reuse the mission and Navigation phase
contracts while replacing simulation-specific acquisition and publication.
Unsupported execution targets fail closed until their lifecycle and transport
adapters exist.

The current SWIL adapter supports multi-phase missions only for generated
sources. Stationary and CSV sources accept one mission phase and reject a
multi-phase graph until phase advancement moves into a target-independent
mission runtime.

Complete scenario names follow `<product>_<mission>_<purpose>.json`; reusable
components name the local contract they provide instead of repeating the full
product name. See `docs/CONFIGURATION.md` for the complete runtime contract and
naming rules.
