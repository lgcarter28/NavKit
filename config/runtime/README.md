# Runtime Inputs

This directory contains runtime inputs for applications, simulators, demos, and
analysis workflows.

Runtime inputs answer "what scenario are we running today?" They are separate
from compile-time configuration, which answers "what product or app are we
compiling?"

See `docs/CONFIGURATION.md` for the runtime naming contract. Complete scenarios
live under `navkit_sim/scenario/` and follow
`<product>_<trajectory>_<purpose>.json`; reusable components identify the local
contract they require rather than repeating the entire product name.
