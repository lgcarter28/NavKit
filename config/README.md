# NavKit Configuration Tree

Repository-provided configuration is split by when it is selected:

- `compiletime/`: C++ headers selected when configuring/building an application
  or product target.
- `runtime/`: JSON and other runtime inputs consumed by executables,
  simulators, demos, and analysis workflows.

See [`docs/CONFIGURATION.md`](../docs/CONFIGURATION.md) for the configuration
model and extension guidance.
