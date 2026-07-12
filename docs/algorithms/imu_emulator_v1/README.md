# IMU Emulator v1

This folder contains the implementation-oriented IMU emulator and triad error
model specification for the first NavKit INS simulation path.

Build locally with:

```powershell
latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
```

The document is intentionally narrower than the broader navigation reference.
It exists to define the equations, naming, runtime configuration shape, and
tests required before the strapdown Navigator consumes simulated IMU
increments.
