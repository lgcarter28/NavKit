<!-- Copyright (c) 2026 William Gordon Carter. -->
<!-- All Rights Reserved. -->

# TikZ 3-2-1 Euler sequence

`euler_321_sequence.tex` is a two-page, standalone, vector-first academic
figure. Page 1 shows the right-handed yaw-pitch-roll frame construction as
three explicit left-to-right transitions: (a) yaw, (b) pitch, and (c) roll.
Page 2 combines the same derived geometry into a
single composition view with the initial, intermediate, and final frames plus
all three rotation planes and arcs. The solid axes are successive frame-basis
images, not a claim that the drawn physical rotation is itself passive.
Separately, each page states NavKit's passive component-transform convention
explicitly:

```text
v^b = C_n^b v^n,
C_n^b = R_3(psi) R_2(theta) R_1(phi).
```

The main edit points are `\YawAngle`, `\PitchAngle`, `\RollAngle`, the camera
angles passed to `\tdplotsetmaincoords`, and the visual styles near the top of
the file. Axis endpoints are written directly from the successive frame-basis
construction so that the geometry remains inspectable in source.

From the repository root, build the two-page vector PDF and separate 600 dpi
inspection images with:

```powershell
latexmk -pdf -interaction=nonstopmode -halt-on-error `
  -outdir=build/docs/euler_rotations/tikz `
  docs/euler_rotations/tikz/euler_321_sequence.tex

pdftocairo -png -r 600 `
  build/docs/euler_rotations/tikz/euler_321_sequence.pdf `
  build/docs/euler_rotations/tikz/euler_321_sequence_page
```

The raster previews are `euler_321_sequence_page-1.png` for the broken-out
construction and `euler_321_sequence_page-2.png` for the combined view.

The authored `.tex` file is the source of truth. PDF, PNG, and LaTeX auxiliary
files belong only under the ignored `build/` tree.
