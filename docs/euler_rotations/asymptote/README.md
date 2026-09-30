<!-- Copyright (c) 2026 William Gordon Carter. -->
<!-- All Rights Reserved. -->

# Asymptote 3-2-1 Euler sequence

`euler_321_sequence.asy` is the sole authored Asymptote source for this
two-page comparison artifact. It owns the shared angles, camera, palette,
Rodrigues construction, geometry helpers, and both page layouts. Page 1 shows
(a) yaw, (b) pitch, and (c) roll strictly left-to-right. Page 2 shows the same
construction in one combined view.

The inputs match the TikZ source exactly: yaw is 38 degrees, pitch is 27
degrees, roll is 32 degrees, and the navigation/intermediate/body frames use
the same blue (`#187EA8`), teal (`#15967D`), amber (`#D08B25`), and red
(`#C94C3A`) palette.

Asymptote's deferred 3-D objects do not remain isolated reliably across a
native `newpage()` boundary with this vector render path. The single source is
therefore invoked once per page mode, and the two vector pages are losslessly
assembled into the canonical `euler_321_sequence.pdf`.

From the repository root:

```powershell
New-Item -ItemType Directory -Force `
  build/docs/euler_rotations/asymptote | Out-Null

asy -cd docs/euler_rotations/asymptote -u sequence `
  -tex pdflatex -render 0 -f pdf `
  -o ../../../build/docs/euler_rotations/asymptote/euler_321_sequence_page_1 `
  euler_321_sequence.asy

asy -cd docs/euler_rotations/asymptote -u combined `
  -tex pdflatex -render 0 -f pdf `
  -o ../../../build/docs/euler_rotations/asymptote/euler_321_sequence_page_2 `
  euler_321_sequence.asy

pdfunite `
  build/docs/euler_rotations/asymptote/euler_321_sequence_page_1.pdf `
  build/docs/euler_rotations/asymptote/euler_321_sequence_page_2.pdf `
  build/docs/euler_rotations/asymptote/euler_321_sequence.pdf
```

`-render 0` preserves vector geometry. Generated page intermediates,
inspection PNGs, and Asymptote/LaTeX auxiliaries belong only under the ignored
`build/` tree; after verification, retain only the canonical two-page PDF.
