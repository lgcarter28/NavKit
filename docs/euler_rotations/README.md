<!-- Copyright (c) 2026 William Gordon Carter. -->
<!-- All Rights Reserved. -->

# Euler-rotation publication-figure study

This directory is a controlled comparison of TikZ and Asymptote for a
publication-quality 3-2-1 Euler frame-construction figure. It is an experiment
in authored figure workflow, not the canonical definition of NavKit's attitude
contract. The mathematical source of truth remains the owning algorithm or
reference document.

Both implementations show the same successive, right-handed frame-basis
construction

```text
F^n --yaw about z_n--> F^1 --pitch about y_1--> F^2
    --roll about x_2--> F^b
```

and state NavKit's corresponding passive component transform explicitly:

```text
v^b = C_n^b v^n
C_n^b = R_3(psi) R_2(theta) R_1(phi)
```

The distinction matters: the solid axes illustrate successive images of a
moving frame, while `C_n^b` maps vector components from the local-level frame
to the body frame.

## Implementations

- [`tikz/euler_321_sequence.tex`](tikz/euler_321_sequence.tex) uses
  `tikz-3dplot`, explicit projected-coordinate expressions, and native LaTeX
  text. See [`tikz/README.md`](tikz/README.md) for edit points and build
  commands.
- [`asymptote/euler_321_sequence.asy`](asymptote/euler_321_sequence.asy) uses
  native three-dimensional triples, Rodrigues rotations, an orthographic
  camera, and reusable geometry helpers. The one source renders both the
  staged and combined views. See
  [`asymptote/README.md`](asymptote/README.md) for build commands.

Generated PDFs, PNG previews, and renderer intermediates belong under
`build/docs/euler_rotations/` and are intentionally ignored. The authored
`.tex` and `.asy` files beside this README are the source of truth.

The canonical comparison artifacts are structurally equivalent two-page
PDFs. In each one, page 1 presents (a) yaw, (b) pitch, and (c) roll strictly
left-to-right, while page 2 presents the combined construction. TikZ emits
both pages directly; Asymptote renders its two native 3-D pages independently
and losslessly concatenates them to avoid deferred-object leakage across an
Asymptote `newpage()` boundary.

The comparison inputs are deliberately identical: yaw is 38 degrees, pitch
is 27 degrees, roll is 32 degrees, and both sources use the same hexadecimal
blue/teal/amber/red frame palette. Both use the same orthographic camera view
direction `(8,16,9)` with `+z` as screen up; the TikZ polar/azimuth parameters
are the exact projection-equivalent representation of that vector. The
canonical source and output basename is `euler_321_sequence` for both methods.

Every staged rotation shows both affected-axis arcs: yaw maps both
`x_n -> x_1` and `y_n -> y_1`, pitch maps both `x_1 -> x_2` and
`z_1 -> z_2`, and roll maps both `y_2 -> y_b` and `z_2 -> z_b`. The combined
page preserves all six arcs. Each staged and combined view also places a
color-matched curved rotation indicator near the positive end of the invariant
axis (`z_n`, `y_1`, or `x_2`). Angle symbols on the six basis-vector arcs sit
immediately beyond their arrowheads so each label identifies its associated
mapping rather than floating at the middle of the arc.

## Observed comparison

| Criterion | TikZ | Asymptote |
| --- | --- | --- |
| Final visual quality | Excellent | Excellent |
| Geometry model | Explicit trigonometric/projection expressions | Native 3-D vectors, rotations, planes, and camera |
| Typography | Native, searchable/selectable LaTeX text | LaTeX glyphs converted to crisp vector outlines by this render path |
| Final artifact | One two-page vector PDF | One two-page vector PDF assembled from two native vector renders |
| Integration with a LaTeX document | Direct | Straightforward PDF inclusion after a separate render-and-concatenate step |
| Editing small layout details | Very precise | Precise, but label placement still needs visual judgment |
| Editing angles/camera/3-D construction | More brittle because projection and placement are explicit | Cleaner because all intermediate geometry is derived from parameters |
| Windows build behavior in this trial | Reliable `latexmk` build | Reliable with explicit `-render 0`; automatic 3-D rendering stalled on translucent planes |

For this specific class of genuine three-dimensional attitude/frame figures,
Asymptote is the preferred long-term geometry engine. It keeps the
mathematical construction legible and makes changes to angles or camera view
substantially safer. The TikZ rendering from this trial is, however, the more
polished publication artifact today: it uses the page more efficiently, gives
the clearest old/new-axis hierarchy, and retains searchable LaTeX text.

Accordingly, retain the TikZ artifact for immediate document use and mature a
shared Asymptote style/layout layer before standardizing on Asymptote for a
family of 3-D figures. TikZ remains the preferred default for two-dimensional
mathematical diagrams, signal-flow diagrams, state machines, architecture
figures, and figures whose typography and layout must integrate directly with
an owning LaTeX document.

## Verification performed

- Both implementations were rendered as two-page vector PDFs with page 1
  containing the left-to-right staged construction and page 2 containing the
  combined construction. Asymptote's final PDF is a lossless concatenation of
  two independently rendered native 3-D pages because deferred 3-D objects
  were not isolated reliably by its simple `newpage()` path.
- Both previews were inspected after multiple layout iterations.
- None of the generated PDFs contains raster images.
- The TikZ PDF embeds searchable, Unicode-mapped Latin Modern fonts.
- The Asymptote PDFs represent text as vector outlines with the selected native
  3-D render path, so its text is not searchable or selectable.
- Both implementations received independent geometry checks against the declared
  `R_3 R_2 R_1` frame construction.
- Titles, labels, equations, and page bounds were checked for overlap and
  clipping.

The eventual repository-wide figure build should automate these checks and
produce a proof page at final publication scale, as recorded in the roadmap.
