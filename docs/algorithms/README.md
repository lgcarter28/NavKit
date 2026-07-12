# Algorithm Specifications

This directory contains focused implementation-oriented algorithm specs.

- [`navigator_ecef_v1/`](navigator_ecef_v1/) defines the first concrete ECEF
  INS/GNSS navigator contract before implementation.

These documents are narrower than the broader mathematical reference in
[`../navigation_reference/`](../navigation_reference/). They should answer:

- what algorithm are we implementing first;
- what conventions and approximations are intentionally selected;
- what behavior must be tested before the implementation is trusted;
- what generalizations are deliberately deferred.
